/**
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @file    rtc_driver_rp2040.c
 * @author  Sebastian Lesse
 * @date    2022 / 07 / 25
 * @brief   Implementation of a Real-Time-Clock driver
 *          usaeable on the Raspberry-Pico 2022
 *          
 *          This implementation uses the ALARM0 register to generate an IRQ
 *          to count millisecond. The value of timestamp_ms will be incremented
 *          everytime the IRQ is raised. This value is used to be returned
 *          to callers asking for the actual system-time.
 * 
 */

#define TRACER_OFF

// --------------------------------------------------------------------------------

#ifdef TRACER_ON
#warning __WARNING__TRACER_ENABLED__WARNING__
#endif

// --------------------------------------------------------------------------------

#include "config.h"

// --------------------------------------------------------------------------------

#include "tracer.h"

// --------------------------------------------------------------------------------

#include "cpu.h"

// --------------------------------------------------------------------------------

#include "irq/irq_interface.h"
#include "driver/rtc/rtc_driver_interface.h"

// --------------------------------------------------------------------------------

#define RP2040_RTC_MS_TO_US(VALUE_MS)               (VALUE_MS * __UNSIGNED(1000))

// --------------------------------------------------------------------------------

#define RP2040_RTC_MS_TIMESTAMP_INCREMENT           __UNSIGNED(5)
#define RP2040_RTC_US_TIMER_INCREMENT               RP2040_RTC_MS_TO_US(RP2040_RTC_MS_TIMESTAMP_INCREMENT)

// --------------------------------------------------------------------------------

/**
 * @brief Structure to control the TIMER register of the RP2040
 * @see RP2040 Datasheet (build-date: 2021-03-05 / build-version: 9bf4a25-clean)
 * 
 */
typedef struct {

    /**
     * @brief Write Timer High-Value, TIMEHW (Bits 63 to 32)
     * See datasheet ch. 4.6.5 - TIMEHW Register on page 562
     * for more details.
     */
    io_rw_32 time_wh;

    /**
     * @brief Write Timer Low-Value, TIMELW (Bits 31 to 0)
     * See datasheet ch. 4.6.5 - TIMELW Register on page 562
     * for more details.
     */
    io_rw_32 time_l;

    /**
     * @brief Read Timer High-Value, TIMEHW (Bits 63 to 32)
     * See datasheet ch. 4.6.5 - TIMEHW Register on page 562
     * for more details.
     */
    io_ro_32 time_rh;

    /**
     * @brief Read Timer Low-Value, TIMELW (Bits 31 to 0)
     * See datasheet ch. 4.6.5 - TIMELW Register on page 562
     * for more details.
     */
    io_ro_32 time_rl;

    /**
     * @brief Set value at which the alarm (0 to 3) will fire
     */
    io_rw_32 alarm[4]; // 4

    /**
     * @brief Indicates the armed/disarmed status of each alarm
     * 0x0000000f [3:0]   : ARMED (0)
     * 
     */
    io_rw_32 armed;

    /**
     * @brief Raw read from bits 63:32 of time (no side effects)
     */
    io_ro_32 timerawh;

    /**
     * @brief Raw read from bits 31:0 of time (no side effects)
     */
    io_ro_32 timerawl;

    // 
    /**
     * @brief Set bits high to enable pause when the corresponding debug ports are active
     * 0x00000004 [2]     : DBG1 (1): Pause when processor 1 is in debug mode
     * 0x00000002 [1]     : DBG0 (1): Pause when processor 0 is in debug mode
     * 
     */
    io_rw_32 dbgpause;

    /**
     * @brief Set high to pause the timer
     * 0x00000001 [0]     : PAUSE (0)
     */
    io_rw_32 pause;

    /**
     * @brief Raw Interrupts
     * 0x00000008 [3]     : ALARM_3 (0)
     * 0x00000004 [2]     : ALARM_2 (0)
     * 0x00000002 [1]     : ALARM_1 (0)
     * 0x00000001 [0]     : ALARM_0 (0)
     */
    io_rw_32 intr;

    /**
     * @brief Interrupt Enable
     * 0x00000008 [3]     : ALARM_3 (0)
     * 0x00000004 [2]     : ALARM_2 (0)
     * 0x00000002 [1]     : ALARM_1 (0)
     * 0x00000001 [0]     : ALARM_0 (0)
     */
    io_rw_32 inte;

    /**
     * @brief Interrupt Force
     * 0x00000008 [3]     : ALARM_3 (0)
     * 0x00000004 [2]     : ALARM_2 (0)
     * 0x00000002 [1]     : ALARM_1 (0)
     * 0x00000001 [0]     : ALARM_0 (0)
     */
    io_rw_32 intf;

    /**
     * @brief Interrupt status after masking & forcing
     * 0x00000008 [3]     : ALARM_3 (0)
     * 0x00000004 [2]     : ALARM_2 (0)
     * 0x00000002 [1]     : ALARM_1 (0)
     * 0x00000001 [0]     : ALARM_0 (0)
     */
    io_ro_32 ints;

} RP2040_TIMER_REG;

// --------------------------------------------------------------------------------

/**
 * @brief Handler to incremetn the current timestamp_ms by one
 * on every milliseconds.
 */
void rtc_ms_tick_handler(void);

IRQ_BUILD_HANDLER(RTC_MS_TICK_IRQ_HANDLER, &rtc_ms_tick_handler, IRQ_NUM_TIMER0, IRQ_DISABLED)

// --------------------------------------------------------------------------------

/**
 * @brief The current system time in milliseconds
 * 
 */
static u32 timestamp_ms = 0;

// --------------------------------------------------------------------------------

/**
 * @brief Access the Timer-Regsiter of the RP2040
 * 
 * @return R/W reference to the RP2040 timer-register
 */
static inline RP2040_TIMER_REG* get_timer_reg(void) {
    return (RP2040_TIMER_REG*)(RP2040_TIMER_REG_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @see driver/rtc/rtc_interface.h#rtc_driver_init
 */
void rtc_driver_init(void) {
    DEBUG_PASS("rtc_driver_init()");

    get_timer_reg()->alarm[0] = get_timer_reg()->time_rl + RP2040_RTC_US_TIMER_INCREMENT;
    get_timer_reg()->inte |= __UNSIGNED(1);

    RTC_MS_TICK_IRQ_HANDLER_init();
    RTC_MS_TICK_IRQ_HANDLER_set_enabled(IRQ_ENABLED);
}

/**
 * @see driver/rtc/rtc_interface.h#rtc_driver_deinit
 */
void rtc_driver_deinit(void) {
    DEBUG_PASS("rtc_driver_deinit()");
}

// --------------------------------------------------------------------------------

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_gettime_u8
 */
u8 rtc_timer_gettime_u8(void) {
    return (u8)(timestamp_ms);
}

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_gettime_u16
 */
u16 rtc_timer_gettime_u16(void) {
    return (u16)(timestamp_ms);
}

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_gettime_u32
 */
u32 rtc_timer_gettime_u32(void) {
    return (u32)(timestamp_ms);
}

// --------------------------------------------------------------------------------

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_istimeup_u8
 */
u8 rtc_timer_istimeup_u8(u8 time_reference, u8 time_interval) {
    return (rtc_timer_elapsed_u8(time_reference) > time_interval) ? 1 : 0;
}

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_istimeup_u16
 */
u8 rtc_timer_istimeup_u16(u16 time_reference, u16 time_interval) {
    return (rtc_timer_elapsed_u16(time_reference) > time_interval) ? 1 : 0;
}

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_istimeup_u32
 */
u8 rtc_timer_istimeup_u32(u32 time_reference, u32 time_interval) {
    return (rtc_timer_elapsed_u32(time_reference) > time_interval) ? 1 : 0;
}

// --------------------------------------------------------------------------------

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_elapsed_u8
 */
u8 rtc_timer_elapsed_u8(u8 time_reference) {
    return rtc_timer_gettime_u8() - time_reference;
}

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_elapsed_u16
 */
u16 rtc_timer_elapsed_u16(u16 time_reference) {
    return rtc_timer_gettime_u16() - time_reference;
}

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_usleep
 */
u32 rtc_timer_elapsed_u32(u32 time_reference) {
    return rtc_timer_gettime_u32() - time_reference;
}

// --------------------------------------------------------------------------------

void rtc_ms_tick_handler(void) {
    get_timer_reg()->intr = __UNSIGNED(1);
    get_timer_reg()->alarm[0] += RP2040_RTC_US_TIMER_INCREMENT;
    timestamp_ms += RP2040_RTC_MS_TIMESTAMP_INCREMENT;
}

// --------------------------------------------------------------------------------

/**
 * @see driver/rtc/rtc_interface.h#rtc_timer_usleep
 */
void rtc_timer_usleep(u32 delay_us) {

    // we only allow 31 bits, otherwise we could have a race in the loop below with
    // values very close to 2^32
    uint32_t start = get_timer_reg()->timerawl;
    while (get_timer_reg()->timerawl - start < delay_us) {
        // do nothing - just wait
    }
}

// --------------------------------------------------------------------------------
