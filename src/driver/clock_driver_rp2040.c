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
 * @file    clock_driver_RP2040.c
 * @author  Sebastian Lesse
 * @date    2022 / 07 / 25
 * @brief   Initialization of the clock-module of the Raspberry Pi Pico
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

#include "rp2040_reset.h"

// --------------------------------------------------------------------------------

#include BOARD_DESCRIPTION_FILE

// --------------------------------------------------------------------------------

#include "driver/clock/clock_driver_interface.h"

// --------------------------------------------------------------------------------

/**
 * @brief Basse-address of the watchdog control-register of the RP2040
 */
#define RP2040_WATCHDOG_REG_BASE_ADDRESS    __UNSIGNED(0x40058000)

// --------------------------------------------------------------------------------

/**
 * @brief Basse-address of the clock control-register of the RP2040
 */
#define RP2040_CLOCK_REG_BASE_ADDRESS       __UNSIGNED(0x40008000)

// --------------------------------------------------------------------------------

/**
 * @brief Basse-address of the XOSC control-register of the RP2040
 */
#define RP2040_XOSC_REG_BASE_ADDRESS        __UNSIGNED(0x40024000)

// --------------------------------------------------------------------------------

/**
 * @brief Basse-address of the SYS-PLL control-register of the RP2040
 */
#define RP2040_PLL_SYS_BASE_ADDRESS         __UNSIGNED(0x40028000)

// --------------------------------------------------------------------------------

/**
 * @brief Basse-address of the USB-PLL control-register of the RP2040
 */
#define RP2040_PLL_USB_BASE_ADDRESS         __UNSIGNED(0x4002c000)

// --------------------------------------------------------------------------------

#define WATCHDOG_REG_TICK_ENABLE            __UNSIGNED(0x00000200)

// --------------------------------------------------------------------------------

#ifndef MHZ
#define MHZ(clk)    (clk * 1000000)
#endif

// --------------------------------------------------------------------------------

/**
 * @brief PICO_CONFIG: PICO_XOSC_STARTUP_DELAY_MULTIPLIER,
 * Multiplier to lengthen xosc startup delay to accommodate slow-starting oscillators,
 * type=int, min=1, default=1, group=hardware_xosc
 * 
 */
#ifndef RP2040_CLOCK_DRIVER_XOSC_STARTUP_DELAY_MULTIPLIER
#define RP2040_CLOCK_DRIVER_XOSC_STARTUP_DELAY_MULTIPLIER  __UNSIGNED(1)
#endif

/**
 * @brief PICO_CONFIG: XOSC_MHZ
 * The crystal oscillator frequency in Mhz,
 * type=int, default=12, advanced=true, group=hardware_base
 * 
 */
#ifndef XOSC_MHZ
#define XOSC_MHZ    __UNSIGNED(12)
#endif

// --------------------------------------------------------------------------------

#define RP2040_CLOCK_DRIVER_XOSC_CTRL_ENABLE_LSB                    __UNSIGNED(12)

#define RP2040_CLOCK_DRIVER_XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ      __UNSIGNED(0xaa0)
#define RP2040_CLOCK_DRIVER_XOSC_CTRL_ENABLE                        (__UNSIGNED(0xfab) << RP2040_CLOCK_DRIVER_XOSC_CTRL_ENABLE_LSB)
#define RP2040_CLOCK_DRIVER_XOSC_STATUS_STABLE                      __UNSIGNED(0x80000000)

#define RP2040_CLOCK_DRIVER_XOSC_STARTUP_DELAY                      (((( MHZ(XOSC_MHZ) / 1000) + 128) / 256) * RP2040_CLOCK_DRIVER_XOSC_STARTUP_DELAY_MULTIPLIER)

// --------------------------------------------------------------------------------

#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS     __UNSIGNED(0x0)
#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX    __UNSIGNED(0x1)
#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC           __UNSIGNED(0x2)

#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS           __UNSIGNED(0x0)
#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB     __UNSIGNED(0x0)
#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB     __UNSIGNED(0x0)
#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB     __UNSIGNED(0x0)

#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_REF_CTRL_SRC_BITS                        __UNSIGNED(0x00000003)

#define RP2040_CLOCK_CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS                         __UNSIGNED(0x00000800)

#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB                      __UNSIGNED(5)
#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS                     __UNSIGNED(0x000000e0)

#define RP2040_CLOCK_DRIVER_CLOCKS_CLK_REF_CTRL_SRC_LSB                         __UNSIGNED(0)

// --------------------------------------------------------------------------------

/**
 * @brief Selects the clock source glitchlessly, can be changed on- the-fly
 * 
 */
#define RP2040_CLOCK_DRIVER_SYS_CTRL_REG_SRC        __UNSIGNED(0x00000001)
#define RP2040_CLOCK_DRIVER_REF_CTRL_REG_SRC        __UNSIGNED(0x00000003)

// --------------------------------------------------------------------------------

/**
 * @brief Number of clocks available on the RP2040
 * 
 */
#define RP2040_CLOCK_COUNT              __UNSIGNED(10)

#define RP2040_CLOCK_DRIVER_SRC_GPOUT0  __UNSIGNED(0)
#define RP2040_CLOCK_DRIVER_SRC_GPOUT1  __UNSIGNED(1)
#define RP2040_CLOCK_DRIVER_SRC_GPOUT2  __UNSIGNED(2)
#define RP2040_CLOCK_DRIVER_SRC_GPOUT3  __UNSIGNED(3)
#define RP2040_CLOCK_DRIVER_SRC_REF     __UNSIGNED(4)
#define RP2040_CLOCK_DRIVER_SRC_SYS     __UNSIGNED(5)
#define RP2040_CLOCK_DRIVER_SRC_PERI    __UNSIGNED(6)
#define RP2040_CLOCK_DRIVER_SRC_USB     __UNSIGNED(7)
#define RP2040_CLOCK_DRIVER_SRC_ADC     __UNSIGNED(8)
#define RP2040_CLOCK_DRIVER_SRC_RTC     __UNSIGNED(9)

// --------------------------------------------------------------------------------

#define RP2040_CLOCK_DRIVER_PLL_PRIM_POSTDIV1_LSB    __UNSIGNED(16)
#define RP2040_CLOCK_DRIVER_PLL_PRIM_POSTDIV2_LSB    __UNSIGNED(12)

#define RP2040_CLOCK_DRIVER_PLL_PWR_POWER_DOWN      __UNSIGNED(0x00000001)
#define RP2040_CLOCK_DRIVER_PLL_PWR_VCO_POWER_DOWN  __UNSIGNED(0x00000020)
#define RP2040_CLOCK_DRIVER_PLL_CS_LOCK             __UNSIGNED(0x80000000)
#define RP2040_CLOCK_DRIVERPLL_PWR_POSTDIVPD        __UNSIGNED(0x00000008)

#define RP2040_CLOCK_DRIVER_IS_GLITCHLESS_MUX(clk_index)    (clk_index == RP2040_CLOCK_DRIVER_SRC_SYS || clk_index == RP2040_CLOCK_DRIVER_SRC_REF)

// --------------------------------------------------------------------------------

/**
 * @brief Structure to control the XOSC register of the RP2040
 * @see RP2040 Datasheet (build-date: 2021-03-05 / build-version: 9bf4a25-clean)
 * 
 */
typedef struct {
    
    /**
     * @brief Crystal Oscillator Control
     * See datasheet ch. 2.16.7 page 242-243
     * 
     */
    io_rw_32 ctrl;

    /**
     * @brief Crystal Oscillator Status
     * See datasheet ch. 2.16.7 page 242-243
     * 
     */
    io_rw_32 status;

    /**
     * @brief Crystal Oscillator pause control
     * See datasheet ch. 2.16.7 page 243-244
     * 
     */
    io_rw_32 dormant;

    /**
     * @brief Controls the startup delay
     * See datasheet ch. 2.16.7 page 244
     * 
     */
    io_rw_32 startup;

    uint32_t _pad0[3];

    /**
     * @brief A down counter running at the xosc frequency which counts to zero and stops
     * See datasheet ch. 2.16.7 page 244
     * 
     */
    io_rw_32 count;

} RP2040_XOSC_REG;

// --------------------------------------------------------------------------------

/**
 * @brief Structure to control the CLOCK-PLL register of the RP2040
 * @see RP2040 Datasheet (build-date: 2021-03-05 / build-version: 9bf4a25-clean)
 * 
 */
typedef struct {
    
    /**
     * @brief Control and Status
     * See datasheet ch. 2.18.4 page 256
     * 
     */
    io_rw_32 cs;

    /**
     * @brief Controls the PLL power modes
     * See datasheet ch. 2.18.4 page 256
     * 
     */
    io_rw_32 pwr;

    /**
     * @brief Feedback divisor
     * See datasheet ch. 2.18.4 page 256-257
     * 
     */
    io_rw_32 fbdiv_int;

    /**
     * @brief Controls the PLL post dividers for the primary output
     * See datasheet ch. 2.18.4 page 257
     * 
     */
    io_rw_32 prim;

} RP2040_CLOCK_PLL_REG;

// --------------------------------------------------------------------------------

/**
 * @brief Sub-Structure to control the CLOCK-CONTROL and CLOCK-DEVIDER register of the RP2040
 * @see RP2040 Datasheet (build-date: 2021-03-05 / build-version: 9bf4a25-clean)
 * 
 */
typedef struct {
    
    /**
     * @brief Clock control, can be changed on-the-fly (except for auxsrc)
     * See datasheet ch. 2.15.7 page 220-231
     * 
     */
    io_rw_32 ctrl;

    /**
     * @brief Clock divisor, can be changed on-the-fly
     * See datasheet ch. 2.15.7 page 220-231
     * 
     */
    io_rw_32 div;

    /**
     * @brief Indicates which SRC is currently selected by the glitchless mux (one-hot).
     * See datasheet ch. 2.15.7 page 220-231
     * 
     */
    io_ro_32 selected;

} RP2040_CLOCK_CONTROL_REG;

// --------------------------------------------------------------------------------

/**
 * @brief Sub-Structure to control the CLOCK-FC0 register of the RP2040
 * @see RP2040 Datasheet (build-date: 2021-03-05 / build-version: 9bf4a25-clean)
 * 
 */
typedef struct {
    
    /**
     * @brief Reference clock frequency in kHz
     * See datasheet ch. 2.15.7 page 231
     * 
     */
    io_rw_32 ref_khz;

    /**
     * @brief Minimum pass frequency in kHz.
     * This is optional. Set to 0 if you are not using RW 0x0000000 the pass/fail flags
     * See datasheet ch. 2.15.7 page 232
     * 
     */
    io_rw_32 min_khz;

    /**
     * @brief Maximum pass frequency in kHz
     * This is optional. Set to 0x1ffffff if you are RW 0x1ffffff not using the pass/fail flags
     * See datasheet ch. 2.15.7 page 232
     * 
     */
    io_rw_32 max_khz;

    /**
     * @brief Delays the start of frequency counting to allow the mux to settle
     * Delay is measured in multiples of the reference clock period
     * See datasheet ch. 2.15.7 page 232
     * 
     */
    io_rw_32 delay;

    /**
     * @brief The test interval is 0.98us * 2**interval,
     * but let’s call it 1us * 2**interval
     * The default gives a test interval of 250us
     * See datasheet ch. 2.15.7 page 232
     * 
     */
    io_rw_32 interval;

    /**
     * @brief Clock sent to frequency counter, set to 0 when not required
     * See datasheet ch. 2.15.7 page 232-233
     * 
     */
    io_rw_32 src;

    /**
     * @brief Frequency counter status
     * See datasheet ch. 2.15.7 page 233
     * 
     */
    io_ro_32 status;

    /**
     * @brief Result of frequency measurement,
     * only valid when status_done=1
     * See datasheet ch. 2.15.7 page 233-234
     * 
     */
    io_ro_32 result;

} RP2040_CLOCK_FC0_REG;

// --------------------------------------------------------------------------------

/**
 * @brief Structure to control the CLOCK register of the RP2040
 * @see RP2040 Datasheet (build-date: 2021-03-05 / build-version: 9bf4a25-clean)
 * 
 */
typedef struct {

    RP2040_CLOCK_CONTROL_REG clk[RP2040_CLOCK_COUNT]; // 10

    /**
     * @brief enable / (force) clear of resus
     * See datasheet ch. 2.15.7 page 231
     * 
     */
    io_rw_32 resus_ctrl;

    /**
     * @brief Clock has been resuscitated, correct the error then send RO 0x0 ctrl_clear=1
     * See datasheet ch. 2.15.7 page 231
     * 
     */
    io_rw_32 resus_status;

    /**
     * @brief 
     * 
     */
    RP2040_CLOCK_FC0_REG fc0;

    /**
     * @brief enable clock in wake mode
     * See datasheet ch. 2.15.7 page 234
     * 
     */
    io_rw_32 wake_en0;

    /**
     * @brief enable clock in wake mode
     * See datasheet ch. 2.15.7 page 235
     * 
     */
    io_rw_32 wake_en1;

    /**
     * @brief enable clock in sleep mode
     * See datasheet ch. 2.15.7 page 235
     * 
     */
    io_rw_32 sleep_en0;

    /**
     * @brief enable clock in sleep mode
     * See datasheet ch. 2.15.7 page 236
     * 
     */
    io_rw_32 sleep_en1;

    /**
     * @brief indicates the state of the clock enable
     * See datasheet ch. 2.15.7 page 237
     * 
     */
    io_ro_32 enabled0;

    /**
     * @brief indicates the state of the clock enable
     * See datasheet ch. 2.15.7 page 238
     * 
     */
    io_ro_32 enabled1;

    /**
     * @brief Raw Interrupts
     * See datasheet ch. 2.15.7 page 239
     * 
     */
    io_ro_32 intr;

    /**
     * @brief Interrupt Enable
     * See datasheet ch. 2.15.7 page 239
     * 
     */
    io_rw_32 inte;

    /**
     * @brief Interrupt Force
     * See datasheet ch. 2.15.7 page 239
     * 
     */
    io_rw_32 intf;

    /**
     * @brief Interrupt status after masking & forcing
     * See datasheet ch. 2.15.7 page 239
     * 
     */
    io_ro_32 ints;

} RP2040_CLOCK_REG;

// --------------------------------------------------------------------------------

/**
 * @brief Structure to control the WATCHDOG register of the RP2040
 * @see RP2040 Datasheet (build-date: 2021-03-05 / build-version: 9bf4a25-clean)
 * 
 */
typedef struct {

    /**
     * @brief Watchdog control.
     * The rst_wdsel register determines which subsystems
     * are reset when the watchdog is triggered.
     * The watchdog can be triggered in software.
     * See datasheet ch. 4.7.6 page 568
     * 
     */
    io_rw_32 ctrl;

    /**
     * @brief Load the watchdog timer.
     * The maximum setting is 0xffffff which corresponds to 0xffffff / 2
     * ticks before triggering a watchdog reset (see errata RP2040-E1).
     * See datasheet ch. 4.7.6 page 569
     * 
     */
    io_wo_32 load;

    /**
     * @brief Logs the reason for the last reset. Both bits are zero for the case of a hardware reset.
     * See datasheet ch. 4.7.6 page 569
     * 
     */
    io_ro_32 reason;

    /**
     * @brief Scratch register. Information persists through soft reset of the chip.
     * See datasheet ch. 4.7.6 page 569
     * 
     */
    io_rw_32 scratch[8];

    /**
     * @brief Controls the tick generator
     * See datasheet ch. 4.7.6 page 570
     * 
     */
    io_rw_32 tick;

} RP2040_WATCHDOG_REG;

// --------------------------------------------------------------------------------

/**
 * @brief Returns pointer to access the watchdog register
 * 
 * @return Pointer to the watchdog register
 */
static inline RP2040_WATCHDOG_REG* clock_driver_get_watchdog_reg(void) {
    return ((RP2040_WATCHDOG_REG*)RP2040_WATCHDOG_REG_BASE_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @brief Returns pointer to access the clock register
 * 
 * @return Pointer to the clock register
 */
static inline RP2040_CLOCK_REG* clock_driver_get_clock_reg(void) {
    return ((RP2040_CLOCK_REG*)RP2040_CLOCK_REG_BASE_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @brief Returns pointer to access the XOSC register
 * 
 * @return Pointer to the XOSC register
 */
static inline RP2040_XOSC_REG* clock_driver_get_xosc_reg(void) {
    return ((RP2040_XOSC_REG*)RP2040_XOSC_REG_BASE_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @brief Returns pointer to access the USB-PLL register
 * 
 * @return Pointer to the USB-PLL register
 */
static inline RP2040_CLOCK_PLL_REG* clock_driver_get_usb_pll_reg(void) {
    return ((RP2040_CLOCK_PLL_REG*)RP2040_PLL_USB_BASE_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @brief Returns pointer to access the SYS-PLL register
 * 
 * @return Pointer to the SYS-PLL register
 */
static inline RP2040_CLOCK_PLL_REG* clock_driver_get_sys_pll_reg(void) {
    return ((RP2040_CLOCK_PLL_REG*)RP2040_PLL_SYS_BASE_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @brief Configured frequency of all available clocks.
 * clock_driver_init() needs to be called. Otherwise this
 * array does not contain valid values.
 * 
 */
static u32 clock_configured_frequencies[RP2040_CLOCK_COUNT];

/**
 * @brief Get the configured frequency of the clock defined by index
 * 
 * @param index index of the clock of interesst
 * @return the actual configured frequencey of the clock
 */
static inline u32 clock_driver_get_frequency(u32 index) {
    return clock_configured_frequencies[index];
}

/**
 * @brief Sets the configured frequency of a clock
 * 
 * @param index index of the clock 
 * @param frequency frequency to set for the desired clock
 */
static inline void clock_driver_set_frequency(u32 index, u32 frequency) {
    clock_configured_frequencies[index] = frequency;
}

// --------------------------------------------------------------------------------

/**
 * @brief Initializes XOSC
 * 
 */
static void clock_driver_initialize_xosc(void) {

    // Assumes 1-15 MHz input, checked above.
    clock_driver_get_xosc_reg()->ctrl = RP2040_CLOCK_DRIVER_XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ;

    // Set xosc startup delay
    clock_driver_get_xosc_reg()->startup = RP2040_CLOCK_DRIVER_XOSC_STARTUP_DELAY;

    // Set the enable bit now that we have set freq range and startup delay
    cpu_atomic_bit_set(&clock_driver_get_xosc_reg()->ctrl, RP2040_CLOCK_DRIVER_XOSC_CTRL_ENABLE);

    // Wait for XOSC to be stable
    while( !(clock_driver_get_xosc_reg()->status & RP2040_CLOCK_DRIVER_XOSC_STATUS_STABLE) ) {
        //cpu_watch_forever_loop();
    }
}

// --------------------------------------------------------------------------------

/**
 * @brief 
 * 
 * @param pll 
 * @param refdiv 
 * @param vco_freq 
 * @param post_div1 
 * @param post_div2 
 */
static void clock_driver_initalize_pll(RP2040_CLOCK_PLL_REG* p_pll, u32 refdiv, u32 vco_freq, u32 post_div1, u32 post_div2) {

    u32 ref_mhz = XOSC_MHZ / refdiv;

    // What are we multiplying the reference clock by to get the vco freq
    // (The regs are called div, because you divide the vco output and compare it to the refclk)
    u32 fbdiv = vco_freq / MHZ(ref_mhz);

    // div1 feeds into div2 so if div1 is 5 and div2 is 2 then you get a divide by 10
    u32 pdiv = (post_div1 << RP2040_CLOCK_DRIVER_PLL_PRIM_POSTDIV1_LSB)
             | (post_div2 << RP2040_CLOCK_DRIVER_PLL_PRIM_POSTDIV2_LSB);

    // if (pll->cs & PLL_CS_LOCK_BITS) {
    //     if (refdiv == (pll->cs & PLL_CS_REFDIV_BITS)) {
    //         if (fbdiv == (pll->fbdiv_int & PLL_FBDIV_INT_BITS)) {
    //             if (pdiv == (pll->prim & (PLL_PRIM_POSTDIV1_BITS | PLL_PRIM_POSTDIV2_BITS))) {
    //                 // do not disrupt PLL that is already correctly configured and operating
    //                 return;
    //             }
    //         }
    //     }
    // }

    // Load VCO-related dividers before starting VCO
    p_pll->cs = refdiv;
    p_pll->fbdiv_int = fbdiv;

    // Turn on PLL
    cpu_atomic_bit_clear(
        &p_pll->pwr,
        RP2040_CLOCK_DRIVER_PLL_PWR_POWER_DOWN          // Main power
        | RP2040_CLOCK_DRIVER_PLL_PWR_VCO_POWER_DOWN    // VCO Power
    );  

    // Wait for PLL to lock
    while ( !(p_pll->cs & RP2040_CLOCK_DRIVER_PLL_CS_LOCK))  {
        cpu_watch_forever_loop();
    }

    // Set up post dividers
    p_pll->prim = pdiv;

    // Turn on post divider
    cpu_atomic_bit_clear(&p_pll->pwr, RP2040_CLOCK_DRIVERPLL_PWR_POSTDIVPD);
}

// --------------------------------------------------------------------------------

/**
 * @brief 
 * 
 * @param clk_index 
 * @param src 
 * @param auxsrc 
 * @param src_freq 
 * @param freq 
 */
static void clock_driver_configure_clock(u32 clk_index, u32 src, u32 auxsrc, u32 src_freq, u32 freq) {

    // Div register is 24.8 int.frac divider so multiply by 2^8 (left shift by 8)
    u32 div = (u32) (((u64) src_freq << 8) / freq);

    RP2040_CLOCK_CONTROL_REG* p_clock = &clock_driver_get_clock_reg()->clk[clk_index];

    // If increasing divisor, set divisor before source. Otherwise set source
    // before divisor. This avoids a momentary overspeed when e.g. switching
    // to a faster source and increasing divisor to compensate.
    if (div > p_clock->div) {
        p_clock->div = div;
    }

    if (RP2040_CLOCK_DRIVER_IS_GLITCHLESS_MUX(clk_index) && src == RP2040_CLOCK_DRIVER_CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX) {

        // If switching a glitchless slice (ref or sys) to an aux source, switch
        // away from aux *first* to avoid passing glitches when changing aux mux.
        // Assume (!!!) glitchless source 0 is no faster than the aux source.

        cpu_atomic_bit_clear(&p_clock->ctrl, RP2040_CLOCK_DRIVER_CLOCKS_CLK_REF_CTRL_SRC_BITS);
        while ( !(p_clock->selected & 1u) ) {
            cpu_watch_forever_loop();
        }

    } else {

        // If no glitchless mux, cleanly stop the clock to avoid glitches
        // propagating when changing aux mux. Note it would be a really bad idea
        // to do this on one of the glitchless clocks (clk_sys, clk_ref).

        // Disable clock. On clk_ref and clk_sys this does nothing,
        // all other clocks have the ENABLE bit in the same position.
        cpu_atomic_bit_clear(&p_clock->ctrl, RP2040_CLOCK_CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);

        if (clock_driver_get_frequency(clk_index) > 0) {

            // Delay for 3 cycles of the target clock, for ENABLE propagation.
            // Note XOSC_COUNT is not helpful here because XOSC is not
            // necessarily running, nor is timer... so, 3 cycles per loop:

            u32 delay_cyc = clock_driver_get_frequency(RP2040_CLOCK_DRIVER_SRC_SYS) / clock_driver_get_frequency(clk_index) + 1;
            asm volatile (
                ".syntax unified \n\t"
                "1: \n\t"
                "subs %0, #1 \n\t"
                "bne 1b"
                : "+r" (delay_cyc)
            );
        }
    }

    // Set aux mux first, and then glitchless mux if this clock has one
    cpu_bit_replace(
        &p_clock->ctrl,
        (auxsrc << RP2040_CLOCK_DRIVER_CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB),
        RP2040_CLOCK_DRIVER_CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS
    );

    if (RP2040_CLOCK_DRIVER_IS_GLITCHLESS_MUX(clk_index)) {

        cpu_bit_replace(
            &p_clock->ctrl,
            src << RP2040_CLOCK_DRIVER_CLOCKS_CLK_REF_CTRL_SRC_LSB,
            RP2040_CLOCK_DRIVER_CLOCKS_CLK_REF_CTRL_SRC_BITS
        );

        while ( !(p_clock->selected & (1u << src)) ) {
            cpu_watch_forever_loop();
        }
    }

    // Enable clock. On clk_ref and clk_sys this does nothing,
    // all other clocks have the ENABLE bit in the same position.
    cpu_atomic_bit_set(&p_clock->ctrl, RP2040_CLOCK_CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);

    // Now that the source is configured, we can trust that the user-supplied
    // divisor is a safe value.
    p_clock->div = div;

    // Store the configured frequency
    clock_driver_set_frequency(clk_index, (u32)(((u64) src_freq << 8) / div));
}

// --------------------------------------------------------------------------------

/**
 * @see clock_driver_interface.h#clock_driver_init
 * 
 */
void clock_driver_init(void) {

    // Start tick in watchdog
    // Important: This function also provides a tick reference to the timer
    clock_driver_get_watchdog_reg()->tick = XOSC_MHZ | WATCHDOG_REG_TICK_ENABLE;

    // Disable resus that may be enabled from previous software
    clock_driver_get_clock_reg()->resus_ctrl = 0;

    // Enable the xosc
    clock_driver_initialize_xosc();

    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    cpu_atomic_bit_clear(
        &clock_driver_get_clock_reg()->clk[RP2040_CLOCK_DRIVER_SRC_SYS].ctrl,
        RP2040_CLOCK_DRIVER_SYS_CTRL_REG_SRC
    );

    while (clock_driver_get_clock_reg()->clk[RP2040_CLOCK_DRIVER_SRC_SYS].selected != 0x1) {
        cpu_watch_forever_loop();
    }

    cpu_atomic_bit_clear(
        &clock_driver_get_clock_reg()->clk[RP2040_CLOCK_DRIVER_SRC_REF].ctrl,
        RP2040_CLOCK_DRIVER_REF_CTRL_REG_SRC
    );

    while (clock_driver_get_clock_reg()->clk[RP2040_CLOCK_DRIVER_SRC_REF].selected != 0x1) {
        cpu_watch_forever_loop();
    }

    rp2040_reset_pll_sys();
    clock_driver_initalize_pll(
        clock_driver_get_sys_pll_reg(),
        1,          // refdiv,
        MHZ(1500), // vco_freq,
        6,          // post_div1,
        2           // post_div2
    );

    rp2040_reset_pll_usb();
    clock_driver_initalize_pll(
        clock_driver_get_usb_pll_reg(),
        1,          // refdiv,
        MHZ(1200), // vco_freq,
        5,          // post_div1,
        5           // post_div2
    );

    // Configure clocks

    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    clock_driver_configure_clock(
        RP2040_CLOCK_DRIVER_SRC_REF,
        RP2040_CLOCK_DRIVER_CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
        0, // No aux mux
        MHZ(12),
        MHZ(12)
    );

    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clock_driver_configure_clock(
        RP2040_CLOCK_DRIVER_SRC_SYS,
        RP2040_CLOCK_DRIVER_CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        RP2040_CLOCK_DRIVER_CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        MHZ(125),
        MHZ(125)
    );

    // CLK USB = PLL USB (48MHz) / 1 = 48MHz
    clock_driver_configure_clock(
        RP2040_CLOCK_DRIVER_SRC_USB,
        0, // No GLMUX
        RP2040_CLOCK_DRIVER_CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        MHZ(48),
        MHZ(48)
    );

    // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
    clock_driver_configure_clock(
        RP2040_CLOCK_DRIVER_SRC_ADC,
        0, // No GLMUX
        RP2040_CLOCK_DRIVER_CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        MHZ(48),
        MHZ(48)
    );

    // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
    clock_driver_configure_clock(
        RP2040_CLOCK_DRIVER_SRC_RTC,
        0, // No GLMUX
        RP2040_CLOCK_DRIVER_CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        MHZ(48),
        46875
    );

    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    // Normally choose clk_sys or clk_usb
    clock_driver_configure_clock(
        RP2040_CLOCK_DRIVER_SRC_PERI,
        0,
        RP2040_CLOCK_DRIVER_CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        MHZ(125),
        MHZ(125)
    );
}

// --------------------------------------------------------------------------------

/**
 * @see rp2040_clock_frequencies.h#clock_driver_peripheral_clk_frequency
 */
u32 clock_driver_peripheral_clk_frequency(void) {
    return clock_driver_get_frequency(RP2040_CLOCK_DRIVER_SRC_PERI);
}

// --------------------------------------------------------------------------------
