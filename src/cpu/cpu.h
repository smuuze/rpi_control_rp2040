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
 * @file    cpu.h
 * @author  Sebastian Lesse
 * @date    2022 / 07 / 25
 * @brief   basic definitions to be used on the Raspberry Pi Pcio 2022
 * 
 */

// --------------------------------------------------------------------------------

#ifndef _H_cpu_
#define _H_cpu_

// --------------------------------------------------------------------------------

#include <stdint.h>
// #include <string.h>
// #include <time.h>
// #include <unistd.h>
// #include <stdio.h>
// #include <stdlib.h>

// --------------------------------------------------------------------------------

#ifndef config_WATCHDOG_TIMEOUT_VALUE
#define config_WATCHDOG_TIMEOUT_VALUE
#endif

// --------------------------------------------------------------------------------

#define watchdog_enable()       do{}while(0)
#define watchdog()              do{}while(0)

// --------------------------------------------------------------------------------

#ifndef config_SLEEP_MODE
#define config_SLEEP_MODE
#endif

// --------------------------------------------------------------------------------

#define CPU_PREPARE_SLEEP_MODE()        do{}while(0)
#define CPU_ENTER_SLEEP_MODE()          do{}while(0);
#define CPU_DEACTIVATE_SLEEP_MODE()     do{}while(0)

// --------------------------------------------------------------------------------

#ifndef NULL
#define NULL 0
#endif

// --------------------------------------------------------------------------------

#define ATOMIC_OPERATION(operation)     operation

// --------------------------------------------------------------------------------

/**
 * @brief Base address of the gpio status and control
 * register in the RP2040 memory-map
 * 
 */
#define RP2040_IO_REG_BASE_ADDRESS  __UNSIGNED(0x40014000)

/**
 * @brief Basse address of the gpio pads register
 * in the RP2040 memory-map
 * 
 */
#define RP2040_PADS_BASE_ADDRESS   __UNSIGNED(0x4001c000)

/**
 * @brief Offset of the first gpio inside of the PADs register
 * 
 */
#define RP2040_PADS_GPIO_OFFSET     __UNSIGNED(0x04)

/**
 * @brief Base-address of the RP2040 SIO register
 * 
 */
#define RP2040_SIO_REG_BASE_ADDRESS __UNSIGNED(0xd0000000)

/**
 * @brief Offset of the gpio register inside of the SIO STRUCTURE
 * 
 */
#define RP2040_SIO_REG_GPIO_OFFSET  __UNSIGNED(0x04)

/**
 * @brief Offset of the NVIC IRQ CLEAR PENDING register
 * of the RP2040
 */
#define RP2040_IRQ_CLEAR_PENDING_REG_ADDRESS __UNSIGNED(0xe0000000) + __UNSIGNED(0x0000e280)

/**
 * @brief Offset of the NVIC IRQ SET ENABLE register
 * of the RP2040
 */
#define RP2040_IRQ_SET_ENABLE_REG_ADDRESS __UNSIGNED(0xe0000000) + __UNSIGNED(0x0000e100)

/**
 * @brief Offset of the NVIC IRQ SET ENABLE register
 * of the RP2040
 */
#define RP2040_IRQ_CLEAR_ENABLE_REG_ADDRESS __UNSIGNED(0xe0000000) + __UNSIGNED(0x0000e180)

/**
 * @brief Offset of the Timer Register of the RP2040
 * 
 */
#define RP2040_TIMER_REG_ADDRESS __UNSIGNED(0x40054000)

// --------------------------------------------------------------------------------

/**
 * @brief The number of gpio-pins of the RP2040
 * 
 */
#define RP2040_NUM_GPIO_PINS        30

// --------------------------------------------------------------------------------

/**
 * @brief Number of IRQs available on the RP2040 MCU
 * @see Datasheet section 2.3.2 on page 60
 * 
 */
#define RP2040_NUM_IRQ              26

// --------------------------------------------------------------------------------

/**
 * @brief Basse-address of the uart0 control-register of the RP2040
 */
#define RP2040_UART0_REG_BASE_ADDRESS       __UNSIGNED(0x40034000)

/**
 * @brief Basse-address of the uart1 control-register of the RP2040
 */
#define RP2040_UART1_REG_BASE_ADDRESS       __UNSIGNED(0x40038000)

// --------------------------------------------------------------------------------

/**
 * @brief Number of the RP2040 IRQs
 * @see Datasheet section 2.3.2 on page 60
 * 
 */

#define IRQ_NUM_TIMER0                      __UNSIGNED(0)
#define IRQ_NUM_UART0                       __UNSIGNED(20)

// --------------------------------------------------------------------------------

/**
 * @brief Used for register of the rp2040 that are read-only
 * 
 */
typedef const volatile uint32_t io_ro_32;

/**
 * @brief Used for registers of the rp2040 that can be read and written.
 * 
 */
typedef volatile uint32_t io_rw_32;

/**
 * @brief Used for registers of the rp2040 that can only be written.
 * 
 */
typedef volatile uint32_t io_wo_32;

// --------------------------------------------------------------------------------

/**
 * @brief Short for uint8_t
 * 
 */
typedef uint8_t u8;

/**
 * @brief Short for uint16_t
 * 
 */
typedef uint16_t u16;

/**
 * @brief Short for uint32_t
 * 
 */
typedef uint32_t u32;

/**
 * @brief Short for int8_t
 * 
 */
typedef int8_t i8;

/**
 * @brief Short for int16_t
 * 
 */
typedef int16_t i16;

/**
 * @brief Short for int32_t
 * 
 */
typedef int32_t i32;

/**
 * @brief Short for uint64_t
 * 
 */
typedef uint64_t u64;

/**
 * @brief Short for uint8_t
 * 
 */
typedef uint8_t ux8;

/**
 * @brief Short for uint16_t
 * 
 */
typedef uint16_t ux16;

/**
 * @brief Short for uint32_t
 * 
 */
typedef uint32_t ux32;

/**
 * @brief Short for int64_t
 * 
 */
typedef int64_t i64;

// --------------------------------------------------------------------------------

#ifndef __UNUSED__
#define __UNUSED__ __attribute__((unused))
#endif

#ifndef __USED__
#define __USED__ __attribute__((__used__))
#endif

#ifndef __PACKED__
#define __PACKED__ __attribute__((packed))
#endif

// --------------------------------------------------------------------------------

#ifndef __SECTION
#define __SECTION(section_name) __attribute__((used, section(section_name)))
#endif

// --------------------------------------------------------------------------------

#ifndef __CONCAT
#define __CONCAT(a, b) a ## b
#endif

#ifndef __STRING
#define __STRING(x) #x
#endif

#ifndef __UNSIGNED
#define __UNSIGNED(x)   x##u
#endif

#ifndef __ALWAYS_INLINE
#define __ALWAYS_INLINE inline
#endif

// --------------------------------------------------------------------------------

/**
 * @brief Sets a breakpoint
 * 
 */
static inline void cpu_breakpoint(void) {
     __asm__("bkpt #0");
 }

// --------------------------------------------------------------------------------

/**
 * @brief function to check if a loop runs too long.
 * At this moment this fucntion does nothing.
 * 
 */
 static inline void cpu_watch_forever_loop(void) {

}

// --------------------------------------------------------------------------------

/**
 * @brief Atomic set of bits into a writeable register.
 * 
 * @param address reference to the rigister where to set the bits
 * @param bit_mask bits to set in the register
 */
__ALWAYS_INLINE static void cpu_atomic_bit_set(io_rw_32* p_register, u32 bit_mask) {

    /**
     * @brief Uses the optional address-offset to realize a bit-set only
     * See datasheet ch. 2.1.2 Atomic Register Access
     * 
     */
    *(io_rw_32*) ((void*)(0x2000 | (u32)((volatile void*) p_register ))) = bit_mask;
}

// --------------------------------------------------------------------------------

/**
 * @brief Atomic clear of bits into a writeable register.
 * 
 * @param address reference to the rigister where to clear the bits
 * @param bit_mask bits to clear in the register
 */
__ALWAYS_INLINE static void cpu_atomic_bit_clear(io_rw_32* p_register, u32 bit_mask) {

    /**
     * @brief Uses the optional address-offset to realize a bit-clear only
     * See datasheet ch. 2.1.2 Atomic Register Access
     * 
     */
    *(io_rw_32*) ((void*)(0x3000 | (u32)((volatile void*) p_register ))) = bit_mask;
}

// --------------------------------------------------------------------------------

/**
 * @brief Atomic xor of bits into a writeable register.
 * 
 * @param address reference to the rigister where to clear the bits
 * @param bit_mask bits to xor in the register
 */
__ALWAYS_INLINE static void cpu_atomic_bit_xor(io_rw_32* p_register, u32 bit_mask) {

    /**
     * @brief Uses the optional address-offset to realize a bit-xor only
     * See datasheet ch. 2.1.2 Atomic Register Access
     * 
     */
    *(io_rw_32*) ((void*)(0x1000 | (u32)((volatile void*) p_register ))) = bit_mask;
}

// --------------------------------------------------------------------------------

/**
 * @brief Replaces the bits of mask with the given bit-value.
 * 
 * @param address reference to the rigister where to replace the bits
 * @param bit_mask bits to be replaced
 * @param bit_value values to use for replacement
 */
__ALWAYS_INLINE static void cpu_bit_replace(io_rw_32* p_register, u32 bit_mask, u32 bit_value) {
    cpu_atomic_bit_xor(p_register, (*p_register ^ bit_value) & bit_mask);
}

// --------------------------------------------------------------------------------

#endif // _H_cpu_

// --------------------------------------------------------------------------------
