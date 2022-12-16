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
 * @file    rp2040_reset.h
 * @author  Sebastian Lesse
 * @date    2022 / 08 / 15
 * @brief   Short description of this file
 * 
 */

// --------------------------------------------------------------------------------

#ifndef _H_rp2040_reset_
#define _H_rp2040_reset_

// --------------------------------------------------------------------------------

#include "cpu.h"

// --------------------------------------------------------------------------------

// Reset control
#define CPU_RESET_SUB_USB_CTRL      0x01000000
#define CPU_RESET_SUB_UART_01       0x00800000
#define CPU_RESET_SUB_UART_00       0x00400000
#define CPU_RESET_SUB_TIMER         0x00200000
#define CPU_RESET_SUB_TBMAN         0x00100000
#define CPU_RESET_SUB_SYSINFO       0x00080000
#define CPU_RESET_SUB_SYSCFG        0x00040000
#define CPU_RESET_SUB_SPI_01        0x00020000
#define CPU_RESET_SUB_SPI_00        0x00010000
#define CPU_RESET_SUB_RTC           0x00008000
#define CPU_RESET_SUB_PWM           0x00004000
#define CPU_RESET_SUB_PLL_USB       0x00002000
#define CPU_RESET_SUB_PLL_SYS       0x00001000
#define CPU_RESET_SUB_PIO_01        0x00000800
#define CPU_RESET_SUB_PIO_00        0x00000400
#define CPU_RESET_SUB_PADS_QSPI     0x00000200
#define CPU_RESET_SUB_PDAS_BANK_00  0x00000100
#define CPU_RESET_SUB_JTAG          0x00000080
#define CPU_RESET_SUB_IO_QSPI       0x00000040
#define CPU_RESET_SUB_IO_BANK_00    0x00000020
#define CPU_RESET_SUB_I2C_01        0x00000010
#define CPU_RESET_SUB_I2C_00        0x00000008
#define CPU_RESET_SUB_DMA           0x00000004
#define CPU_RESET_SUB_BUS_CTRL      0x00000002
#define CPU_RESET_SUB_ADC           0x00000001

// --------------------------------------------------------------------------------

/**
 * @brief Baseaddress of the reset register of the RP2040
 * 
 */
#define RP2040_RESET_REGISTER_BASE 0x4000c000 

// --------------------------------------------------------------------------------

typedef struct {

    /**
     * @brief Reset control. If a bit is set it means the peripheral is in reset.
     * 0 means the peripheral’s reset is deasserted.
     * 
     * Bits     Name        Description         Type    Reset
     * -------------------------------------------------------
     * 31:25    Reserved.   -                   -       -
     * 24       USBCTRL                         RW      0x1
     * 23       UART1                           RW      0x1
     * 22       UART0                           RW      0x1
     * 21       TIMER                           RW      0x1
     * 20       TBMAN                           RW      0x1
     * 19       SYSINFO                         RW      0x1
     * 18       SYSCFG                          RW      0x1
     * 17       SPI1                            RW      0x1
     * 16       SPI0                            RW      0x1
     * 15       RTC                             RW      0x1
     * 14       PWM                             RW      0x1
     * 13       PLL_USB                         RW      0x1
     * 12       PLL_SYS                         RW      0x1
     * 11       PIO1                            RW      0x1
     * 10       PIO0                            RW      0x1
     * 9        PADS_QSPI                       RW      0x1
     * 8        PADS_BANK0                      RW      0x1
     * 7        JTAG                            RW      0x1
     * 6        IO_QSPI                         RW      0x1
     * 5        IO_BANK0                        RW      0x1
     * 4        I2C1                            RW      0x1
     * 3        I2C0                            RW      0x1
     * 2        DMA                             RW      0x1
     * 1        BUSCTRL                         RW      0x1
     * 0        ADC                             RW      0x1
     */
    io_rw_32 reset;

    /**
     * @brief Watchdog select. If a bit is set then the watchdog will reset
     * this peripheral when the watchdog fires.
     * 
     * Bits     Name        Description         Type    Reset
     * -------------------------------------------------------
     * 31:25    Reserved.   -                   -       -
     * 24       USBCTRL                         RW      0x0
     * 23       UART1                           RW      0x0
     * 22       UART0                           RW      0x0
     * 21       TIMER                           RW      0x0
     * 20       TBMAN                           RW      0x0
     * 19       SYSINFO                         RW      0x0
     * 18       SYSCFG                          RW      0x0
     * 17       SPI1                            RW      0x0
     * 16       SPI0                            RW      0x0
     * 15       RTC                             RW      0x0
     * 14       PWM                             RW      0x0
     * 13       PLL_USB                         RW      0x0
     * 12       PLL_SYS                         RW      0x0
     * 11       PIO1                            RW      0x0
     * 10       PIO0                            RW      0x0
     * 9        PADS_QSPI                       RW      0x0
     * 8        PADS_BANK0                      RW      0x0
     * 7        JTAG                            RW      0x0
     * 6        IO_QSPI                         RW      0x0
     * 5        IO_BANK0                        RW      0x0
     * 4        I2C1                            RW      0x0
     * 3        I2C0                            RW      0x0
     * 2        DMA                             RW      0x0
     * 1        BUSCTRL                         RW      0x0
     * 0        ADC                             RW      0x0
     */
    io_rw_32 wdsel;

    /**
     * @brief Reset done.
     * If a bit is set then a reset done signal has been returned by the peripheral.
     * This indicates that the peripheral’s registers are ready to be accessed.
     * 
     * Bits     Name        Description         Type    Reset
     * -------------------------------------------------------
     * 31:25    Reserved.   -                   -       -
     * 24       USBCTRL                         RO      0x0
     * 23       UART1                           RO      0x0
     * 22       UART0                           RO      0x0
     * 21       TIMER                           RO      0x0
     * 20       TBMAN                           RO      0x0
     * 19       SYSINFO                         RO      0x0
     * 18       SYSCFG                          RO      0x0
     * 17       SPI1                            RO      0x0
     * 16       SPI0                            RO      0x0
     * 15       RTC                             RO      0x0
     * 14       PWM                             RO      0x0
     * 13       PLL_USB                         RO      0x0
     * 12       PLL_SYS                         RO      0x0
     * 11       PIO1                            RO      0x0
     * 10       PIO0                            RO      0x0
     * 9        PADS_QSPI                       RO      0x0
     * 8        PADS_BANK0                      RO      0x0
     * 7        JTAG                            RO      0x0
     * 6        IO_QSPI                         RO      0x0
     * 5        IO_BANK0                        RO      0x0
     * 4        I2C1                            RO      0x0
     * 3        I2C0                            RO      0x0
     * 2        DMA                             RO      0x0
     * 1        BUSCTRL                         RO      0x0
     * 0        ADC                             RO      0x0
     * 
     */
    io_ro_32 reset_done;

} RP2040_RESET_REG;

// --------------------------------------------------------------------------------

/**
 * @brief Resets the subsystems taht are given via its bits.
 * This function blocks until all subsystems have been reseted.
 * 
 * @param sub_system_bits bitmask of subsystems to be reseted
 */
void rp2040_reset_subsystem(u32 sub_system_bits);

// --------------------------------------------------------------------------------

/**
 * @brief Resets the UART0 sub-system and waits until reset is finisehd.
 * 
 */
static inline void rp2040_reset_uart0(void) {
    rp2040_reset_subsystem(CPU_RESET_SUB_UART_00);
}

/**
 * @brief Resets the USB-PLL sub-system and waits until reset is finisehd.
 * 
 */
static inline void rp2040_reset_pll_usb(void) {
    rp2040_reset_subsystem(CPU_RESET_SUB_PLL_USB);
}

/**
 * @brief Resets the SYS-PLL sub-system and waits until reset is finisehd.
 * 
 */
static inline void rp2040_reset_pll_sys(void) {
    rp2040_reset_subsystem(CPU_RESET_SUB_PLL_SYS);
}

// --------------------------------------------------------------------------------

#endif // _H_rp2040_reset_

// --------------------------------------------------------------------------------
