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
 * @file    unittest_uart_driver_rp2040.c
 * @author  Sebastian Lesse
 * @date    2022 / 09 / 24
 * @brief   Short description of this file
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

#include "unittest.h"

// --------------------------------------------------------------------------------

UT_ACTIVATE()

// --------------------------------------------------------------------------------

#include "initialization/initialization.h"
#include "cfg_driver_interface.h"
#include "driver/communication/usart/usart0_driver.h"
#include "driver/irq/irq_interface.h"

// --------------------------------------------------------------------------------

#define TEST_CASE_ID_INITIALIZATION                     0
#define TEST_CASE_ID_CONFIGURE_8N1_9600BD_AT_125_MHZ    1
#define TEST_CASE_ID_CONFIGURE_6O2_115200BD_AT_125_MHZ  2
#define TEST_CASE_ID_CONFIGURE_5E2_19200_BD_AT_100MHZ   3
#define TEST_CASE_ID_CONFIGURE_INVALID_CLK_PERI         4
#define TEST_CASE_ID_WRITE_10_BYTES_VALID_1             5
#define TEST_CASE_ID_IRQ_VALID_1                        6
#define TEST_CASE_ID_IRQ_RX_VALID_1                     7

// --------------------------------------------------------------------------------

#define UT_SIGNAL_TIMEOUT_MS    100

// --------------------------------------------------------------------------------

/**
 * @brief Structure to control the UART register of the RP2040.
 * Here all fields are rw becaus we need to manipulate the values for unit-testing.
 * @see RP2040 Datasheet (build-date: 2021-03-05 / build-version: 9bf4a25-clean)
 * 
 */
typedef struct {

    /**
     * @brief Data Register, UARTDR
     * See datasheet ch. 4.2.8 - UARTDR Register on page 447
     * for more details.
     */
    io_rw_32 data;

    /**
     * @brief Receive Status Register/Error Clear Register, UARTRSR/UARTECR
     * See datasheet ch. 4.2.8 - UARTDR Register on page 448
     * for more details.
     * 
     */
    io_rw_32 rsr;

    /**
     * @brief Respest the offset of the UARTFR-register.
     * Do not use this.
     */
    uint32_t _pad0[4];

    /**
     * @brief Flag Register, UARTFR
     * See datasheet ch. 4.2.8 - UARTDR Register on page 449
     * for more details.
     */
    io_rw_32 fr;

    /**
     * @brief Respest the offset of the UARTILPR Register
     * Do not use this.
     */
    uint32_t _pad1;

    /**
     * @brief IrDA Low-Power Counter Register, UARTILPR
     * See datasheet ch. 4.2.8 - UARTDR Register on page 450
     * for more details.
     */
    io_rw_32 ilpr;

    /**
     * @brief Integer Baud Rate Register, UARTIBRD
     * See datasheet ch. 4.2.8 - UARTDR Register on page 450
     * for more details.
     */
    io_rw_32 ibrd;

    /**
     * @brief Fractional Baud Rate Register, UARTFBRD
     * See datasheet ch. 4.2.8 - UARTDR Register on page 450
     * for more details.
     */
    io_rw_32 fbrd;

    /**
     * @brief Line Control Register, UARTLCR_H
     * See datasheet ch. 4.2.8 - UARTDR Register on page 451
     * for more details.
     */
    io_rw_32 lcr_h;

    /**
     * @brief Control Register, UARTCR
     * See datasheet ch. 4.2.8 - UARTDR Register on page 452
     * for more details.
     */
    io_rw_32 cr;

    /**
     * @brief Interrupt FIFO Level Select Register, UARTIFLS
     * See datasheet ch. 4.2.8 - UARTDR Register on page 453
     * for more details.
     */
    io_rw_32 ifls;

    /**
     * @brief Interrupt Mask Set/Clear Register, UARTIMSC
     * See datasheet ch. 4.2.8 - UARTDR Register on page 453
     * for more details.
     */
    io_rw_32 imsc;

    /**
     * @brief Raw Interrupt Status Register, UARTRIS
     * See datasheet ch. 4.2.8 - UARTDR Register on page 455
     * for more details.
     */
    io_rw_32 ris;

    /**
     * @brief Masked Interrupt Status Register, UARTMIS
     * See datasheet ch. 4.2.8 - UARTDR Register on page 456
     * for more details.
     */
    io_rw_32 mis;

    /**
     * @brief Interrupt Clear Register, UARTICR
     * See datasheet ch. 4.2.8 - UARTDR Register on page 457
     * for more details.
     */
    io_rw_32 icr;

    /**
     * @brief DMA Control Register, UARTDMACR
     * See datasheet ch. 4.2.8 - UARTDR Register on page 457
     * for more details.
     */
    io_rw_32 dmacr;

} RP2040_UART_REG;

RP2040_UART_REG ut_uart0_reg;
RP2040_UART_REG ut_uart1_reg;

// --------------------------------------------------------------------------------

/**
 * @brief The buffer where to store fifo bytes into.
 * 
 */
static u8 uart_tx_fifo_bufer[32];

/**
 * @brief The buffer where to read fifo bytes from.
 * 
 */
static u8 uart_rx_fifo_bufer[32];

// --------------------------------------------------------------------------------

static u8 counter_GET_UART0_REG = 0;
static u8 counter_GET_UART1_REG = 0;
static u8 counter_GET_CLK_FREQ  = 0;
static u8 counter_RESET_RP2040  = 0;
static u8 counter_SET_FIFO_DATA = 0;
static u8 counter_GET_FIFO_DATA = 0;

static void unittest_reset_counter(void) {

    counter_GET_UART0_REG = 0;
    counter_GET_UART1_REG = 0;
    counter_GET_CLK_FREQ  = 0;
    counter_RESET_RP2040  = 0;
    counter_SET_FIFO_DATA = 0;
    counter_GET_FIFO_DATA = 0;

    memset(&ut_uart0_reg, 0x00, sizeof(RP2040_UART_REG));
    memset(&ut_uart1_reg, 0x00, sizeof(RP2040_UART_REG));
    memset(uart_tx_fifo_bufer, 0x00, sizeof(uart_tx_fifo_bufer));
}

// --------------------------------------------------------------------------------

// stubs

void* ut_get_RP2040_UART0_REG_BASE_ADDRESS(void) {
    counter_GET_UART0_REG += 1;
    return (void*)&ut_uart0_reg;
}

extern void* ut_get_RP2040_UART1_REG_BASE_ADDRESS(void) {
    counter_GET_UART1_REG += 1;
    return (void*)&ut_uart1_reg;
}

/**
 * @brief is called everytime a byte is set into the fifo of the actual uart-instance
 * 
 */
void ut_set_fifo_data_callback(void) {

    if (UT_GET_TEST_CASE_ID() == TEST_CASE_ID_WRITE_10_BYTES_VALID_1) {

        if (counter_SET_FIFO_DATA < sizeof(uart_tx_fifo_bufer)) {
            uart_tx_fifo_bufer[counter_SET_FIFO_DATA] = ut_uart0_reg.data;
        }
        
        if (counter_SET_FIFO_DATA == 4) {
            ut_uart0_reg.fr |= 0x20;
        }
    }

    if (UT_GET_TEST_CASE_ID() == TEST_CASE_ID_IRQ_VALID_1) {

        if (counter_SET_FIFO_DATA < sizeof(uart_tx_fifo_bufer)) {
            uart_tx_fifo_bufer[counter_SET_FIFO_DATA] = ut_uart0_reg.data;
        }
    }

    counter_SET_FIFO_DATA += 1;
}

// --------------------------------------------------------------------------------

/**
 * @brief array holding the root of all irq-handlers
 */
static IRQ_INTERFACE_HANDLER* irq_handler_table[26];

/**
 * @see irg/irq_interface.h#irq_driver_init
 */
void irq_driver_init(void) {
    DEBUG_PASS("irq_driver_init()");
}

/**
 * @see irg/irq_interface.h#irq_enable_handler
 */
IRQ_INTERFACE_RET_VAL irq_enable_handler(u8 irq_num, u8 is_enabled, IRQ_INTERFACE_HANDLER* p_handler) {

}

// --------------------------------------------------------------------------------

/**
 * @see irg/irq_interface.h#irq_add_handler
 */
IRQ_INTERFACE_RET_VAL irq_add_handler(u8 irq_num, IRQ_INTERFACE_HANDLER* p_handler) {
    DEBUG_TRACE_byte(irq_num, "irq_add_handler() - IRQ-handler add - num:");
    irq_handler_table[irq_num] = p_handler;
    return IRQ_INTERFACE_OK;
}

/**
 * @see irg/irq_interface.h#irq_set_enabled
 */
IRQ_INTERFACE_RET_VAL irq_set_enabled(u8 irq_num, u8 is_enabled) {
    return IRQ_INTERFACE_OK;
}

void IRQ_20_Handler(void) {
    if (irq_handler_table[20] != 0) {
        irq_handler_table[20]->p_callback();
    }
}

// --------------------------------------------------------------------------------

void ut_get_fifo_data_callback(void) {

    if (UT_GET_TEST_CASE_ID() ==  TEST_CASE_ID_IRQ_RX_VALID_1) {
        if (counter_GET_FIFO_DATA < sizeof(uart_rx_fifo_bufer)) {
            ut_uart0_reg.data = uart_rx_fifo_bufer[counter_GET_FIFO_DATA];
        }
        
        if (counter_GET_FIFO_DATA == 4) {
            ut_uart0_reg.fr |= 0x10;
        }
    }

    counter_GET_FIFO_DATA += 1;
}

// --------------------------------------------------------------------------------

/**
 * @brief actual value of clock-frequency returned by 
 * clock_driver_peripheral_clk_frequency()
 * is set by the testcases.
 * 
 */
static u32 testcase_clock_freq = 0u;

u16 time_mgmnt_gettime_u16(void) {
    return 0;
}

u8 time_mgmnt_istimeup_raw_u16(u16 time_reference, u16 time_interval) {
    return 0;
}

u32 clock_driver_peripheral_clk_frequency(void) {
    counter_GET_CLK_FREQ += 1;
    return testcase_clock_freq;
}

void rp2040_reset_uart0(void) {
    counter_RESET_RP2040 += 1;

}

// --------------------------------------------------------------------------------

// Signals / Slots

/**
 * @brief IRQ Handler of the uart0 isntance
 * implemented by the uart-driver
 * 
 */
extern void IRQ_20_Handler(void);

// --------------------------------------------------------------------------------

/**
 * @brief Initialize all GPIOS
 * Check if the values taht have been written to the cpu-regsiters match the expected values.
 * 
 */
static void TEST_CASE_initialization(void) {

    UT_START_TEST_CASE("Initialization")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_INITIALIZATION);
        unittest_reset_counter();

        initialization();

        UT_CHECK_IS_EQUAL(counter_GET_UART0_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_UART1_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_CLK_FREQ, 0);
        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 1);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief configures uart0 with the following values:
 * - Baud-rate: 9600;
 * - Databits: 8
 * - Parity: none
 * - Stopbits: 1
 * 
 */
static void TEST_CASE_configure_CLK_PERI_NOT_VALID(void) {

    UT_START_TEST_CASE("Configure CLK-PERI NOT VALID")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_CONFIGURE_INVALID_CLK_PERI);
        unittest_reset_counter();

        TRX_DRIVER_CONFIGURATION trx_cfg = {
            .module = {
                .usart = {
                    .baudrate = BAUDRATE_9600,
                    .databits = DATABITS_8,
                    .parity   = PARITY_NONE,
                    .stopbits = STOPBITS_1
                }
            }
        };

        testcase_clock_freq = 0; // CLK-PERI invalid

        u32 uart0_lcr_h_match = (0 << 4)    // Fifo enabled
                              + (0 << 3)    // 1 stopbits
                              + (0 << 1)    // No Parity
                              + (0 << 5);   // 8 databits

        u32 uart0_cr_match    = (0 << 9)    // receive enabled
                              + (0 << 8)    // transmit enabled
                              + (0 << 0);   // uart enabled

        u32 uart0_imsc_match  = (0 << 4)    // RX IRQ Enabled
                              + (0 << 5);    // TX IRGQ enabled

        usart0_driver_configure(&trx_cfg);

        UT_CHECK_IS_EQUAL(ut_uart0_reg.lcr_h, uart0_lcr_h_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.ibrd, 0);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.fbrd, 0);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.cr, uart0_cr_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.imsc, uart0_imsc_match);

        UT_CHECK_IS_EQUAL(ut_uart1_reg.lcr_h, 0);
        UT_CHECK_IS_EQUAL(ut_uart1_reg.ibrd, 0);
        UT_CHECK_IS_EQUAL(ut_uart1_reg.fbrd, 0);

        UT_CHECK_IS_EQUAL(counter_GET_UART0_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_UART1_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_CLK_FREQ, 1);
        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 0);
        UT_CHECK_IS_EQUAL(counter_GET_FIFO_DATA, 0);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief configures uart0 with the following values:
 * - Baud-rate: 9600;
 * - Databits: 8
 * - Parity: none
 * - Stopbits: 1
 * 
 */
static void TEST_CASE_configure_8N1_9600_at_125MHz(void) {

    UT_START_TEST_CASE("Configure 8N1 9600 Bd at 125MHz")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_CONFIGURE_8N1_9600BD_AT_125_MHZ);
        unittest_reset_counter();

        TRX_DRIVER_CONFIGURATION trx_cfg = {
            .module = {
                .usart = {
                    .baudrate = BAUDRATE_9600,
                    .databits = DATABITS_8,
                    .parity   = PARITY_NONE,
                    .stopbits = STOPBITS_1
                }
            }
        };

        testcase_clock_freq = 125 * 1000000;

        u32 uart0_lcr_h_match = (1 << 4)    // Fifo enabled
                              + (0 << 3)    // 1 stopbits
                              + (0 << 1)    // No Parity
                              + (3 << 5);   // 8 databits

        u32 uart0_cr_match    = (1 << 9)    // receive enabled
                              + (1 << 8)    // transmit enabled
                              + (1 << 0);   // uart enabled

        u32 uart0_imsc_match  = (1 << 4)    // RX IRQ Enabled
                              + (1 << 5);    // TX IRQ enabled

        u32 uart0_ifls_match  = (0 << 0)    // Transmit FIFO becomes <= 1 / 8 full
                              + (0 << 3);   // Receive FIFO becomes >= 1 / 8 full 

        usart0_driver_configure(&trx_cfg);

        UT_CHECK_IS_EQUAL(ut_uart0_reg.lcr_h, uart0_lcr_h_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.ibrd,813);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.fbrd, 51);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.cr, uart0_cr_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.imsc, uart0_imsc_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.ifls, uart0_ifls_match);

        UT_CHECK_IS_EQUAL(ut_uart1_reg.lcr_h, 0);
        UT_CHECK_IS_EQUAL(ut_uart1_reg.ibrd, 0);
        UT_CHECK_IS_EQUAL(ut_uart1_reg.fbrd, 0);

        UT_CHECK_IS_EQUAL(counter_GET_UART0_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_UART1_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_CLK_FREQ, 1);
        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 0);
        UT_CHECK_IS_EQUAL(counter_GET_FIFO_DATA, 0);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief configures uart0 with the following values:
 * - Baud-rate: 19200;
 * - Databits: 6
 * - Parity: odd
 * - Stopbits: 2
 * 
 */
static void TEST_CASE_configure_6O2_115200_at_125MHz(void) {

    UT_START_TEST_CASE("Configure 6O2 115200 Bd at 125MHz")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_CONFIGURE_6O2_115200BD_AT_125_MHZ);
        unittest_reset_counter();

        TRX_DRIVER_CONFIGURATION trx_cfg = {
            .module = {
                .usart = {
                    .baudrate = BAUDRATE_115200,
                    .databits = DATABITS_6,
                    .parity   = PARITY_ODD,
                    .stopbits = STOPBITS_2
                }
            }
        };

        testcase_clock_freq = 125 * 1000000;

        u32 uart0_lcr_h_match = (1 << 4)    // Fifo enabled
                              + (1 << 3)    // 2 stopbits
                              + (1 << 1)    // ODD-Parity
                              + (1 << 5);   // 6 databits

        u32 uart0_cr_match    = (1 << 9)    // receive enabled
                              + (1 << 8)    // transmit enabled
                              + (1 << 0);   // uart enabled

        u32 uart0_imsc_match  = (1 << 4)    // RX IRQ Enabled
                              + (1 << 5);    // TX IRQ enabled

        u32 uart0_ifls_match  = (0 << 0)    // Transmit FIFO becomes <= 1 / 8 full
                              + (0 << 3);   // Receive FIFO becomes >= 1 / 8 full 

        usart0_driver_configure(&trx_cfg);

        UT_CHECK_IS_EQUAL(ut_uart0_reg.lcr_h, uart0_lcr_h_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.ibrd, 67);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.fbrd, 52);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.cr, uart0_cr_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.imsc, uart0_imsc_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.ifls, uart0_ifls_match);

        UT_CHECK_IS_EQUAL(ut_uart1_reg.lcr_h, 0);
        UT_CHECK_IS_EQUAL(ut_uart1_reg.ibrd, 0);
        UT_CHECK_IS_EQUAL(ut_uart1_reg.fbrd, 0);

        UT_CHECK_IS_EQUAL(counter_GET_UART0_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_UART1_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_CLK_FREQ, 1);
        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 0);
        UT_CHECK_IS_EQUAL(counter_GET_FIFO_DATA, 0);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief configures uart0 with the following values:
 * - Baud-rate: 19200;
 * - Databits: 5
 * - Parity: EVEN
 * - Stopbits: 2
 * 
 */
static void TEST_CASE_configure_5E2_19200_at_100MHz(void) {

    UT_START_TEST_CASE("Configure 5E2 19200 Bd at 100MHz")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_CONFIGURE_5E2_19200_BD_AT_100MHZ);
        unittest_reset_counter();

        TRX_DRIVER_CONFIGURATION trx_cfg = {
            .module = {
                .usart = {
                    .baudrate = BAUDRATE_19200,
                    .databits = DATABITS_5,
                    .parity   = PARITY_EVEN,
                    .stopbits = STOPBITS_2
                }
            }
        };

        testcase_clock_freq = 100 * 1000000;

        u32 uart0_lcr_h_match = (1 << 4)    // Fifo enabled
                              + (1 << 3)    // 2 stopbits
                              + (3 << 1)    // EVEN-Parity
                              + (0 << 5);   // 5 databits

        u32 uart0_cr_match    = (1 << 9)    // receive enabled
                              + (1 << 8)    // transmit enabled
                              + (1 << 0);   // uart enabled

        u32 uart0_imsc_match  = (1 << 4)    // RX IRQ Enabled
                              + (1 << 5);    // TX IRQ enabled

        usart0_driver_configure(&trx_cfg);

        UT_CHECK_IS_EQUAL(ut_uart0_reg.lcr_h, uart0_lcr_h_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.ibrd, 325);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.fbrd, 33);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.cr, uart0_cr_match);
        UT_CHECK_IS_EQUAL(ut_uart0_reg.imsc, uart0_imsc_match);

        UT_CHECK_IS_EQUAL(ut_uart1_reg.lcr_h, 0);
        UT_CHECK_IS_EQUAL(ut_uart1_reg.ibrd, 0);
        UT_CHECK_IS_EQUAL(ut_uart1_reg.fbrd, 0);

        UT_CHECK_IS_EQUAL(counter_GET_UART0_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_UART1_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_CLK_FREQ, 1);
        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 0);
        UT_CHECK_IS_EQUAL(counter_GET_FIFO_DATA, 0);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief configures uart0 with the following values:
 * - Tries to write 10 bytes into the uart-driver
 * - 5 of them will be stored into the uart-fifo
 * - the other five will bes tored into the message-buffer
 * 
 */
static void TEST_CASE_write_10_bytes(void) {

    UT_START_TEST_CASE("Write 10 bytes - 5 into FIFO")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_WRITE_10_BYTES_VALID_1);
        unittest_reset_counter();

        u8 buffer[] = { 0xAF, 0xF1, 0xAF, 0xF2, 0xAF, 0xF3, 0xAF, 0xF4, 0xAF, 0xF5};
        u16 num_bytes_written = usart0_driver_set_N_bytes(sizeof(buffer), buffer);

        UT_CHECK_IS_EQUAL(num_bytes_written, 10);
        UT_CHECK_IS_EQUAL(counter_SET_FIFO_DATA, 5);
        UT_CHECK_IS_EQUAL(counter_GET_FIFO_DATA, 0);

        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[0], buffer[0]);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[1], buffer[1]);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[2], buffer[2]);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[3], buffer[3]);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[4], buffer[4]);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[5], 0x00);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[6], 0x00);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[7], 0x00);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[8], 0x00);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[9], 0x00);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Calls the IRQ handler of the uart0 instance
 * The remaining 5 bytes from the testcase above are written into the uart-fifo
 * 
 */
static void TEST_CASE_irq_1(void) {

    UT_START_TEST_CASE("IRQ 1")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_IRQ_VALID_1);
        unittest_reset_counter();

        ut_uart0_reg.mis = 0x20;
        IRQ_20_Handler();

        /**
         * @brief these are the remaining bytes of 
         * TEST_CASE_write_10_bytes()
         * 
         */
        u8 buffer[] = { 0xF3, 0xAF, 0xF4, 0xAF, 0xF5};

        UT_CHECK_IS_EQUAL(counter_SET_FIFO_DATA, 5);
        UT_CHECK_IS_EQUAL(counter_GET_FIFO_DATA, 0);

        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[0], buffer[0]);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[1], buffer[1]);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[2], buffer[2]);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[3], buffer[3]);
        UT_CHECK_IS_EQUAL(uart_tx_fifo_bufer[4], buffer[4]);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Calls the IRQ handler of the uart0 instance
 * The remaining 5 bytes from the testcase above are written into the uart-fifo
 * 
 */
static void TEST_CASE_rx_irq_1(void) {

    UT_START_TEST_CASE("RX - IRQ 1")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_IRQ_RX_VALID_1);
        unittest_reset_counter();

        // RX IRQ
        ut_uart0_reg.mis = 0x10;

        uart_rx_fifo_bufer[0] = 0xF3;
        uart_rx_fifo_bufer[1] = 0xAF;
        uart_rx_fifo_bufer[2] = 0xF4;
        uart_rx_fifo_bufer[3] = 0xAF;
        uart_rx_fifo_bufer[4] = 0xF5;

        u8 buffer[32];
        memset(buffer, 0x00, sizeof(buffer));

        // set flag that bytes have been received
        ut_uart0_reg.fr = 0x10;
        IRQ_20_Handler();

        UT_CHECK_IS_EQUAL(counter_SET_FIFO_DATA, 0);
        UT_CHECK_IS_EQUAL(counter_GET_FIFO_DATA, 5);
        UT_CHECK_IS_EQUAL(usart0_driver_bytes_available(), 5);

        u16 bytes_read = usart0_driver_get_N_bytes(sizeof(buffer), buffer);
        UT_CHECK_IS_EQUAL(bytes_read, 5);

        UT_CHECK_IS_EQUAL(buffer[0], uart_rx_fifo_bufer[0]);
        UT_CHECK_IS_EQUAL(buffer[1], uart_rx_fifo_bufer[1]);
        UT_CHECK_IS_EQUAL(buffer[2], uart_rx_fifo_bufer[2]);
        UT_CHECK_IS_EQUAL(buffer[3], uart_rx_fifo_bufer[3]);
        UT_CHECK_IS_EQUAL(buffer[4], uart_rx_fifo_bufer[4]);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

int main(void) {

    for (u8 i = 0; i < 26; ++i) {
        irq_handler_table[i] = 0;
    }

    UT_START_TESTBENCH("Welcome the the UNITTEST for RP2040 UART-driver 1.0")
    {
        TEST_CASE_initialization();
        TEST_CASE_configure_CLK_PERI_NOT_VALID();
        TEST_CASE_configure_8N1_9600_at_125MHz();
        TEST_CASE_configure_6O2_115200_at_125MHz();
        TEST_CASE_configure_5E2_19200_at_100MHz();
        TEST_CASE_write_10_bytes();
        TEST_CASE_irq_1();
        // TEST_CASE_rx_irq_1();
    }
    UT_END_TESTBENCH()

    return UT_TEST_RESULT();
}

// --------------------------------------------------------------------------------
