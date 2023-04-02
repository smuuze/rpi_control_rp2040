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
 * @file    unittest_spi_driver_rp2040.c
 * @author  Sebastian Lesse
 * @date    2023 / 03 / 18
 * @brief   Short description of this file
 * 
 */

#define TRACER_ON

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
#include "driver/communication/spi/spi0_driver.h"
#include "driver/irq/irq_interface.h"

// --------------------------------------------------------------------------------

#define TEST_CASE_ID_INITIALIZATION                     0
#define TEST_CASE_ID_CONFIGURATION_01                   1
#define TEST_CASE_ID_ADD_BYTES_TO_FIFO_01               2
#define TEST_CASE_ID_ADD_BYTES_TO_FIFO_02               3
#define TEST_CASE_ID_TX_BYTES_FROM_FIFO                 4

// --------------------------------------------------------------------------------

/**
 * @brief 
 */
typedef struct {

    /**
     * @brief Control register 0,
     * 0x0000ff00 [15:8] : SCR (0): Serial clock rate
     * 0x00000080 [7]   : SPH (0): SSPCLKOUT phase, applicable to Motorola SPI frame format only
     * 0x00000040 [6]   : SPO (0): SSPCLKOUT polarity, applicable to Motorola SPI frame format only
     * 0x00000030 [5:4] : FRF (0): Frame format: 00 Motorola SPI frame format
     * 0x0000000f [3:0] : DSS (0): Data Size Select: 0000 Reserved, undefined operation
     */
    io_rw_32 SSPCR0;

    /**
     * @brief Control register 1
     * 0x00000008 [3] : SOD (0): Slave-mode output disable
     * 0x00000004 [2] : MS (0): Master or slave mode select
     * 0x00000002 [1] : SSE (0): Synchronous serial port enable: 0 SSP operation disabled
     * 0x00000001 [0] : LBM (0): Loop back mode: 0 Normal serial port operation enabled
     */
    io_rw_32 SSPCR1;

    /**
     * @brief Data register
     * 0x0000ffff [15:0] : DATA (0): Transmit/Receive FIFO: Read Receive FIFO
     */
    io_rw_32 SSPDR;

    /**
     * @brief Status register
     * 0x00000010 [4] : BSY (0): PrimeCell SSP busy flag, RO: 0 SSP is idle
     * 0x00000008 [3] : RFF (0): Receive FIFO full, RO: 0 Receive FIFO is not full
     * 0x00000004 [2] : RNE (0): Receive FIFO not empty, RO: 0 Receive FIFO is empty
     * 0x00000002 [1] : TNF (1): Transmit FIFO not full, RO: 0 Transmit FIFO is full
     * 0x00000001 [0] : TFE (1): Transmit FIFO empty, RO: 0 Transmit FIFO is not empty
     */
    io_ro_32 SSPSR;

    /**
     * @brief Clock prescale register
     * 0x000000ff [7:0] : CPSDVSR (0): Clock prescale divisor
     */
    io_rw_32 SSPCPSR;

    /**
     * @brief Interrupt mask set or clear register
     * 0x00000008 [3] : TXIM (0): Transmit FIFO interrupt mask: 0 Transmit FIFO half empty or less condition interrupt is masked
     * 0x00000004 [2] : RXIM (0): Receive FIFO interrupt mask: 0 Receive FIFO half full or less condition interrupt is masked
     * 0x00000002 [1] : RTIM (0): Receive timeout interrupt mask: 0 Receive FIFO not empty and no read prior to timeout...
     * 0x00000001 [0] : RORIM (0): Receive overrun interrupt mask: 0 Receive FIFO written to while full condition...
     */
    io_rw_32 SSPIMSC;

    /**
     * @brief Raw interrupt status register
     * 0x00000008 [3] : TXRIS (1): Gives the raw interrupt state, prior to masking, of the SSPTXINTR interrupt
     * 0x00000004 [2] : RXRIS (0): Gives the raw interrupt state, prior to masking, of the SSPRXINTR interrupt
     * 0x00000002 [1] : RTRIS (0): Gives the raw interrupt state, prior to masking, of the SSPRTINTR interrupt
     * 0x00000001 [0] : RORRIS (0): Gives the raw interrupt state, prior to masking, of the SSPRORINTR interrupt
     */
    io_ro_32 SSPRIS;

    /**
     * @brief Masked interrupt status register
     * 0x00000008 [3] : TXMIS (0): Gives the transmit FIFO masked interrupt state, after masking, of the SSPTXINTR interrupt
     * 0x00000004 [2] : RXMIS (0): Gives the receive FIFO masked interrupt state, after masking, of the SSPRXINTR interrupt
     * 0x00000002 [1] : RTMIS (0): Gives the receive timeout masked interrupt state, after masking, of the SSPRTINTR interrupt
     * 0x00000001 [0] : RORMIS (0): Gives the receive over run masked interrupt status, after masking, of the...
     */
    io_ro_32 SSPMIS;

    /**
     * @brief Interrupt clear register, SSPICR on page 3-11
     * 0x00000002 [1] : RTIC (0): Clears the SSPRTINTR interrupt
     * 0x00000001 [0] : RORIC (0): Clears the SSPRORINTR interrupt
     */
    io_rw_32 SSPICR;

    /**
     * @brief DMA control register, SSPDMACR on page 3-12
     * 0x00000002 [1] : TXDMAE (0): Transmit DMA Enable
     * 0x00000001 [0] : RXDMAE (0): Receive DMA Enable
     */
    io_rw_32 SSPDMACR;

} RP2040_SPI_REG;

// --------------------------------------------------------------------------------

RP2040_SPI_REG ut_spi0_reg;
RP2040_SPI_REG ut_spi1_reg;

// --------------------------------------------------------------------------------

/**
 * @brief The buffer where to store fifo bytes into.
 */
static u8 spi_tx_fifo_bufer[32];

// --------------------------------------------------------------------------------

static u8 counter_RESET_RP2040 = 0;
static u8 counter_GET_SPI0_REG = 0;
static u8 counter_GET_SPI1_REG = 0;

static u8 counter_ENABLE_IRQ        = 0;
static u8 counter_ENABLE_IRQ_SPI0   = 0;
static u8 counter_ENABLE_IRQ_SPI1   = 0;

static u8 counter_ADD_IRQ_HANDLER           = 0;
static u8 counter_ADD_IRQ_HANDLER_SPI0      = 0;
static u8 counter_ADD_IRQ_HANDLER_SPI1      = 0;

static u8 counter_ENABLE_IRQ_HANDLER        = 0;
static u8 counter_ENABLE_IRQ_HANDLER_SPI0   = 0;
static u8 counter_ENABLE_IRQ_HANDLER_SPI1   = 0;

static u8 counter_SET_FIFO_DATA = 0;

static void unittest_reset_counter(void) {

    DEBUG_PASS("UT - unittest_reset_counter()");

    counter_RESET_RP2040 = 0;
    counter_GET_SPI0_REG = 0;
    counter_GET_SPI1_REG = 0;

    counter_ENABLE_IRQ      = 0;
    counter_ENABLE_IRQ_SPI0 = 0;
    counter_ENABLE_IRQ_SPI1 = 0;

    counter_ENABLE_IRQ_HANDLER      = 0;
    counter_ENABLE_IRQ_HANDLER_SPI0 = 0;
    counter_ENABLE_IRQ_HANDLER_SPI1 = 0;

    counter_ADD_IRQ_HANDLER         = 0;
    counter_ADD_IRQ_HANDLER_SPI0    = 0;
    counter_ADD_IRQ_HANDLER_SPI1    = 0;

    counter_SET_FIFO_DATA = 0;

    memset(&ut_spi0_reg, 0, sizeof(ut_spi0_reg));
    memset(&ut_spi1_reg, 0, sizeof(ut_spi1_reg));

    memset(spi_tx_fifo_bufer, 0x00, sizeof(spi_tx_fifo_bufer));
}

// --------------------------------------------------------------------------------

void* ut_get_RP2040_SPI0_REG_BASE_ADDRESS(void) {
    counter_GET_SPI0_REG += 1;
    return (void*)&ut_spi0_reg;
}

void* ut_get_RP2040_SPI1_REG_BASE_ADDRESS(void) {
    counter_GET_SPI1_REG += 1;
    return (void*)&ut_spi1_reg;
}

// --------------------------------------------------------------------------------

/**
 * @brief Is called via the macro UNITTEST_SET_FIFO_DATA_CALLBACK from the spi-driver
 * @see config.h#UNITTEST_SET_FIFO_DATA_CALLBACK
 */
void ut_set_fifo_data_callback(void) {

    DEBUG_PASS("ut_set_fifo_data_callback() - TEST_CASE_ID_ADD_BYTES_TO_FIFO_01");

    if (UT_GET_TEST_CASE_ID() == TEST_CASE_ID_ADD_BYTES_TO_FIFO_01) {

        if (counter_SET_FIFO_DATA < sizeof(spi_tx_fifo_bufer)) {
            spi_tx_fifo_bufer[counter_SET_FIFO_DATA] = ut_spi0_reg.SSPDR;
            ut_spi0_reg.SSPDR = 0;
        }
        
        if (counter_SET_FIFO_DATA >= sizeof(spi_tx_fifo_bufer)) {
            ut_spi0_reg.SSPCPSR = 0; // transmit-fifo is full
        }

    } else if (UT_GET_TEST_CASE_ID() == TEST_CASE_ID_ADD_BYTES_TO_FIFO_02) {

        if (counter_SET_FIFO_DATA < sizeof(spi_tx_fifo_bufer)) {
            spi_tx_fifo_bufer[counter_SET_FIFO_DATA] = ut_spi0_reg.SSPDR;
            ut_spi0_reg.SSPDR = 0;
        }
        
        if (counter_SET_FIFO_DATA >= 4) {
            ut_spi0_reg.SSPCPSR = 0; // transmit-fifo is full
        }

    } else if (UT_GET_TEST_CASE_ID() == TEST_CASE_ID_TX_BYTES_FROM_FIFO) {

        if (counter_SET_FIFO_DATA < sizeof(spi_tx_fifo_bufer)) {
            spi_tx_fifo_bufer[counter_SET_FIFO_DATA] = ut_spi0_reg.SSPDR;
            ut_spi0_reg.SSPDR = 0;
        }
        
        if (counter_SET_FIFO_DATA > 4) {
            ut_spi0_reg.SSPCPSR = 0; // transmit-fifo is full
        }
    }
    
    counter_SET_FIFO_DATA += 1;
}

// --------------------------------------------------------------------------------

/**
 * @brief actual value of clock-frequency returned by 
 * clock_driver_peripheral_clk_frequency()
 * is set by the testcases.
 * 
 */
// static u32 testcase_clock_freq = 0u;

u16 time_mgmnt_gettime_u16(void) {
    return 0;
}

u8 time_mgmnt_istimeup_raw_u16(u16 time_reference, u16 time_interval) {
    return 0;
}

// u32 clock_driver_peripheral_clk_frequency(void) {
//     counter_GET_CLK_FREQ += 1;
//     return testcase_clock_freq;
// }

// --------------------------------------------------------------------------------

void rp2040_reset_spi0(void) {
    DEBUG_PASS("UT - rp2040_reset_spi0()");
    counter_RESET_RP2040 += 1;
}

void rp2040_reset_spi1(void) {
    DEBUG_PASS("UT - rp2040_reset_spi1()");
    counter_RESET_RP2040 += 1;
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
    DEBUG_PASS("UT - irq_driver_init()");
}

/**
 * @see irg/irq_interface.h#irq_enable_handler
 */
IRQ_INTERFACE_RET_VAL irq_enable_handler(u8 irq_num, u8 is_enabled, IRQ_INTERFACE_HANDLER* p_handler) {

    DEBUG_TRACE_byte(irq_num, "UT - irq_enable_handler() - IRQ-NUM:");
    counter_ENABLE_IRQ_HANDLER += 1;

    if (irq_num == IRQ_NUM_SPI0) {
        if (p_handler != NULL && is_enabled != 0) {
            counter_ENABLE_IRQ_HANDLER_SPI0 += 1;
            DEBUG_PASS("UT - irq_enable_handler() - SPI0 - SUCCESSFUL");
        } else {
            DEBUG_PASS("UT - irq_enable_handler() - SPI0 - NULLPOINTER");
        }
    }

    return irq_set_enabled(irq_num, is_enabled);
}

/**
 * @see irg/irq_interface.h#irq_add_handler
 */
IRQ_INTERFACE_RET_VAL irq_add_handler(u8 irq_num, IRQ_INTERFACE_HANDLER* p_handler) {
    DEBUG_TRACE_byte(irq_num, "UT - irq_add_handler() - IRQ-NUM:");
    counter_ADD_IRQ_HANDLER += 1;

    if (irq_num == IRQ_NUM_SPI0) {
        if (p_handler != NULL) {
            counter_ADD_IRQ_HANDLER_SPI0 += 1;
            DEBUG_PASS("UT - irq_add_handler() - SPI0 - SUCCESSFUL");
        } else {
            DEBUG_PASS("UT - irq_add_handler() - SPI0 - NULLPOINTER");
        }
    }

    irq_handler_table[irq_num] = p_handler;
    return IRQ_INTERFACE_OK;
}

/**
 * @see irg/irq_interface.h#irq_set_enabled
 */
IRQ_INTERFACE_RET_VAL irq_set_enabled(u8 irq_num, u8 is_enabled) {
    DEBUG_TRACE_byte(irq_num, "UT - irq_set_enabled() - IRQ-NUM:");

    counter_ENABLE_IRQ += 1;

    if (irq_num == IRQ_NUM_SPI0 && is_enabled != 0) {
        counter_ENABLE_IRQ_SPI0 += 1;
        DEBUG_PASS("UT - irq_enable_handler() - SPI0");
    }

    if (UT_GET_TEST_CASE_ID() == TEST_CASE_ID_INITIALIZATION) {
        return IRQ_INTERFACE_OK;
    }

    return IRQ_INTERFACE_INVALID;
}

// --------------------------------------------------------------------------------

void IRQ_18_Handler(void) {
    DEBUG_PASS("IRQ_18_Handler()");
    if (irq_handler_table[IRQ_NUM_SPI0] != 0) {
        irq_handler_table[IRQ_NUM_SPI0]->p_callback();
    }
}

// --------------------------------------------------------------------------------

/**
 * @brief Expectations:
 * The SPI-peripheral is reseted
 */
static void TEST_CASE_initialization(void) {

    UT_START_TEST_CASE("Initialization")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_INITIALIZATION);
        unittest_reset_counter();

        initialization();

        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 1);

        UT_CHECK_IS_EQUAL(counter_GET_SPI0_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_SPI1_REG, 0);
        
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCPSR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR0, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR1, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPSR, 0);
        
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCPSR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR0, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR1, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPSR, 0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ,      0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI0, 0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI1, 0);

        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER,         1);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI0,    1);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI1,    0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER,      0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI0, 0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI1, 0);

        UT_CHECK_IS_EQUAL(counter_SET_FIFO_DATA, 0);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Expectations:
 * The SPI-peripheral is reseted
 */
static void TEST_CASE_configuration_01(void) {

    UT_START_TEST_CASE("Configuration 01")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_CONFIGURATION_01);
        unittest_reset_counter();

        TRX_DRIVER_CONFIGURATION configuration_01 = {
            .module = {
                .spi = {
                    .data_order = 0,
                    .mode = DRIVER_SPI_MODE_0,
                    .clk_divider = DRIVER_SPI_CLK_DEVIDER_4,
                    .is_master = COM_DRIVER_IS_MASTER
                }
            }
        };

        spi0_driver_configure(&configuration_01);
        
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCPSR, 4); // clock-prescaler = 4
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR0, 7); // data-size 8 bit
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR1, 0); // is not enabled, is master

        configuration_01.module.spi.data_order = 0;
        configuration_01.module.spi.mode = DRIVER_SPI_MODE_1;
        configuration_01.module.spi.clk_divider = DRIVER_SPI_CLK_DEVIDER_8;
        configuration_01.module.spi.is_master = COM_DRIVER_IS_MASTER;

        spi0_driver_configure(&configuration_01);
        
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCPSR, 8); // clock-prescaler = 8
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR0, 135); // data-size 8 bit | SPH
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR1, 0); // is not enabled, is master

        configuration_01.module.spi.data_order = 0;
        configuration_01.module.spi.mode = DRIVER_SPI_MODE_2;
        configuration_01.module.spi.clk_divider = DRIVER_SPI_CLK_DEVIDER_16;
        configuration_01.module.spi.is_master = COM_DRIVER_IS_MASTER;

        spi0_driver_configure(&configuration_01);
        
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCPSR, 16); // clock-prescaler = 8
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR0, 71); // data-size 8 bit | SPO
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR1, 0); // is not enabled, is master

        configuration_01.module.spi.data_order = 0;
        configuration_01.module.spi.mode = DRIVER_SPI_MODE_3;
        configuration_01.module.spi.clk_divider = DRIVER_SPI_CLK_DEVIDER_32;
        configuration_01.module.spi.is_master = COM_DRIVER_IS_MASTER;

        spi0_driver_configure(&configuration_01);
        
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCPSR, 32); // clock-prescaler = 8
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR0, 199); // data-size 8 bit | SPH | SPO
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR1, 0); // is not enabled, is master

        configuration_01.module.spi.data_order = 0;
        configuration_01.module.spi.mode = DRIVER_SPI_MODE_3;
        configuration_01.module.spi.clk_divider = DRIVER_SPI_CLK_DEVIDER_64;
        configuration_01.module.spi.is_master = COM_DRIVER_IS_SLAVE;

        spi0_driver_configure(&configuration_01);
        
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCPSR, 64); // clock-prescaler = 8
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR0, 199); // data-size 8 bit | SPH | SPO
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR1, 4); // is slave but not enabled

        // ---
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPSR, 0);

        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 0);

        UT_CHECK_IS_EQUAL(counter_GET_SPI0_REG, 5); // changed configuration for 5 times
        UT_CHECK_IS_EQUAL(counter_GET_SPI1_REG, 0);
        
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCPSR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR0, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR1, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPSR, 0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ,      5);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI0, 5);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI1, 0);

        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER,         0);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI0,    0);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI1,    0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER,      5);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI0, 5);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI1, 0);
        
        UT_CHECK_IS_EQUAL(counter_SET_FIFO_DATA, 0);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Expectations:
 * Write 4 bytes into the HW-fifo of spi0
 */
static void TEST_CASE_add_bytes_to_fifo_01(void) {

    UT_START_TEST_CASE("Add Bytes to Fifo 01")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_ADD_BYTES_TO_FIFO_01);
        unittest_reset_counter();

        u8 buffer[4] = {
            0x11, 0x22,0x33,0x44
        };

         // transmit fifo is not full
        ut_spi0_reg.SSPCPSR |= 0x00000002;

        spi0_driver_start_tx();

        u16 num_bytes_written = spi0_driver_set_N_bytes(
            sizeof(buffer),
            buffer
        );

        UT_CHECK_IS_EQUAL(num_bytes_written, 4);

        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 0);

        UT_CHECK_IS_EQUAL(counter_GET_SPI0_REG, 2); // start-tx and set-N-bytes
        UT_CHECK_IS_EQUAL(counter_GET_SPI1_REG, 0);
        
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCPSR, 0x00000002); // fifo is not full
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR0, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR1, 2); // start-tx
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPSR, 0);
        
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCPSR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR0, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR1, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPSR, 0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ,      0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI0, 0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI1, 0);

        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER,         0);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI0,    0);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI1,    0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER,      0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI0, 0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI1, 0);

        UT_CHECK_IS_EQUAL(counter_SET_FIFO_DATA, 4);

        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[0], 0x11);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[1], 0x22);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[2], 0x33);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[3], 0x44);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Expectations:
 * Writes 5 bytes into the HW-fifo of spi0
 * Then writes 5 bytes into the sw-buffer of spi0
 * In summary there are 10 bytes written
 */
static void TEST_CASE_add_bytes_to_fifo_02(void) {

    UT_START_TEST_CASE("Add Bytes to Fifo 02")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_ADD_BYTES_TO_FIFO_02);
        unittest_reset_counter();

        u8 buffer[] = {
            0x01, 0x02, 0x03, 0x04, 0x05,
            0x06, 0x07, 0x08, 0x09, 0x0A
        };

         // transmit fifo is not full
        ut_spi0_reg.SSPCPSR |= 0x00000002;

        spi0_driver_start_tx();
        u16 num_bytes_written = spi0_driver_set_N_bytes(
            sizeof(buffer),
            buffer
        );

        UT_CHECK_IS_EQUAL(num_bytes_written, 10);

        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 0);

        UT_CHECK_IS_EQUAL(counter_GET_SPI0_REG, 2); // start-tx / set-N-bytes
        UT_CHECK_IS_EQUAL(counter_GET_SPI1_REG, 0);
        
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCPSR, 0); // fifo is not full
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR0, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR1, 2); // enabled because of start-tx
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPSR, 0);
        
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCPSR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR0, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR1, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPSR, 0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ,      0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI0, 0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI1, 0);

        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER,         0);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI0,    0);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI1,    0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER,      0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI0, 0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI1, 0);

        UT_CHECK_IS_EQUAL(counter_SET_FIFO_DATA, 5);

        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[0], 0x01);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[1], 0x02);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[2], 0x03);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[3], 0x04);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[4], 0x05);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Expectations:
 * Transmits the 5 bytes of the sw-buffer that have been added
 * in testcase TEST_CASE_add_bytes_to_fifo_02()
 */
static void TEST_CASE_tx_bytes_from_fifo(void) {

    UT_START_TEST_CASE("TX bytes from fifo")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_TX_BYTES_FROM_FIFO);
        unittest_reset_counter();

        // we need to "enable" the spi
        ut_spi0_reg.SSPCR1 |= 0x00000002;

         // transmit fifo is not full
        ut_spi0_reg.SSPCPSR |= 0x00000002;

        IRQ_18_Handler();

        UT_CHECK_IS_EQUAL(counter_RESET_RP2040, 0);

        UT_CHECK_IS_EQUAL(counter_GET_SPI0_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_SPI1_REG, 0);
        
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCPSR, 2); // fifo is not full
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR0, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPCR1, 2); // spi is enabled
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi0_reg.SSPSR, 0);
        
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCPSR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR0, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPCR1, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDMACR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPDR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPICR, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPIMSC, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPMIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPRIS, 0);
        UT_CHECK_IS_EQUAL(ut_spi1_reg.SSPSR, 0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ,      0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI0, 0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_SPI1, 0);

        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER,         0);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI0,    0);
        UT_CHECK_IS_EQUAL(counter_ADD_IRQ_HANDLER_SPI1,    0);

        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER,      0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI0, 0);
        UT_CHECK_IS_EQUAL(counter_ENABLE_IRQ_HANDLER_SPI1, 0);

        UT_CHECK_IS_EQUAL(counter_SET_FIFO_DATA, 5); // 5 bytes written

        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[0], 0x06);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[1], 0x07);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[2], 0x08);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[3], 0x09);
        UT_CHECK_IS_EQUAL(spi_tx_fifo_bufer[4], 0x0A);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

int main(void) {

    for (u8 i = 0; i < 26; ++i) {
        irq_handler_table[i] = 0;
    }

    memset(&ut_spi0_reg, 0, sizeof(RP2040_SPI_REG));

    UT_START_TESTBENCH("Welcome the the UNITTEST for RP2040 SPI-driver 1.0")
    {
        TEST_CASE_initialization();
        TEST_CASE_configuration_01();
        TEST_CASE_add_bytes_to_fifo_01();
        TEST_CASE_add_bytes_to_fifo_02();
        TEST_CASE_tx_bytes_from_fifo();
    }
    UT_END_TESTBENCH()

    return UT_TEST_RESULT();
}

// --------------------------------------------------------------------------------
