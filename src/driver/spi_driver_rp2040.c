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
 * @file    spi_driver_rp2040.c
 * @author  Sebastian Lesse
 * @date    2023 / 03 / 07
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

#include "rp2040_reset.h"

// --------------------------------------------------------------------------------

#include "driver/irq/irq_interface.h"
#include "driver/cfg_driver_interface.h"
#include "common/local_msg_buffer.h"
#include "driver/communication/spi/spi0_driver.h"
#include "local_module_status.h"
#include "time_management/time_management.h"

// --------------------------------------------------------------------------------

// Sets the Clock-phase bit of SSPCR0
#define SPI_DRIVER_SSPCR0_SPH_EN                    __UNSIGNED(1 << 7)

// Sets the clock-polarity bit of SSPCR0
#define SPI_DRIVER_SSPCR0_SPO_EN                    __UNSIGNED(1 << 6)

// Available data-sizes useable for SSPCR0
#define SPI_DRIVER_SSPCR0_DATA_SIZE_4_BIT           __UNSIGNED(3)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_5_BIT           __UNSIGNED(4)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_6_BIT           __UNSIGNED(5)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_7_BIT           __UNSIGNED(6)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_8_BIT           __UNSIGNED(7)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_9_BIT           __UNSIGNED(8)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_10_BIT          __UNSIGNED(9)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_11_BIT          __UNSIGNED(10)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_12_BIT          __UNSIGNED(11)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_13_BIT          __UNSIGNED(12)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_14_BIT          __UNSIGNED(13)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_15_BIT          __UNSIGNED(14)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_16_BIT          __UNSIGNED(15)

// MASTER / SLAVE mode
#define SPI_DRIVER_SSPCR1_MASTER                    __UNSIGNED(0x00000000)
#define SPI_DRIVER_SSPCR1_SLAVE                     __UNSIGNED(0x00000004)

// Enable the SSP peripheral
#define SPI_DRIVER_SSPCR1_ENABLE                    __UNSIGNED(0x00000002)

// If this bit is set, the transmit-fifo is empty
#define SPI_DRIVER_SSPSR_TFE                        __UNSIGNED(0x00000001)

// If this bit is set, the transmit-fifo is not full
#define SPI_DRIVER_SSPSR_TNF                        __UNSIGNED(0x00000002)

// If this bit is set, the receive-fifo is not empty
#define SPI_DRIVER_SSPSR_RNE                        __UNSIGNED(0x00000004)

// If this bit is set, the receive-fifo is full
#define SPI_DRIVER_SSPSR_RFF                        __UNSIGNED(0x00000008)

// If this bit is set, the spi is currently transmitting data
#define SPI_DRIVER_SSPSR_BSY                        __UNSIGNED(0x00000010)

// Available clock prescaler values
#define SPI_DRIVER_SSPCPSR_4                        4
#define SPI_DRIVER_SSPCPSR_8                        8
#define SPI_DRIVER_SSPCPSR_16                       16
#define SPI_DRIVER_SSPCPSR_32                       32
#define SPI_DRIVER_SSPCPSR_64                       64
#define SPI_DRIVER_SSPCPSR_128                      128

#define SPI_DRIVER_SSPIMSC_TXIM                     __UNSIGNED(0x00000008)

#define SPI_DRIVER_SSPICR_RORIC                     __UNSIGNED(0x00000001)

#define SPI_DRIVER_SSPMIS_TXMIS                     __UNSIGNED(0x00000008)
#define SPI_DRIVER_SSPMIS_RXMIS                     __UNSIGNED(0x00000004)
#define SPI_DRIVER_SSPMIS_RTMIS                     __UNSIGNED(0x00000002)
#define SPI_DRIVER_SSPMIS_RORMIS                    __UNSIGNED(0x00000001)

/**
 * @brief Value to clear all IRQ of a spi
 */
#define SPI_DRIVER_SSPICR_CLEAR_ALL                 __UNSIGNED(0x00000003)

#define SPI_DRIVER_ENABLE                           1
#define SPI_DRIVER_DISABLE                          0

#define SPI_DRIVER_PADDING_BYTE                     __UNSIGNED(0x00)

// --------------------------------------------------------------------------------

#ifndef SPI0_DRIVER_MAX_NUM_BYTES_TRANSMIT_BUFFER
#define SPI0_DRIVER_MAX_NUM_BYTES_TRANSMIT_BUFFER   128
#endif

#ifndef SPI0_DRIVER_MAX_NUM_BYTES_RECEIVE_BUFFER
#define SPI0_DRIVER_MAX_NUM_BYTES_RECEIVE_BUFFER    128
#endif

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

/**
 * @brief transmit and receive buffer of the spi0 instance of the rp2040
 * Both buffers must only b used within the SPI0 instance.
 */

BUILD_LOCAL_MSG_BUFFER(
    static inline,
    SPI0_TX_BUFFER,
    SPI0_DRIVER_MAX_NUM_BYTES_TRANSMIT_BUFFER
)
BUILD_LOCAL_MSG_BUFFER_CLASS(SPI0_TX_BUFFER)

BUILD_LOCAL_MSG_BUFFER(
    static inline,
    SPI0_RX_BUFFER,
    SPI0_DRIVER_MAX_NUM_BYTES_RECEIVE_BUFFER
)
BUILD_LOCAL_MSG_BUFFER_CLASS(SPI0_RX_BUFFER)

// --------------------------------------------------------------------------------

#define SPI0_STATUS_RX_ACTIVE       0
#define SPI0_STATUS_TX_ACTIVE       1
#define SPI0_STATUS_INITIALIZED     2

BUILD_MODULE_STATUS_FAST(SPI0_STATUS, 4)

// --------------------------------------------------------------------------------

/**
 * @brief Driver internal irq-callback for spi0
 */
static void spi0_irq_callback(void);

IRQ_BUILD_HANDLER(SPI0_IRQ_HANDLER, &spi0_irq_callback, IRQ_NUM_SPI0, IRQ_DISABLED)

// --------------------------------------------------------------------------------

/**
 * @brief Timer to chck for the timeout on receiving a couple of bytes
 */
TIME_MGMN_BUILD_STATIC_TIMER_U16(SPI0_TRX_TIMER)

// --------------------------------------------------------------------------------

/**
 * @brief Access the SPI0-Register of the RP2040
 * 
 * @return R/W reference to the RP2040 SPI0-register
 */
static inline RP2040_SPI_REG* get_spi0_reg(void) {
    return (RP2040_SPI_REG*)(RP2040_SPI0_REG_BASE_ADDRESS);
}

/**
 * @brief Access the SPI1-Register of the RP2040
 * 
 * @return R/W reference to the RP2040 SPI1-register
 */
static inline RP2040_SPI_REG* get_spi1_reg(void) {
    return (RP2040_SPI_REG*)(RP2040_SPI1_REG_BASE_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @brief Enables the given spi-instance.
 * After the spi has been enabled it can transfer data.
 * 
 * @param p_spi spi-instance to enable
 */
static inline void spi_driver_enable(RP2040_SPI_REG* p_spi, u8 enable) {
    if (enable == SPI_DRIVER_ENABLE) {
        p_spi->SSPCR1 |= SPI_DRIVER_SSPCR1_ENABLE;
    } else {
        p_spi->SSPCR1 &= ~SPI_DRIVER_SSPCR1_ENABLE;
    }
}

/**
 * @brief Checks if the given spi-instance is enabled or not
 * by matching the internal hw-register.
 * 
 * @param p_spi reference to the spi where to check for
 * @return 0 if the spi is not enabled, otherweise a value unequal to zero.
 */
static inline u8 spi_driver_is_enabled(RP2040_SPI_REG* p_spi) {
    return p_spi->SSPCR1 & SPI_DRIVER_SSPCR1_ENABLE;
}

/**
 * @brief Checks if the TX HW-Fifo of the given spi-instance is empty or not.
 * 
 * @param p_spi instance to check for
 * @return 1 the TX HW-Fifo is empty, otherwise 0
 */
static inline u8 spi_driver_tx_fifo_empty(RP2040_SPI_REG* p_spi) {
    return (p_spi->SSPSR & SPI_DRIVER_SSPSR_TFE) != 0;
}

/**
 * @brief Checks if the TX HW-FIFO of the given spi-instance is full or not.
 * 
 * @param p_spi instance to check for
 * @return 1 the TX HW-Fifo is full, otherwise 0
 */
static inline u8 spi_driver_tx_fifo_full(RP2040_SPI_REG* p_spi) {
    return (p_spi->SSPSR & SPI_DRIVER_SSPSR_TNF) == 0;
}

/**
 * @brief Checks if the given spi-instance has currently bytes
 * inside of the rx-fifo.
 * 
 * @param p_spi spi-instance to check for non-empty rx-fifo
 * @return 1 if the rx-fifo of the spi-instance is empty, otherwise 0.
 */
static inline u8 spi_driver_rx_fifo_empty(RP2040_SPI_REG* p_spi) {
    return (p_spi->SSPSR & SPI_DRIVER_SSPSR_RNE) == 0;
}

/**
 * @brief Checks if the RX-Fifo of the given spi isntance is full.
 * 
 * @param p_spi isntance to check the RX-Fifo
 * @return 1 the RX-Fifo is full, otherwise 0
 */
static inline u8 spi_driver_rx_fifo_full(RP2040_SPI_REG* p_spi) {
    return (p_spi->SSPSR & SPI_DRIVER_SSPSR_RFF) != 0;
}

/**
 * @brief Checks if the given spi-instance is busy.
 * If the spi-instance is busy it cannot transmit data
 * 
 * @param p_spi spi-instance to check for
 * @return 1 if the rx-fifo of the spi-instance is empty, otherwise 0.
 */
static inline u8 spi_driver_is_busy(RP2040_SPI_REG* p_spi) {
    return (p_spi->SSPSR & SPI_DRIVER_SSPSR_BSY) != 0;
}

/**
 * @brief Checks if the given spi-instance is configured as master.
 * 
 * @param p_spi spi-instance to check
 * @return 1 spi is configured as master, otherwise 0
 */
static inline u8 spi_driver_is_master(RP2040_SPI_REG* p_spi) {
    return (p_spi->SSPCR1 & SPI_DRIVER_SSPCR1_SLAVE) == 0;
}

/**
 * @brief Clears the RX-Fifo of the given spi-isntance
 * by reading all available data bytes without storing them.
 * 
 * @param p_spi spi-instance where to clear the RX-Fifo
 */
static inline void spi_driver_clear_rx_fifo(RP2040_SPI_REG* p_spi) {
    while (spi_driver_rx_fifo_empty(p_spi) == 0) {
        (void)p_spi->SSPDR;
    }
}

/**
 * @brief Waits for the given spi-instance as long it is busy.
 * 
 * @param p_spi spi-instance to wait for
 */
static inline void spi_driver_wait_for(RP2040_SPI_REG* p_spi) {
    while (spi_driver_is_busy(p_spi)) {

    }
}

// --------------------------------------------------------------------------------

/**
 * @brief Configures the given spi instance depending on the given configuration
 * THis function will disable the spi-instance befor it will be configured.
 * This function does not enable the spi-instance. You need to do it by yourself.
 * 
 * @param p_spi pointer to the spi instance to be configured 
 * @param p_cfg configuration to be used to configure the spi instance
 */
static void spi_driver_configure(
    RP2040_SPI_REG* p_spi,
    TRX_DRIVER_CONFIGURATION* p_cfg
) {
    DEBUG_PASS("spi_driver_configure()");

    // Disable the SPI
    p_spi->SSPCR1 = 0;

    if (p_cfg->module.spi.data_order == DRIVER_SPI_DATA_ORDER_MSB) {
        DEBUG_PASS("spi_driver_configure() - DATA-ORDER-MSB");

    } else {
        DEBUG_PASS("spi_driver_configure() - DATA-ORDER-LSB");

    }

    // Always use the following values
    // - Data-Size: 8 Bits
    // - Frameformat: Motorola SPI frame format
    u32 new_sspcr0 = SPI_DRIVER_SSPCR0_DATA_SIZE_8_BIT;

    switch (p_cfg->module.spi.mode) {
        
        default: // no break;
        case DRIVER_SPI_MODE_0 :
            DEBUG_PASS("spi_driver_configure() - MODE_0");
            new_sspcr0 |= 0;
            break;
        
        case DRIVER_SPI_MODE_1 :
            DEBUG_PASS("spi_driver_configure() - MODE_1");
            new_sspcr0 |= SPI_DRIVER_SSPCR0_SPH_EN;
            break;
        
        case DRIVER_SPI_MODE_2 :
            DEBUG_PASS("spi_driver_configure() - MODE_2");
            new_sspcr0 |= SPI_DRIVER_SSPCR0_SPO_EN;
            break;
        
        case DRIVER_SPI_MODE_3 :
            DEBUG_PASS("spi_driver_configure() - MODE_3");
            new_sspcr0 |= (SPI_DRIVER_SSPCR0_SPO_EN | SPI_DRIVER_SSPCR0_SPH_EN);
            break;
    }

    p_spi->SSPCR0 = new_sspcr0;

    u32 new_sspcpsr = 0;

    switch (p_cfg->module.spi.clk_divider) {
        
        case DRIVER_SPI_CLK_DEVIDER_4 :
            DEBUG_PASS("spi_driver_configure() - CLK_DEVIDER_4");
            new_sspcpsr = SPI_DRIVER_SSPCPSR_4;
            break;
        
        case DRIVER_SPI_CLK_DEVIDER_8 :
            DEBUG_PASS("spi_driver_configure() - CLK_DEVIDER_8");
            new_sspcpsr = SPI_DRIVER_SSPCPSR_8;
            break;
        
        case DRIVER_SPI_CLK_DEVIDER_16 :
            DEBUG_PASS("spi_driver_configure() - CLK_DEVIDER_16");
            new_sspcpsr = SPI_DRIVER_SSPCPSR_16;
            break;
        
        case DRIVER_SPI_CLK_DEVIDER_32 :
            DEBUG_PASS("spi_driver_configure() - CLK_DEVIDER_32");
            new_sspcpsr = SPI_DRIVER_SSPCPSR_32;
            break;
        
        case DRIVER_SPI_CLK_DEVIDER_64 :
            DEBUG_PASS("spi_driver_configure() - CLK_DEVIDER_64");
            new_sspcpsr = SPI_DRIVER_SSPCPSR_64;
            break;
        
        default: // no break;
        case DRIVER_SPI_CLK_DEVIDER_128 :
            DEBUG_PASS("spi_driver_configure() - CLK_DEVIDER_128");
            new_sspcpsr = SPI_DRIVER_SSPCPSR_128;
            break;
    }

    p_spi->SSPCPSR = new_sspcpsr;

    // Setting this register will also enable the SPI peripheral
    u32 new_sspcr1 = 0; //SPI_DRIVER_SSPCR1_ENABLE;

    if (p_cfg->module.spi.is_master == COM_DRIVER_IS_MASTER) {
        DEBUG_PASS("spi_driver_configure() - MASTER");
        new_sspcr1 |= SPI_DRIVER_SSPCR1_MASTER;
    } else {
        DEBUG_PASS("spi_driver_configure() - SLAVE");
        new_sspcr1 |= SPI_DRIVER_SSPCR1_SLAVE;
    }

    p_spi->SSPCR1 = new_sspcr1;

    spi_driver_enable(p_spi, SPI_DRIVER_ENABLE);
}

// --------------------------------------------------------------------------------

/**
 * @brief Checks if the spi-instance is ready for transmission.
 * The spi instance is ready if all of the following conditions apply.
 * - The spi is enabled
 * - The TX HW-FIFO is not full
 * - the TX SW-FBUFFER is not full
 * 
 * @param p_spi spi-instance to check for
 * @param p_msg_buffer TX SW-Buffer of the spi-instance
 * @return 1 if the spi is ready for transmission, otherwise 0
 */
static inline u8 spi_driver_is_ready_for_tx(
    RP2040_SPI_REG* p_spi,
    const LOCAL_MSG_BUFFER_CLASS* p_msg_buffer
) {
    if (spi_driver_is_enabled(p_spi) == 0) {
        DEBUG_PASS("spi_driver_is_ready_for_tx() - NOT ENABLED");
        return 0;
    }

    if (spi_driver_tx_fifo_empty(p_spi) == 0) {
        DEBUG_PASS("spi_driver_is_ready_for_tx() - TX-FIFO NOT EMPTY");
        return 0;
    }

    if (spi_driver_is_busy(p_spi)) {
        DEBUG_PASS("spi_driver_is_ready_for_tx() - IS BUSY");
        return 0;
    }

    if (p_msg_buffer->bytes_available()) {
        DEBUG_PASS("spi_driver_is_ready_for_tx() - SW-BUFFER NOT EMPTY");
        return 0;
    }

    DEBUG_PASS("spi_driver_is_ready_for_tx() - IS READY");
    return 1;
}

/**
 * @brief Checks if the spi-instance is ready for receiption.
 * The spi instance is ready if all of the following conditions apply.
 * - The spi is enabled
 * - The RX HW-FIFO is not full
 * - the RX SW-FBUFFER is not full
 * - The spi-instance is not busy
 * 
 * @param p_spi spi-instance to check for
 * @param p_msg_buffer TX SW-Buffer of the spi-instance
 * @return 1 if the spi is ready for transmission, otherwise 0
 */
static inline u8 spi_driver_is_ready_for_rx(
    RP2040_SPI_REG* p_spi,
    const LOCAL_MSG_BUFFER_CLASS* p_msg_buffer
) {
    if (spi_driver_is_enabled(p_spi) == 0) {
        DEBUG_PASS("spi_driver_is_ready_for_rx() - NOT ENABLED");
        return 0;
    }

    if (p_msg_buffer->bytes_free() == 0) {
        DEBUG_PASS("spi_driver_is_ready_for_tx() - SW-BUFFER IS FULL");
        return 0;
    }

    if (spi_driver_rx_fifo_full(p_spi)) {
        DEBUG_PASS("spi_driver_is_ready_for_tx() - RX-FIFO IS FULL");
        return 0;
    }

    DEBUG_PASS("spi_driver_is_ready_for_rx() - IS READY");
    return 1;
}

// --------------------------------------------------------------------------------

/**
 * @brief Copies as much as possible bytes from the given hw-fifo
 * to the sw-buffer of the given spi-instance.
 * This function does not checks if the spi-instance is enabled.
 * 
 * @param p_spi spi-instance where to read from
 * @param p_msg_buffer sw-buffer where to copy into
 * 
 * @return The number of bytes that have been read from the hw-fifo.
 */
u16 spi_driver_read_from_hw_fifo(
    RP2040_SPI_REG* p_spi,
    const LOCAL_MSG_BUFFER_CLASS* p_msg_buffer
) {

    u16 byte_counter = p_msg_buffer->bytes_available();

    // check if there is space inside of the rx-buffer
    while (p_msg_buffer->bytes_free()) {

        if (spi_driver_rx_fifo_empty(p_spi)) {
            break; // no more bytes available
        }

        p_msg_buffer->add_byte((u8)p_spi->SSPDR);


        #ifdef UNITTEST_GET_FIFO_DATA_CALLBACK
        {
            UNITTEST_GET_FIFO_DATA_CALLBACK
        }
        #endif
    }

    return p_msg_buffer->bytes_available() - byte_counter;
}


/**
 * @brief Copies as much as possible bytes from the given sw-buffer
 * to the hw-fifo of the given spi-instance.
 * If the spi-instance is not enabled, nothing happens.
 * This function will clear the HW-TX-Fifo of the given spi-instance.
 * 
 * @param p_spi spi-instance where to copy into
 * @param p_msg_buffer sw-buffer where to read the data from
 * 
 * @return The number of bytes that have been written into the hw-fifo.
 */
u16 spi_driver_write_to_hw_fifo(
    RP2040_SPI_REG* p_spi,
    const LOCAL_MSG_BUFFER_CLASS* p_msg_buffer
) {

    if (spi_driver_is_enabled(p_spi) == 0) {
        DEBUG_PASS("spi_driver_write_to_hw_fifo() - NOT ENABLED");
        return 0;
    }

    // wait until the previous data is transmitted
    while (spi_driver_is_busy(p_spi)) {
        cpu_watch_forever_loop();
    }

    u16 byte_counter = p_msg_buffer->bytes_available();

    DEBUG_TRACE_word(
        p_msg_buffer->bytes_available(),
        "spi_driver_write_to_hw_fifo() - NUM BYTES TO TRANSMIT:"
    );

    // we do not expecting data on transmission
    spi_driver_clear_rx_fifo(p_spi);

    p_msg_buffer->start_read();

    while (p_msg_buffer->bytes_available()) {

        if (spi_driver_tx_fifo_full(p_spi)) {
            break;
        }

        p_spi->SSPDR = (u32)p_msg_buffer->get_byte();

        #ifdef UNITTEST_SET_FIFO_DATA_CALLBACK
        {
            UNITTEST_SET_FIFO_DATA_CALLBACK
        }
        #endif
    }

    p_msg_buffer->stop_read();

    // we do not expecting data on transmission
    spi_driver_clear_rx_fifo(p_spi);

    // clear the overrun flag
    p_spi->SSPICR = SPI_DRIVER_SSPICR_RORIC;

    DEBUG_TRACE_word(
        byte_counter,
        "spi_driver_write_to_hw_fifo() - NUM BYTES COPIED:"
    );

    // there are still some bytes available
    // after the bytes that are currently inside the tx fifo
    // have been transmitted we fill the remaining bytes into the
    // tx-fifo within the next interrupt.
    // If no more bytes avaialble we disable the interrupt.
    if (p_msg_buffer->bytes_available()) {
        get_spi0_reg()->SSPIMSC |= SPI_DRIVER_SSPIMSC_TXIM;
    } else {
        get_spi0_reg()->SSPIMSC &= ~SPI_DRIVER_SSPIMSC_TXIM;
    }

    return byte_counter - p_msg_buffer->bytes_available();
}

/**
 * @brief Receives num_bytes from a remote host as a spi-master
 * by sending dummy bytes to generate a clock-signal.
 * This function blocks.
 * 
 * @param p_spi spi isntance where to receive from
 * @param p_msg_buffer sw-buffer where to put the received data into
 * @param num_bytes number of bytes to receive 
 * @return the number of bytes that have been received.
 */
static u16 spi_driver_receive(
    RP2040_SPI_REG* p_spi,
    const LOCAL_MSG_BUFFER_CLASS* p_msg_buffer,
    u16 num_bytes
) {

    u16 num_bytes_rx = p_msg_buffer->bytes_available();
    spi_driver_clear_rx_fifo(p_spi);

    while (num_bytes) {

        // write as much bytes into the tx fifo
        while (spi_driver_tx_fifo_full(p_spi) == 0) {

            p_spi->SSPDR = (u32)SPI_DRIVER_PADDING_BYTE;
            if (--num_bytes == 0) {
                break;
            }
        }

        spi_driver_wait_for(p_spi);
        spi_driver_read_from_hw_fifo(p_spi, p_msg_buffer);
    }

    num_bytes_rx = p_msg_buffer->bytes_available() - num_bytes_rx;
    DEBUG_TRACE_word(num_bytes_rx, "spi_driver_receive() - NUM-BYTES RECEIVED:");

    return num_bytes_rx;
}

// --------------------------------------------------------------------------------

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_initialize
 */
void spi0_driver_initialize(void) {
    DEBUG_PASS("spi0_driver_initialize()");

    rp2040_reset_spi0();

    SPI0_TX_BUFFER_clear_all();
    SPI0_RX_BUFFER_clear_all();

    if (SPI0_IRQ_HANDLER_init() != IRQ_INTERFACE_OK) {
        DEBUG_PASS("spi0_driver_initialize() - SPI0_IRQ_HANDLER_init() HAS FAILED");
        return;
    }
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_configure
 */
void spi0_driver_configure(TRX_DRIVER_CONFIGURATION* p_cfg) {
    spi_driver_configure(get_spi0_reg(), p_cfg);
    SPI0_IRQ_HANDLER_set_enabled(IRQ_ENABLED);
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_power_off
 */
void spi0_driver_power_off(void) {
    spi_driver_enable(get_spi0_reg(), SPI_DRIVER_DISABLE);
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_bytes_available
 */
u16 spi0_driver_bytes_available(void) {
    return SPI0_RX_BUFFER_bytes_available();
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_get_N_bytes
 */
u16 spi0_driver_get_N_bytes(u16 num_bytes, u8* p_buffer_to) {

    SPI0_RX_BUFFER_start_read();
    u16 num_bytes_read = SPI0_RX_BUFFER_get_N_bytes(num_bytes, p_buffer_to);
    SPI0_RX_BUFFER_stop_read();

    DEBUG_TRACE_N(num_bytes, p_buffer_to, "spi0_driver_get_N_bytes()");

    return num_bytes_read;
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_set_N_bytes
 */
u16 spi0_driver_set_N_bytes(u16 num_bytes, const u8* p_buffer_from) {

    DEBUG_TRACE_N(num_bytes, p_buffer_from, "spi0_driver_set_N_bytes()");

    SPI0_TX_BUFFER_start_write(); // this will delete all data added so far
    u16 bytes_stored = SPI0_TX_BUFFER_add_N_bytes(num_bytes, p_buffer_from);
    SPI0_TX_BUFFER_stop_write();

    DEBUG_TRACE_word(bytes_stored, "spi0_driver_set_N_bytes() - NUM BYTES STORED");

    // if TX is already active, we start transmitting immediately
    if (SPI0_STATUS_is_set(SPI0_STATUS_TX_ACTIVE)) {
        spi_driver_write_to_hw_fifo(get_spi0_reg(), SPI0_TX_BUFFER_get_class());
    }

    return bytes_stored;
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_is_ready_for_tx
 */
u8 spi0_driver_is_ready_for_tx(void) {
    return spi_driver_is_ready_for_tx(
        get_spi0_reg(),
        SPI0_TX_BUFFER_get_class()
    );
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_is_ready_for_rx
 */
u8 spi0_driver_is_ready_for_rx(void) {
    return spi_driver_is_ready_for_rx(
        get_spi0_reg(),
        SPI0_TX_BUFFER_get_class()
    );
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_start_rx
 */
void spi0_driver_start_rx(u16 num_of_rx_bytes) {

    (void) num_of_rx_bytes;

    SPI0_STATUS_set(SPI0_STATUS_RX_ACTIVE);

    if (spi_driver_is_master(get_spi0_reg())) {

        // if we are a master, we generate the clock signal
        // and send some dummy-bytes
        spi_driver_clear_rx_fifo(get_spi0_reg());
        spi_driver_receive(
            get_spi0_reg(),
            SPI0_RX_BUFFER_get_class(),
            num_of_rx_bytes
        );

    } else {

        // if we are the client
        // we simply activate receiving
        // We need to wait for the master and his clock-signal
    }

    DEBUG_PASS("spi0_driver_start_rx()");

    SPI0_RX_BUFFER_start_write();
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_wait_for_rx
 */
void spi0_driver_wait_for_rx(u16 num_bytes, u16 timeout_ms) {

    SPI0_TRX_TIMER_start();

    /**
     * @brief Received data is processed within the
     * interrupt-routine.
     */
    while (SPI0_RX_BUFFER_bytes_available() < num_bytes) {
        if (SPI0_TRX_TIMER_is_up(timeout_ms)) {
            DEBUG_PASS("spi0_driver_wait_for_rx() - Timeout !!! ---");
            break;
        }
    }
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_stop_rx
 */
void spi0_driver_stop_rx(void) {
    DEBUG_PASS("spi0_driver_stop_rx()");
    SPI0_STATUS_unset(SPI0_STATUS_RX_ACTIVE);
    SPI0_RX_BUFFER_stop_write();
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_start_tx
 */
void spi0_driver_start_tx(void) {

    SPI0_STATUS_set(SPI0_STATUS_TX_ACTIVE);

    if (SPI0_TX_BUFFER_bytes_available()) {
        spi_driver_write_to_hw_fifo(get_spi0_reg(), SPI0_TX_BUFFER_get_class());
    }
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_wait_for_tx
 */
void spi0_driver_wait_for_tx(u16 num_bytes, u16 timeout_ms) {

}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_stop_tx
 */
void spi0_driver_stop_tx(void) {
    SPI0_STATUS_unset(SPI0_STATUS_TX_ACTIVE);
    get_spi0_reg()->SSPIMSC &= ~SPI_DRIVER_SSPIMSC_TXIM;
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_clear_rx_buffer
 */
void spi0_driver_clear_rx_buffer(void) {
    SPI0_RX_BUFFER_clear_all();
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_clear_tx_buffer
 */
void spi0_driver_clear_tx_buffer(void) {
    SPI0_TX_BUFFER_clear_all();
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_set_address
 */
void spi0_driver_set_address (u8 addr) {
    (void) addr;
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_mutex_request
 */
u8 spi0_driver_mutex_request(void) {
    return 0;
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_mutex_release
 */
void spi0_driver_mutex_release(u8 m_id) {

}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ callback for the SPI0 instance of the RP2040
 * If there are some bytes available inside of the SPI0 TX buffer
 * and TX is enabled, these bytes will be copied into the HW-fifo of the spi0 instance
 */
static void spi0_irq_callback(void){
    DEBUG_PASS("spi0_irq_callback()");
    
    u32 irq_status = get_spi0_reg()->SSPMIS;
    get_spi0_reg()->SSPICR = SPI_DRIVER_SSPICR_CLEAR_ALL;

    // check if rx is activated
    // if not discard data
    if (irq_status & SPI_DRIVER_SSPMIS_RXMIS) {
        if (SPI0_STATUS_is_set(SPI0_STATUS_RX_ACTIVE)) {

            spi_driver_read_from_hw_fifo(
                get_spi0_reg(),
                SPI0_RX_BUFFER_get_class()
            );

        } else {
            spi_driver_clear_rx_fifo(get_spi0_reg() );
        }
    }

    // Check if there are some bytes that are allowed to be transfered
    if (SPI0_STATUS_is_set(SPI0_STATUS_TX_ACTIVE)) {
        spi_driver_write_to_hw_fifo(
            get_spi0_reg(),
            SPI0_TX_BUFFER_get_class()
        );
    }
}

// --------------------------------------------------------------------------------
