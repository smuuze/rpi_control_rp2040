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

// --------------------------------------------------------------------------------

// Sets the Clock-phase bit of SSPCR0
#define SPI_DRIVER_SSPCR0_SPH_EN                    (1 << 7)

// Sets the clock-polarity bit of SSPCR0
#define SPI_DRIVER_SSPCR0_SPO_EN                    (1 << 6)

// Available data-sizes useable for SSPCR0
#define SPI_DRIVER_SSPCR0_DATA_SIZE_4_BIT           (3)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_5_BIT           (4)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_6_BIT           (5)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_7_BIT           (6)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_8_BIT           (7)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_9_BIT           (8)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_10_BIT          (9)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_11_BIT          (10)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_12_BIT          (11)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_13_BIT          (12)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_14_BIT          (13)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_15_BIT          (14)
#define SPI_DRIVER_SSPCR0_DATA_SIZE_16_BIT          (15)

// MASTER / SLAVE mode
#define SPI_DRIVER_SSPCR1_MASTER                    (0x00000000)
#define SPI_DRIVER_SSPCR1_SLAVE                     (0x00000004)

// Enable the SSP peripheral
#define SPI_DRIVER_SSPCR1_ENABLE                    (0x00000002)

// If this bit is set, the transmit-fifo is not full
#define SPI_DRIVER_SSPSR_TNF                        (0x00000002)

// Available clock prescaler values
#define SPI_DRIVER_SSPCPSR_4                        4
#define SPI_DRIVER_SSPCPSR_8                        8
#define SPI_DRIVER_SSPCPSR_16                       16
#define SPI_DRIVER_SSPCPSR_32                       32
#define SPI_DRIVER_SSPCPSR_64                       64
#define SPI_DRIVER_SSPCPSR_128                      128

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

/**
 * @brief Driver internal irq-callback for spi0
 */
static void spi0_irq_callback(void);

IRQ_BUILD_HANDLER(SPI0_IRQ_HANDLER, &spi0_irq_callback, IRQ_NUM_SPI0, IRQ_DISABLED)

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
 * @brief 
 * 
 * @param p_spi 
 * @return u8 
 */
static inline u8 spi_driver_is_ready_for_tx(RP2040_SPI_REG* p_spi) {
    return (p_spi->SSPCPSR & SPI_DRIVER_SSPSR_TNF);
}

// --------------------------------------------------------------------------------

/**
 * @brief Configures the given spi instance depending on the given configuration
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
    u32 new_sspcr1 = SPI_DRIVER_SSPCR1_ENABLE;

    if (p_cfg->module.spi.is_master == COM_DRIVER_IS_MASTER) {
        DEBUG_PASS("spi_driver_configure() - MASTER");
        new_sspcr1 |= SPI_DRIVER_SSPCR1_MASTER;
    } else {
        DEBUG_PASS("spi_driver_configure() - SLAVE");
        new_sspcr1 |= SPI_DRIVER_SSPCR1_SLAVE;
    }

    p_spi->SSPCR1 = new_sspcr1;
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
    
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_bytes_available
 */
u16 spi0_driver_bytes_available(void) {
    return 0;
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_get_N_bytes
 */
u16 spi0_driver_get_N_bytes(u16 num_bytes, u8* p_buffer_to) {
    return 0;
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_set_N_bytes
 */
u16 spi0_driver_set_N_bytes(u16 num_bytes, const u8* p_buffer_from) {
    
    // u16 bytes_written = spi_driver_add_bytes_to_fifo (
    //     get_spi0_reg()
    // );
    return 0;
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_is_ready_for_tx
 */
u8 spi0_driver_is_ready_for_tx(void) {
    return spi_driver_is_ready_for_tx(get_spi0_reg());
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_is_ready_for_rx
 */
u8 spi0_driver_is_ready_for_rx(void) {
    return 0;
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_start_rx
 */
void spi0_driver_start_rx(u16 num_of_rx_bytes) {
    
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_wait_for_rx
 */
void spi0_driver_wait_for_rx(u16 num_bytes, u16 timeout_ms) {
    
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_stop_rx
 */
void spi0_driver_stop_rx(void) {
    
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_start_tx
 */
void spi0_driver_start_tx(void) {
    
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
    
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_clear_rx_buffer
 */
void spi0_driver_clear_rx_buffer(void) {
    
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_clear_tx_buffer
 */
void spi0_driver_clear_tx_buffer(void) {
    
}

/**
 * @see frmwrk/spi0_driver.h#spi0_driver_set_address
 */
void spi0_driver_set_address (u8 addr) {
    
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
 * @see spi_driver_rp2040.c#spi0_irq_callback
 */
static void spi0_irq_callback(void){

}

// --------------------------------------------------------------------------------
