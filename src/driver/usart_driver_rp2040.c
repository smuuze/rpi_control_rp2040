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
 * @file    usart0_driver_RP2040.c
 * @author  Sebastian Lesse
 * @date    2022 / 07 / 26
 * @brief   Implementation of a uart0 driver ussable for the Raspberry Pi Pico
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
#include "rp2040_clock_frequencies.h"

// --------------------------------------------------------------------------------

#include "cfg_driver_interface.h"
#include "local_msg_buffer.h"
#include "driver/communication/usart/usart0_driver.h"
#include "local_module_status.h"

#include "driver/irq/irq_interface.h"

// --------------------------------------------------------------------------------

#define USART_DRIVER_RX_UNLIMITED           0xFFFF

// --------------------------------------------------------------------------------

/**
 * @brief Deepth of the uart1 fifo of the rp2040
 * in number of bytes
 * 
 */
#define RP2040_UART0_FIFO_SIZE              __UNSIGNED(0)

// --------------------------------------------------------------------------------

#ifndef USART0_DRIVER_MAX_NUM_BYTES_TRANSMIT_BUFFER
#define USART0_DRIVER_MAX_NUM_BYTES_TRANSMIT_BUFFER     __UNSIGNED(8)
#endif

#ifndef USART0_DRIVER_MAX_NUM_BYTES_RECEIVE_BUFFER
#define USART0_DRIVER_MAX_NUM_BYTES_RECEIVE_BUFFER      __UNSIGNED(8)
#endif

// --------------------------------------------------------------------------------

#define LOCAL_USART_STATUS_RX_ACTIVE            __UNSIGNED(0)
#define LOCAL_USART_STATUS_TX_ACTIVE            __UNSIGNED(1)

BUILD_MODULE_STATUS_FAST(UART0_STATUS_REG, 2)

// --------------------------------------------------------------------------------

#define UART_DRIVER_POWER_ON                    __UNSIGNED(1)
#define UART_DRIVER_POWER_OFF                   __UNSIGNED(0)

// --------------------------------------------------------------------------------

#define UART_DRIVER_UARTCR_UART_ENABLE          __UNSIGNED(0x00000001)
#define UART_DRIVER_UARTCR_TX_ENABLE            __UNSIGNED(0x00000100)
#define UART_DRIVER_UARTCR_RX_ENABLE            __UNSIGNED(0x00000200)

#define UART_DRIVER_UARTLCR_H_PARITY_NONE       __UNSIGNED(0x00000000)
#define UART_DRIVER_UARTLCR_H_PARITY_EN         __UNSIGNED(0x00000002)
#define UART_DRIVER_UARTLCR_H_PARITY_EVEN      (__UNSIGNED(0x00000004) | UART_DRIVER_UARTLCR_H_PARITY_EN)
#define UART_DRIVER_UARTLCR_H_PARITY_ODD       (__UNSIGNED(0x00000000) | UART_DRIVER_UARTLCR_H_PARITY_EN)
#define UART_DRIVER_UARTLCR_H_FIFO_ENABLE       __UNSIGNED(0x00000010)

#define UART_DRIVER_UARTLCR_H_STOPBITS_1        __UNSIGNED(0x00000000)
#define UART_DRIVER_UARTLCR_H_STOPBITS_2        __UNSIGNED(0x00000008)

#define UART_DRIVER_UARTLCR_H_DATA_BITS_8       __UNSIGNED(0x00000060)
#define UART_DRIVER_UARTLCR_H_DATA_BITS_7       __UNSIGNED(0x00000040)
#define UART_DRIVER_UARTLCR_H_DATA_BITS_6       __UNSIGNED(0x00000020)
#define UART_DRIVER_UARTLCR_H_DATA_BITS_5       __UNSIGNED(0x00000000)

#define UART_DRIVER_UARTDMACR_TX_DMA_EN         __UNSIGNED(0x00000002)
#define UART_DRIVER_UARTDMACR_RX_DMA_EN         __UNSIGNED(0x00000001)

/**
 * @brief If this bit is set in the UARTFR (Flags-Register)
 * the Transmit-Fifo is full. No more data is accepted.
 * 
 */
#define UART_DRIVER_UARTFR_TXFF_BITS            __UNSIGNED(0x00000020)

/**
 * @brief If this bit is set in the UARTFR (Flags-Register)
 * there are received data-bytes avaialble
 * 
 */
#define UART_DRIVER_UARTFR_RXFE_BITS            __UNSIGNED(0x00000010)

#define UART_DRIVER_UARTIFLS_TX_FIFO_LEVEL_1_8  __UNSIGNED(0x00000000)
#define UART_DRIVER_UARTIFLS_TX_FIFO_LEVEL_1_4  __UNSIGNED(0x00000001)
#define UART_DRIVER_UARTIFLS_TX_FIFO_LEVEL_1_2  __UNSIGNED(0x00000002)
#define UART_DRIVER_UARTIFLS_TX_FIFO_LEVEL_3_4  __UNSIGNED(0x00000003)
#define UART_DRIVER_UARTIFLS_TX_FIFO_LEVEL_7_8  __UNSIGNED(0x00000004)

#define UART_DRIVER_UARTIFLS_RX_FIFO_LEVEL_1_8  __UNSIGNED(0x00000000)
#define UART_DRIVER_UARTIFLS_RX_FIFO_LEVEL_1_4  __UNSIGNED(0x00000008)
#define UART_DRIVER_UARTIFLS_RX_FIFO_LEVEL_1_2  __UNSIGNED(0x00000010)
#define UART_DRIVER_UARTIFLS_RX_FIFO_LEVEL_3_4  __UNSIGNED(0x00000018)
#define UART_DRIVER_UARTIFLS_RX_FIFO_LEVEL_7_8  __UNSIGNED(0x00000020)

#define UART_DRIVER_UARTICR_CLEAR_ALL           __UNSIGNED(2047)

#define UART_DRIVER_UARTMIS_RTIM                __UNSIGNED(0x00000040)
#define UART_DRIVER_UARTMIS_TXIM                __UNSIGNED(0x00000020)
#define UART_DRIVER_UARTMIS_RXIM                __UNSIGNED(0x00000010)

// --------------------------------------------------------------------------------

/**
 * @brief FIFO TX-threshold at which the IRQ is activated.
 * Can be set via config.h by the user-application.
 * The default value is 1/2
 * 
 */
#ifndef RP2040_UART_DRIVER_FIFO_TX_LEVEL
#define RP2040_UART_DRIVER_FIFO_TX_LEVEL    UART_DRIVER_UARTIFLS_TX_FIFO_LEVEL_1_8
#endif

#ifndef RP2040_UART_DRIVER_FIFO_RX_LEVEL
#define RP2040_UART_DRIVER_FIFO_RX_LEVEL    UART_DRIVER_UARTIFLS_RX_FIFO_LEVEL_1_8
#endif

// --------------------------------------------------------------------------------

/**
 * @brief Structure to control the UART register of the RP2040
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
    io_ro_32 fr;

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
    io_ro_32 ris;

    /**
     * @brief Masked Interrupt Status Register, UARTMIS
     * See datasheet ch. 4.2.8 - UARTDR Register on page 456
     * for more details.
     */
    io_ro_32 mis;

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

// --------------------------------------------------------------------------------

/**
 * @brief transmit and receive buffer of the uart1 instance of the rp2040
 * Both buffers must only b used within the UART0 isntance.
 * 
 */

BUILD_LOCAL_MSG_BUFFER(
    static inline,
    USART0_TX_BUFFER,
    USART0_DRIVER_MAX_NUM_BYTES_TRANSMIT_BUFFER
)
BUILD_LOCAL_MSG_BUFFER_CLASS(USART0_TX_BUFFER)

BUILD_LOCAL_MSG_BUFFER(
    static inline,
    USART0_RX_BUFFER,
    USART0_DRIVER_MAX_NUM_BYTES_RECEIVE_BUFFER
)
BUILD_LOCAL_MSG_BUFFER_CLASS(USART0_RX_BUFFER)

// --------------------------------------------------------------------------------

/**
 * @brief Driver internal irq-callback for uart0
 */
static void uart0_irq_callback(void);

IRQ_BUILD_HANDLER(UART0_IRQ_HANDLER, &uart0_irq_callback, IRQ_NUM_UART0, IRQ_DISABLED)

// --------------------------------------------------------------------------------

/**
 * @brief Returns pointer to access the uart0 register
 * 
 * @return Pointer to the uart0 register
 */
static inline RP2040_UART_REG* uart_driver_get_uart0_reg(void) {
    return ((RP2040_UART_REG*)RP2040_UART0_REG_BASE_ADDRESS);
}

/**
 * @brief Returns pointer to access the uart1 register
 * 
 * @return Pointer to the uart1 register
 */
static inline RP2040_UART_REG* uart_driver_get_uart1_reg(void) {
    return ((RP2040_UART_REG*)RP2040_UART1_REG_BASE_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @brief Enables/disables the given uart-instance.
 * 
 * @param p_uart uart-instance to power on/off
 * @param power_on UART_DRIVER_POWER_ON uart will be powered on
 *                 otherwise uart will be powered off
 */
static void uart_driver_power(RP2040_UART_REG* p_uart, u32 power_on) {

    if (power_on == UART_DRIVER_POWER_ON) {

        DEBUG_PASS("uart_driver_power() - POWER ON");
        p_uart->cr |= UART_DRIVER_UARTCR_UART_ENABLE;

    } else {

        DEBUG_PASS("uart_driver_power() - POWER OFF");
        p_uart->cr &= ~( UART_DRIVER_UARTCR_UART_ENABLE
                   | UART_DRIVER_UARTCR_TX_ENABLE
                   | UART_DRIVER_UARTCR_RX_ENABLE);
    }
}

// --------------------------------------------------------------------------------

/**
 * @brief Starts the TX of the given uart-instance.
 * 
 * @param p_uart uart-instance where to start TX
 */
static void uart_driver_start_tx(RP2040_UART_REG* p_uart) {

    // Enable TX
    p_uart->cr |= UART_DRIVER_UARTCR_TX_ENABLE;
}

// --------------------------------------------------------------------------------

/**
 * @brief Starts the RX of the given uart-instance.
 * 
 * @param p_uart uart-instance where to start RX
 */
static void uart_driver_start_rx(RP2040_UART_REG* p_uart) {

    // Enable RX
    p_uart->cr |= UART_DRIVER_UARTCR_RX_ENABLE;
}

// --------------------------------------------------------------------------------

/**
 * @brief Configures the given baudrate.
 * 
 * @param p_uart refefernce to the uart-instance where to set the baudrate
 * @param baudrate baudrate to be used
 * @param clk_peri_freq current frequency of the CLK-PERI in Hz
 * @return -1 configuring the baudrate has failed,
 * otherwise baudrate was configured successful.
 */
static i32 uart_driver_configure_baudrate(RP2040_UART_REG* p_uart, u32 baudrate, u32 clk_peri_freq) {

    if (clk_peri_freq == 0) {
        DEBUG_PASS("uart_driver_configure_baudrate() - CLOCK-FREQ IST ZERO - ABORT!");
        return -1;
    }

    DEBUG_TRACE_long(clk_peri_freq, "uart_driver_configure_baudrate() - CLOCK-FREQ:");
    DEBUG_TRACE_long(baudrate, "uart_driver_configure_baudrate() - BAUD-RATE:");

    /**
     * @brief See datasheet ch. 4.2.7.1 "Baud Rate Calculation"
     * The original equation is:
     * 
     *      Baud Rate Divisor = (CLK_PERI * 10^6) / (16 * BAUDRATE)
     * 
     * For better handling a fixpoint format is used. the factor is
     * The fixpoint of the IBRD register is 2^6 = 64. For more precision
     * 2^7 = 128 is used within the calculation. The modifies the above
     * equation as follows:
     * 
     *      FIX(Baud Rate Divisor) = 128 * (CLK_PERI * 10^6) / (16 * BAUDRATE)
     *                             =   8 * (CLK_PERI * 10^6) / BAUDRATE
     * 
     * E.g.
     * 
     *      CLK-PERI = 125.000.000 Hz (125 MHz)
     *      BAUDRAT  = 115.200 Baud
     * 
     *      | Original                          | Fixpoint                                                  |
     *      |-----------------------------------|-----------------------------------------------------------|
     *      | baud_rate_div = 125 MHz           | baud_rate_div = 8 * 125 MHz / 115200                      |
     *      |                 / (16 * 115200)   |               = 8680.56                                   |
     *      |               = 67.8168           |                                                           |
     *      |       -> IBRD = 67                |       -> IRBD = baud_rate_div / 128                       |
     *      |       -> FBRD = 0.8168            |               = baud_rate_div >> 7                        |
     *      | FIXPOINT:                         |               = 67                                        |
     *      |     FIX(FBRD) = FBRD * 64 + 0.5   |   > FIX(FBRD) = (baud_rate_div - (IRBD * 128)) / 2        |
     *      |               = 52                |               = ( (baud_rate_div & 0x7F) + 0.5 * 2 ) / 2  |
     *      |                                   |               = ( 104 + 1) / 2                            |
     *      |                                   |               = 52                                        |
     * 
     */

    u32 baud_rate_div = (8 * clk_peri_freq) / baudrate;
    u32 baud_ibrd     = baud_rate_div >> 7;             // integer part
    u32 baud_fbrd     = __UNSIGNED(0);                  // fractional aprt

    if (baud_ibrd == __UNSIGNED(0)) {

        baud_ibrd = __UNSIGNED(1);
        //baud_fbrd = __UNSIGNED(0); // already set

    } else if (baud_ibrd >= __UNSIGNED(65535)) {

        baud_ibrd = __UNSIGNED(65535);
        //baud_fbrd = __UNSIGNED(0); // already set

    }  else {
        baud_fbrd = ((baud_rate_div & 0x7f) + 1) / 2;
    }

    // Load PL011's baud divisor registers
    p_uart->ibrd = baud_ibrd;
    p_uart->fbrd = baud_fbrd;

    DEBUG_TRACE_long(baud_rate_div, "uart_driver_configure_baudrate() - BAUD-DIV:");
    DEBUG_TRACE_long(baud_ibrd, "uart_driver_configure_baudrate() - BAUD-IBRD:");
    DEBUG_TRACE_long(baud_fbrd, "uart_driver_configure_baudrate() - BADU-FBRD");

    // PL011 needs a (dummy) line control register write to latch in the
    // divisors. We don't want to actually change LCR contents here.
    cpu_atomic_bit_set(&p_uart->lcr_h, __UNSIGNED(0));

    // See datasheet
    return (4 * clk_peri_freq) / (64 * baud_ibrd + baud_fbrd);
}

// --------------------------------------------------------------------------------

/**
 * @brief Configures the given uart instance depending on the given configuration
 * 
 * @param p_uart pointer to the uart instance to be configured 
 * @param p_cfg configuration to be used to configure the uart instance
 */
static void uart_driver_configure(
    RP2040_UART_REG* p_uart, TRX_DRIVER_CONFIGURATION* p_cfg
) {

    /**
     * @brief first: check if the peripheral clock was initialized
     * If not, we does not need to continue.
     */
    u32 clk_peri_freq = clock_driver_peripheral_clk_frequency();
    if (clk_peri_freq == 0) {
        DEBUG_PASS("uart_driver_configure() - CLOCK-FREQ IST ZERO - ABORT!");
        return;
    }

    /**
     * @brief Baudrate to use.
     * By default we use 9600 baud.
     */
    u32 baudrate = __UNSIGNED(9600);

    switch (p_cfg->module.usart.baudrate) {

        default:
            // no break;
            // fall through

        case BAUDRATE_9600:
            // already set
            DEBUG_PASS("uart_driver_configure() - BAUD - 9600");
            break;

        case BAUDRATE_19200:
            DEBUG_PASS("uart_driver_configure() - BAUD - 19200");
            baudrate = __UNSIGNED(19200);
            break;

        case BAUDRATE_38400:
            DEBUG_PASS("uart_driver_configure() - BAUD - 38400");
            baudrate = __UNSIGNED(38400);
            break;

        case BAUDRATE_115200:
            DEBUG_PASS("uart_driver_configure() - BAUD - 115200");
            baudrate = __UNSIGNED(115200);
            break;

        case BAUDRATE_230400:
            DEBUG_PASS("uart_driver_configure() - BAUD - 230400");
            baudrate = __UNSIGNED(230400);
            break;
    }

    /**
     * @brief Setup the following configuration values
     * and set them all together to the lcr_h register
     * of the actual uart-instance
     * - number of stopbits
     * - parity
     * - number of databits
     */
    u32 lcr_h_cfg = __UNSIGNED(0);

    switch (p_cfg->module.usart.parity) {

        default:
            // no break
            // fall through

        case PARITY_NONE : 
            lcr_h_cfg |= UART_DRIVER_UARTLCR_H_PARITY_NONE;
            DEBUG_TRACE_long(lcr_h_cfg, "uart_driver_configure() - PARITY - NONE - lcr_h:");
            break;

        case PARITY_EVEN:
            lcr_h_cfg |= UART_DRIVER_UARTLCR_H_PARITY_EVEN;
            DEBUG_TRACE_long(lcr_h_cfg, "uart_driver_configure() - PARITY - EVEN - lcr_h:");
            break;

        case PARITY_ODD:
            lcr_h_cfg |= UART_DRIVER_UARTLCR_H_PARITY_ODD;
            DEBUG_TRACE_long(lcr_h_cfg, "uart_driver_configure() - PARITY - ODD - lcr_h:");
            break;
    }

    switch (p_cfg->module.usart.databits) {
        default:
            // no break
            // fall through

        case DATABITS_9:
            // not available
            // will use 8 databits here
            // no break
            // fall through

        case DATABITS_8:
            lcr_h_cfg |= UART_DRIVER_UARTLCR_H_DATA_BITS_8;
            DEBUG_TRACE_long(lcr_h_cfg, "uart_driver_configure() - DATABITS - 8 - lcr_h:");
            break;

        case DATABITS_5:
            lcr_h_cfg |= UART_DRIVER_UARTLCR_H_DATA_BITS_5;
            DEBUG_TRACE_long(lcr_h_cfg, "uart_driver_configure() - DATABITS - 5 - lcr_h:");
            break;

        case DATABITS_6:
            lcr_h_cfg |= UART_DRIVER_UARTLCR_H_DATA_BITS_6;
            DEBUG_TRACE_long(lcr_h_cfg, "uart_driver_configure() - DATABITS - 6 - lcr_h:");
            break;

        case DATABITS_7:
            lcr_h_cfg |= UART_DRIVER_UARTLCR_H_DATA_BITS_7;
            DEBUG_TRACE_long(lcr_h_cfg, "uart_driver_configure() - DATABITS - 7 - lcr_h:");
            break;
    }

    switch (p_cfg->module.usart.stopbits) {
        default:
            // no break;
            // fall through

        case STOPBITS_1:
            lcr_h_cfg |= UART_DRIVER_UARTLCR_H_STOPBITS_1;
            DEBUG_TRACE_long(lcr_h_cfg, "uart_driver_configure() - STOPBITS - 1 - lcr_h:");
            break;

        case STOPBITS_2:
            lcr_h_cfg |= UART_DRIVER_UARTLCR_H_STOPBITS_2;
            DEBUG_TRACE_long(lcr_h_cfg, "uart_driver_configure() - STOPBITS - 2 - lcr_h:");
            break;
    }

    // Any LCR writes need to take place before enabling the UART
    // this function must be called before lcr_h is set
    __UNUSED__ u32 baud = uart_driver_configure_baudrate(p_uart, baudrate, clk_peri_freq);
    DEBUG_TRACE_long(baud, "uart_driver_configure() - BAUD-VALUE:");

    cpu_atomic_bit_set(&p_uart->lcr_h, lcr_h_cfg);

    // Enable the UART, both TX and RX
    uart_driver_power(p_uart, UART_DRIVER_POWER_ON);
    uart_driver_start_rx(p_uart);
    uart_driver_start_tx(p_uart);
    
    // Enable FIFOs
    DEBUG_PASS("uart_driver_configure() - ENABLE FIFOs");
    cpu_atomic_bit_set(&p_uart->lcr_h, UART_DRIVER_UARTLCR_H_FIFO_ENABLE);

    // enable RX / TX IRQs
    // RXIM and RTIM are required to check for available rx-data via IRQ.
    // - RX asserts when >=4 characters are in the RX-FIFO
    // - RT asserts when there are >=1 characters and no more have been received for 32 bit periods.
    p_uart->imsc = UART_DRIVER_UARTMIS_TXIM | UART_DRIVER_UARTMIS_RXIM;
    
    UART0_IRQ_HANDLER_set_enabled(IRQ_ENABLED);
    irq_set_enabled(IRQ_NUM_UART0, IRQ_ENABLED);

    // set the FIFO threshold levels
    DEBUG_PASS("uart_driver_configure() - SET FIFO THRESHOLD");
    p_uart->ifls = RP2040_UART_DRIVER_FIFO_TX_LEVEL | RP2040_UART_DRIVER_FIFO_RX_LEVEL;

    // Always enable DREQ signals -- no harm in this if DMA is not listening
    p_uart->dmacr = UART_DRIVER_UARTDMACR_TX_DMA_EN
                  | UART_DRIVER_UARTDMACR_RX_DMA_EN;
}

// --------------------------------------------------------------------------------

/**
 * @brief Check if the given uart-inatance is able to accept a new data-byte
 * for transmission.
 * 
 * @param p_uart_inst pointer to the uart-instance to use
 */
static inline u8 usart_driver_is_ready_for_tx(RP2040_UART_REG* p_uart_inst) {

    /**
     * @brief if the transmit FIFO full flag is set
     * the FIFO cannot accept another data-byte.
     * Otherwise one more byte can be stored.
     * 
     */
    return !(p_uart_inst->fr & UART_DRIVER_UARTFR_TXFF_BITS);
}

// --------------------------------------------------------------------------------

/**
 * @brief Check if the RX-fifo of the given uart-isntance is empty or not
 * 
 * @param p_uart_inst uart-instance where to check for an empty fifo
 * @return non-zero if the rx-fifo is empty, otherwise zero.
 */
static inline u8 uart_driver_is_rx_fifo_empty(RP2040_UART_REG* p_uart_inst) {

    /**
     * @brief Receive FIFO empty. The meaning of this
     * bit depends on the state of the FEN bit in the
     * UARTLCR_H Register. If the FIFO is disabled,
     * this bit is set when the receive holding register
     * is empty. If the FIFO is enabled, the RXFE bit is
     * set when the receive FIFO is empty.
     * 
     */
    return (p_uart_inst->fr & UART_DRIVER_UARTFR_RXFE_BITS);
}

// --------------------------------------------------------------------------------

/**
 * @brief Copies num_bytes (as maximum) bytes from p_buffer_from into the hw-fifo
 * of p_uart_inst. If not all bytes fit into the hw-fifo the remaining bytes will
 * not be copied.
 * 
 * @param p_uart_inst reference to the uart-instance where to write the data into
 * @param num_bytes number of bytes to write
 * @param p_buffer_from buffer where to read from
 * @return the number of bytes that have been copied into the uart-fifo
 */
static u16 uart_driver_add_bytes_to_fifo(
    RP2040_UART_REG* p_uart_inst, u32 num_bytes, const u8* const p_buffer_from
) {

    u32 i = __UNSIGNED(0);
    for ( ; i < num_bytes; ++i) {

        if (!usart_driver_is_ready_for_tx(p_uart_inst)) {
            DEBUG_TRACE_long(i, "uart_driver_add_bytes_to_fifo() - FIFO FULL - BYTES WRITTEN:");
            break;
        }

        p_uart_inst->data = p_buffer_from[i];

        #ifdef UNITTEST_SET_FIFO_DATA_CALLBACK
        {
            UNITTEST_SET_FIFO_DATA_CALLBACK
        }
        #endif
    }

    return i;
}

// --------------------------------------------------------------------------------

/**
 * @brief Copies the bytes of the given message-buffer into the fifo of the given
 * uart-instance. Only the number of bytes that fit into the fifo are copied.
 * The remaining bytes will stay inside of the message-buffer
 * 
 * @param p_uart_inst instance of the uart where to fill the fifo
 * @param p_msg_buffer class-object of the message-buffer from where to read the bytes
 */
static void uart_driver_copy_buffer_to_fifo(
    RP2040_UART_REG* p_uart_inst, const LOCAL_MSG_BUFFER_CLASS* p_msg_buffer
) {

    p_msg_buffer->start_read();

    while (p_msg_buffer->bytes_available()) {

        if (!usart_driver_is_ready_for_tx(p_uart_inst)) {
            DEBUG_PASS("uart_driver_copy_buffer_to_fifo() - FIFO FULL");
            break;
        }

        p_uart_inst->data = p_msg_buffer->get_byte();

        #ifdef UNITTEST_SET_FIFO_DATA_CALLBACK
        {
            UNITTEST_SET_FIFO_DATA_CALLBACK
        }
        #endif
    }

    p_msg_buffer->stop_read();
}

// --------------------------------------------------------------------------------

/**
 * @brief Copies the bytes of the rx-fifo into the rx-buffer.
 * 
 * @param p_uart_inst uart-instance from where the bytes are read
 * @param p_msg_buffer message buffer where to store the bytes into
 */
static void uart_driver_copy_fifo_to_buffer (
    RP2040_UART_REG* p_uart_inst, const LOCAL_MSG_BUFFER_CLASS* p_msg_buffer
) {

    p_msg_buffer->start_write();

    while (p_msg_buffer->bytes_free()) {

        if (uart_driver_is_rx_fifo_empty(p_uart_inst)) {
            DEBUG_PASS("uart_driver_copy_fifo_to_buffer() - FIFO IS EMPTY");
            break;
        }

        #ifdef UNITTEST_GET_FIFO_DATA_CALLBACK
        {
            UNITTEST_GET_FIFO_DATA_CALLBACK
        }
        #endif

        u32 data = p_uart_inst->data;
        if (data > (0xFF)) {
            // this is an error (OE / PE / BE / FE)
            DEBUG_TRACE_long(data, "uart_driver_copy_fifo_to_buffer() - FIFO ERROR:");
            break;
        }

        DEBUG_TRACE_long(data, "uart_driver_copy_fifo_to_buffer() - DATA:");
        p_msg_buffer->add_byte(data);
    }

    p_msg_buffer->stop_write();
}

// --------------------------------------------------------------------------------

/**
 * @brief Sets num_bytes from p_msg_buffer into the uart-fifo of p_uart_inst
 * and/or the message-buffer p_msg_buffer.
 * First, this fucntions tries to write as much as possible bytes into the fifo
 * of the given uart-instance. If there are bytes left that does not fit into the
 * fifo these bytes will be written into the message-buffer.
 * 
 * @param p_uart_inst uart-isntance where to write the fifo
 * @param p_msg_buffer message buffer where to write remaining bytes into
 * @param num_bytes number of bytes to write
 * @param p_buffer_from bufer holding the bytes to copy into the uart
 * @return summary of bytes that have been written into the uart-fifo and/or the
 * message-buffer.
 */
static u16 uart_driver_set_N_bytes(
    RP2040_UART_REG* p_uart_inst,
    const LOCAL_MSG_BUFFER_CLASS* p_msg_buffer,
    const u16 num_bytes,
    const u8* const p_buffer_from
) {

    /**
     * @brief try to write as much as possible into the uarts hw-fifo
     */
    u16 bytes_written = uart_driver_add_bytes_to_fifo(
        p_uart_inst,
        num_bytes,
        &p_buffer_from[0]
    );

    DEBUG_TRACE_long(bytes_written, "uart_driver_set_N_bytes() - BYTES WRITTEN INTO FIFO:");

    /**
     * @brief try to write the remaining bytes into the sw-buffer
     */
    if (bytes_written < num_bytes) {
        p_msg_buffer->start_write();
        {
            bytes_written += p_msg_buffer->add_n_bytes(
                num_bytes - bytes_written,
                &p_buffer_from[bytes_written]
            );
        }
        p_msg_buffer->stop_write();
    }

    DEBUG_TRACE_long(bytes_written, "uart_driver_set_N_bytes() - BYTES WRITTEN:");

    /**
     * @brief return the number of bytes that have been stored
     * to inform the user if some of them have not.
     */
    return bytes_written;
}

// --------------------------------------------------------------------------------

/**
 * @see usart0_driver.h#usart0_driver_initialize
 */
void usart0_driver_initialize(void) {

    DEBUG_PASS("usart0_driver_initialize()");

    UART0_STATUS_REG_clear_all();
    USART0_TX_BUFFER_clear_all();
    USART0_RX_BUFFER_clear_all();

    rp2040_reset_uart0();// blocks until uart0 is resetted

    if (UART0_IRQ_HANDLER_init() != IRQ_INTERFACE_OK) {
        DEBUG_PASS("usart0_driver_initialize() - INIT UART0-IRQ-HANDLER HAS FAILED");
        return;
    }
}

/**
 * @see usart0_driver.h#usart0_driver_configure
 */
void usart0_driver_configure(TRX_DRIVER_CONFIGURATION* p_cfg) {
    uart_driver_configure(uart_driver_get_uart0_reg(), p_cfg);
}

/**
 * @see usart0_driver.h#usart0_driver_power_off
 */
void usart0_driver_power_off(void) {
    uart_driver_power(
        uart_driver_get_uart0_reg(),
        UART_DRIVER_POWER_OFF
    );
}

/**
 * @see usart0_driver.h#usart0_driver_bytes_available
 */
u16 usart0_driver_bytes_available(void) {
    return USART0_RX_BUFFER_bytes_available();
}

/**
 * @see usart0_driver.h#usart0_driver_get_N_bytes
 */
u16 usart0_driver_get_N_bytes(u16 num_bytes, u8* p_buffer_to) {
    USART0_RX_BUFFER_start_read();
    u16 bytes_read = USART0_RX_BUFFER_get_N_bytes(num_bytes, p_buffer_to);
    USART0_RX_BUFFER_stop_read();
    return bytes_read;
}

/**
 * @see usart0_driver.h#usart0_driver_set_N_bytes
 */
u16 usart0_driver_set_N_bytes(u16 num_bytes, const u8* const p_buffer_from) {

    return uart_driver_set_N_bytes(
        uart_driver_get_uart0_reg(),
        USART0_TX_BUFFER_get_class(),
        num_bytes,
        p_buffer_from
    );
}

/**
 * @see usart0_driver.h#usart0_driver_is_ready_for_rx
 */
u8 usart0_driver_is_ready_for_rx(void) {
    return 1;
}

/**
 * @see usart0_driver.h#usart0_driver_start_rx
 */
void usart0_driver_start_rx(u16 num_of_rx_bytes) {
    (void) num_of_rx_bytes;
    uart_driver_start_rx(uart_driver_get_uart0_reg());
}

/**
 * @see usart0_driver.h#usart0_driver_wait_for_rx
 */
void usart0_driver_wait_for_rx(u16 num_bytes, u16 timeout_ms) {
    (void) num_bytes;
    (void) timeout_ms;
}

/**
 * @see usart0_driver.h#usart0_driver_stop_rx
 */
void usart0_driver_stop_rx (void) {

}

/**
 * @see usart0_driver.h#usart0_driver_is_ready_for_tx
 */
u8 usart0_driver_is_ready_for_tx (void) {
    return usart_driver_is_ready_for_tx(uart_driver_get_uart0_reg()) ;
}

/**
 * @see usart0_driver.h#usart0_driver_start_tx
 */
void usart0_driver_start_tx (void) {
    uart_driver_start_tx(uart_driver_get_uart0_reg());
}

/**
 * @see usart0_driver.h#usart0_driver_wait_for_tx
 */
void usart0_driver_wait_for_tx(u16 num_bytes, u16 timeout_ms) {
    (void) num_bytes;
    (void) timeout_ms;
}

/**
 * @see usart0_driver.h#usart0_driver_stop_tx
 */
void usart0_driver_stop_tx (void) {

}

/**
 * @see usart0_driver.h#usart0_driver_clear_rx_buffer
 */
void usart0_driver_clear_rx_buffer(void) {
    USART0_RX_BUFFER_clear_all();
}

/**
 * @see usart0_driver.h#usart0_driver_clear_tx_buffer
 */
void usart0_driver_clear_tx_buffer(void) {
    USART0_TX_BUFFER_clear_all();
}

/**
 * @see usart0_driver.h#usart0_driver_set_address
 */
void usart0_driver_set_address (u8 addr) {
    (void) addr;
}

/**
 * @see usart0_driver.h#usart0_driver_mutex_request
 */
u8 usart0_driver_mutex_request(void) {
    return 1;
}

/**
 * @see usart0_driver.h#usart0_driver_mutex_release
 */
void usart0_driver_mutex_release(u8 m_id) {
    (void) m_id;
}

// --------------------------------------------------------------------------------

/**
 * @brief UART0 IRQ Handler
 * 
 */
static void uart0_irq_callback(void) {

    // read the status-reg to find out which IRQ has been fired

    RP2040_UART_REG* p_uart0 = uart_driver_get_uart0_reg();
    u32 irq_mis = p_uart0->mis;
    p_uart0->icr = UART_DRIVER_UARTICR_CLEAR_ALL;

    DEBUG_TRACE_long(irq_mis, "IRQ_20_Handler() - UARTMIS:");

    if (irq_mis & UART_DRIVER_UARTMIS_TXIM) {

        /**
         * @brief If the FIFOs are enabled and the transmit FIFO is equal to
         * or lower than the programmed trigger level then the transmit
         * interrupt is asserted HIGH. The transmit interrupt is cleared by
         * writing data to the transmit FIFO until it becomes greater than
         * the trigger level, or by clearing the interrupt.
         */
    
        DEBUG_TRACE_long(
            USART0_TX_BUFFER_bytes_available(),
            "IRQ_20_Handler() - UART_DRIVER_UARTMIS_TXIM - BYTES-AVAILABLE:"
        );

        if (USART0_TX_BUFFER_bytes_available()) {
            uart_driver_copy_buffer_to_fifo(
                p_uart0,
                USART0_TX_BUFFER_get_class()
            );
        }
    }

    if (uart_driver_is_rx_fifo_empty(p_uart0) != 0) {

        /**
         * @brief There is some data in the receive-fifo - Read it
         */
    
        DEBUG_TRACE_long(
            USART0_RX_BUFFER_bytes_free(),
            "IRQ_20_Handler() - UART_DRIVER_UARTMIS_RXIM - BYTES-FREE:"
        );

        if (USART0_RX_BUFFER_bytes_free()) {
            uart_driver_copy_fifo_to_buffer(
                p_uart0,
                USART0_RX_BUFFER_get_class()
            );
        }
    }

}

// --------------------------------------------------------------------------------

/**
 * @brief UART1 IRQ Handler
 * 
 */
// static void uart1_irq_callback(void) {

//     // read the status-reg to find out which IRQ has been fired

//     RP2040_UART_REG* p_uart = uart_driver_get_uart1_reg();
//     //u32 irq_mis = p_uart->mis;
//     //u32 irq_fr  = p_uart->fr;
//     p_uart->icr = UART_DRIVER_UARTICR_CLEAR_ALL;
// }

// --------------------------------------------------------------------------------
