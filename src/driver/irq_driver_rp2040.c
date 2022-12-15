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
 * @file    irq_interface_rp2040.c
 * @author  Sebastian Lesse
 * @date    2022 / 10 / 26
 * @brief   IRQ implementation useable on a Raspberry Pi RP2040
 *          microcontroller.
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

#include "driver/irq/irq_interface.h"

// --------------------------------------------------------------------------------

/**
 * @brief IRQ clear pending register.
 * Use the Interrupt Clear-Pending Register to clear pending interrupts
 * and determine which interrupts are currently pending
*/
typedef struct {
    io_rw_32 icpr;
} RP2040_IRQ_CLEAR_PENDING_REG;

/**
 * @brief IRQ set enable register.
 * Use the Interrupt Set-Enable Register to enable interrupts and determine which
 * interrupts are currently enabled. If a pending interrupt is enabled, the NVIC
 * activates the interrupt based on its priority. If an interrupt is not enabled,
 * asserting its interrupt signal changes the interrupt state to pending, but the
 * NVIC never activates the interrupt, regardless of its priority.
*/
typedef struct {
    io_rw_32 iser;
} RP2040_IRQ_SET_ENABLE_REG;

/**
 * @brief IRQ clear enable register.
 * Use the Interrupt Clear-Enable Registers to disable interrupts and determine
 * which interrupts are currently enabled.
*/
typedef struct {
    io_rw_32 icer;
} RP2040_IRQ_CLEAR_ENABLE_REG;

// --------------------------------------------------------------------------------

/**
 * @brief array holding the root of all irq-handlers
 */
static IRQ_INTERFACE_HANDLER irq_root[RP2040_NUM_IRQ];

// --------------------------------------------------------------------------------

/**
 * @return reference to the RP2040 IRQ CLEAR PENDING register
 */
static RP2040_IRQ_CLEAR_PENDING_REG* irq_get_clear_pending_reg(void) {
    return (RP2040_IRQ_CLEAR_PENDING_REG*) (RP2040_IRQ_CLEAR_PENDING_REG_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @return reference to the RP2040 IRQ SET ENABLE register
 */
static RP2040_IRQ_SET_ENABLE_REG* irq_get_set_enable_reg(void) {
    return (RP2040_IRQ_SET_ENABLE_REG*) (RP2040_IRQ_SET_ENABLE_REG_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @return reference to the RP2040 IRQ CLEAR ENABLE register
 */
static RP2040_IRQ_CLEAR_ENABLE_REG* irq_get_clear_enable_reg(void) {
    return (RP2040_IRQ_CLEAR_ENABLE_REG*) (RP2040_IRQ_CLEAR_ENABLE_REG_ADDRESS);
}

// --------------------------------------------------------------------------------

/**
 * @see irg/irq_interface.h#irq_driver_init
 */
void irq_driver_init(void) {
    
    DEBUG_PASS("irq_driver_init()");

    for (u8 i = 0; i < RP2040_NUM_IRQ; i++) {
        irq_root[0]._p_next_handler = 0;
    }
}

// --------------------------------------------------------------------------------

/**
 * @see irg/irq_interface.h#irq_add_handler
 */
IRQ_INTERFACE_RET_VAL irq_add_handler(u8 irq_num, IRQ_INTERFACE_HANDLER* p_handler) {

    if (irq_num >= RP2040_NUM_IRQ) {
        DEBUG_TRACE_byte(irq_num, "irq_add_handler() - INVALID irq_num:");
        return IRQ_INTERFACE_UNKNOWN;
    }

    if (p_handler == 0) {
        DEBUG_PASS("irq_add_handler() - p_handler is NULL");
        return IRQ_INTERFACE_INVALID;
    }

    if (p_handler->id != IRQ_HANDLER_INVALID_ID) {
        DEBUG_TRACE_byte(p_handler->id, "irq_add_handler() - handler-id is already set - ID:");
        return IRQ_INTERFACE_OCCUPIED;
    }

    DEBUG_TRACE_byte(irq_num, "irq_add_handler() - adding handler to irq-num:");

    IRQ_INTERFACE_HANDLER* p_act = irq_root[irq_num]._p_next_handler;

    if (p_act == 0) {

        /**
         * @brief there is no handler added yet.
         * Add the first handler to the current irq
         */

        irq_root[irq_num]._p_next_handler = p_handler;
        p_handler->id = 1;
        p_handler->_p_next_handler = 0;

        DEBUG_PASS("irq_add_handler() - first handler added");

    } else {

        u8 counter = 1;
        while (p_act->_p_next_handler != 0) {
            p_act = p_act->_p_next_handler;

            if (counter == IRQ_HANDLER_INVALID_ID) {
                DEBUG_PASS("irq_add_handler() - too much handler");
                return IRQ_INTERFACE_OVERFLOW;
            }
        }

        p_act->_p_next_handler = p_handler;
        p_handler->id = p_act->id + 1;
        p_act->_p_next_handler->_p_next_handler = 0;
    
        DEBUG_TRACE_byte(p_handler->id, "irq_add_handler() - new handler added - ID:");
    }

    return IRQ_INTERFACE_OK;
}

// --------------------------------------------------------------------------------

/**
 * @see irg/irq_interface.h#irq_set_enabled
 */
IRQ_INTERFACE_RET_VAL irq_set_enabled(u8 irq_num, u8 is_enabled) {

    if (irq_num >= RP2040_NUM_IRQ) {
        DEBUG_TRACE_byte(irq_num, "irq_set_enabled() - INVALID irq_num:");
        return IRQ_INTERFACE_UNKNOWN;
    }

    DEBUG_TRACE_byte(irq_num, "irq_set_enabled() - irq_num:");
    DEBUG_TRACE_byte(is_enabled, "irq_set_enabled() - is_enabled:");

    if (is_enabled == IRQ_ENABLED) {

        irq_get_clear_pending_reg()->icpr = (1 << irq_num);
        irq_get_set_enable_reg()->iser    = (1 << irq_num);

    } else {
        irq_get_clear_enable_reg()->icer = (1 << irq_num);
    }

    irq_root[irq_num].is_enabled = is_enabled;

    return IRQ_INTERFACE_OK;
}

// --------------------------------------------------------------------------------

/**
 * @see irg/irq_interface.h#irq_enable_handler
 */
IRQ_INTERFACE_RET_VAL irq_enable_handler(
    u8 irq_num,
    u8 is_enabled,
    IRQ_INTERFACE_HANDLER* p_handler
) {

    if (irq_num >= RP2040_NUM_IRQ) {
        DEBUG_TRACE_byte(irq_num, "irq_set_enabled() - INVALID irq_num:");
        return IRQ_INTERFACE_UNKNOWN;
    }

    if (p_handler == 0) {
        DEBUG_PASS("irq_add_handler() - p_handler is NULL");
        return IRQ_INTERFACE_INVALID;
    }

    DEBUG_TRACE_byte(irq_num, "irq_enable_handler() - irq_num:");
    DEBUG_TRACE_byte(is_enabled, "irq_enable_handler() - is_enabled:");

    p_handler->is_enabled = is_enabled;

    if (is_enabled == IRQ_DISABLED) {

        /**
         * @brief Lets check if there is an other IRQ-handler
         * that is still enabled.
         */

        IRQ_INTERFACE_HANDLER* p_act = irq_root[irq_num]._p_next_handler;

        while (p_act != 0) {

            if (p_act->is_enabled == IRQ_ENABLED) {
                is_enabled = IRQ_ENABLED;
                break;
            } 

            p_act = p_act->_p_next_handler;
        }
    }

    return irq_set_enabled(irq_num, is_enabled);
}

// --------------------------------------------------------------------------------

/**
 * @brief calls all irq-handler of the given irq.
 * 
 * @param irq_num number of the irq that was raised.
 */
static void irq_rp2040_execute(u8 irq_num) {

    if (irq_root[irq_num].is_enabled == IRQ_DISABLED) {
        DEBUG_TRACE_byte(irq_num, "irq_rp2040_execute() - IRQ raised but not enabled - irq_num:");
        return;
    }

    IRQ_INTERFACE_HANDLER* p_act = irq_root[irq_num]._p_next_handler;

    #ifdef TRACER_ENABLED
    {
        if (p_act == 0) {
            DEBUG_TRACE_byte(irq_num, "irq_rp2040_execute() - NO-HANDLER- irq_num:");
        } else {
            DEBUG_TRACE_byte(irq_num, "irq_rp2040_execute() - EXECUTING - irq_num:");
        }
    }
    #endif // TRACER_ENABLED

    while (p_act != 0) {

        if (p_act->p_callback != 0) {

            if (p_act->is_enabled == IRQ_ENABLED) {

                DEBUG_TRACE_byte(p_act->id, "irq_rp2040_execute() - calling handler-id:");
                p_act->p_callback();

            }
            
            #ifdef TRACER_ENABLED
            else {
                DEBUG_TRACE_byte(p_act->id, "irq_rp2040_execute() - disabeld handler - id:");
            }
            #endif

        } 
        
        #ifdef TRACER_ENABLED
        else {
            DEBUG_TRACE_byte(p_act->id, "irq_rp2040_execute() - NULL-POINTER - id:");
        }
        #endif

        p_act = p_act->_p_next_handler;
    }
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 0 - TIMER_00
 */
void IRQ_00_Handler(void) {
    DEBUG_PASS("IRQ_00_Handler() - TIMER_00");
    irq_rp2040_execute(0);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 1 - TIMER_01
 */
void IRQ_01_Handler(void) {
    DEBUG_PASS("IRQ_01_Handler() - TIMER_01");
    irq_rp2040_execute(1);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 2 - TIMER_02
 */
void IRQ_02_Handler(void) {
    DEBUG_PASS("IRQ_02_Handler() - TIMER_02");
    irq_rp2040_execute(2);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 3 - TIMER_03
 */
void IRQ_03_Handler(void) {
    DEBUG_PASS("IRQ_03_Handler() - TIMER_03");
    irq_rp2040_execute(3);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 4 - PWM_WRAP
 */
void IRQ_04_Handler(void) {
    DEBUG_PASS("IRQ_01_Handler() - PWM_WRAP");
    irq_rp2040_execute(4);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 5 - USB_CTRL
 */
void IRQ_05_Handler(void) {
    DEBUG_PASS("IRQ_05_Handler() - USB_CTRL");
    irq_rp2040_execute(5);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 6 - XIP
 */
void IRQ_06_Handler(void) {
    DEBUG_PASS("IRQ_06_Handler() - XIP");
    irq_rp2040_execute(6);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 7 - PIO0_0
 */
void IRQ_07_Handler(void) {
    DEBUG_PASS("IRQ_07_Handler() - PIO0_0");
    irq_rp2040_execute(7);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 8 - PIO0_1 
 */
void IRQ_08_Handler(void) {
    DEBUG_PASS("IRQ_08_Handler() - PIO0_1");
    irq_rp2040_execute(8);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 9 - PIO1_0
 */
void IRQ_09_Handler(void) {
    DEBUG_PASS("IRQ_09_Handler() - PIO1_0");
    irq_rp2040_execute(9);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 10 - PIO1_1
 */
void IRQ_10_Handler(void) {
    DEBUG_PASS("IRQ_10_Handler() - PIO1_1");
    irq_rp2040_execute(10);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 12 - DMA_0
 */
void IRQ_11_Handler(void) {
    DEBUG_PASS("IRQ_11_Handler() - DMA_0");
    irq_rp2040_execute(11);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 13 - DMA_1
 */
void IRQ_12_Handler(void) {
    DEBUG_PASS("IRQ_12_Handler() - DMA_1");
    irq_rp2040_execute(12);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 13 - IO_BANK0
 */
void IRQ_13_Handler(void) {
    DEBUG_PASS("IRQ_13_Handler() - IO_BANK0");
    irq_rp2040_execute(13);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 14 - IO_QSPI
 */
void IRQ_14_Handler(void) {
    DEBUG_PASS("IRQ_14_Handler() - IO_QSPI");
    irq_rp2040_execute(14);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 15 - SIO_PROC0
 */
void IRQ_15_Handler(void) {
    DEBUG_PASS("IRQ_15_Handler() - SIO_PROC0");
    irq_rp2040_execute(15);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 16 - SIO_PROC1
 */
void IRQ_16_Handler(void) {
    DEBUG_PASS("IRQ_16_Handler() - SIO_PROC1");
    irq_rp2040_execute(16);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 17 - CLOCKS
 */
void IRQ_17_Handler(void) {
    DEBUG_PASS("IRQ_17_Handler() - CLOCKS");
    irq_rp2040_execute(17);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 18 - SPI0
 */
void IRQ_18_Handler(void) {
    DEBUG_PASS("IRQ_18_Handler() - SPI0");
    irq_rp2040_execute(18);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 19 - SPI1
 */
void IRQ_19_Handler(void) {
    DEBUG_PASS("IRQ_19_Handler() - SPI1");
    irq_rp2040_execute(19);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 20 - UART_0
 */
void IRQ_20_Handler(void) {
    DEBUG_PASS("IRQ_20_Handler() - UART_0");
    irq_rp2040_execute(20);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 21 - UART_1
 */
void IRQ_21_Handler(void) {
    DEBUG_PASS("IRQ_21_Handler() - UART_1");
    irq_rp2040_execute(21);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 22 - ADC_FIFO
 */
void IRQ_22_Handler(void) {
    DEBUG_PASS("IRQ_22_Handler() - ADC_FIFO");
    irq_rp2040_execute(22);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 23 - I2C_0
 */
void IRQ_23_Handler(void) {
    DEBUG_PASS("IRQ_23_Handler() - I2C_0");
    irq_rp2040_execute(23);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 24 - I2C_1
 */
void IRQ_24_Handler(void) {
    DEBUG_PASS("IRQ_24_Handler() - I2C_1");
    irq_rp2040_execute(24);
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ-Handler number 25 - RTC
 */
void IRQ_25_Handler(void) {
    DEBUG_PASS("IRQ_25_Handler() - RTC");
    irq_rp2040_execute(25);
}

// --------------------------------------------------------------------------------
