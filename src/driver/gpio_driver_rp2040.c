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
 * @file    gpio_driver_rp2040.c
 * @author  Sebastian Lesse
 * @date    2022 / 07 / 25
 * @brief   Implementation of a gpio-driver usable for the Raspberry Pi Pico 
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

#include "driver/gpio/gpio_interface.h"

// --------------------------------------------------------------------------------

#define RP2040_GPIO_CTRL_FUNC_SEL_MASK      0x1FU

// --------------------------------------------------------------------------------

/**
 * @brief mask to set the pull-up enabled
 * bit in the pad-register of the rp2040
 * 
 */
#define RP2040_GPIO_PAD_PULL_UP_EN          (1U << 3U)

/**
 * @brief mask to set the pull-down enabled
 * bit in the pad-register of the rp2040
 * 
 */
#define RP2040_GPIO_PAD_PULL_DOWN_EN        (1U << 2U)

/**
 * @brief Select the function SPIOn F1
 * for the specific GPIO
 * 
 */
#define RP2040_GPIO_FUNC_F1                 (1U)

/**
 * @brief Select the function UARTn F1
 * for the specific GPIO
 * 
 */
#define RP2040_GPIO_FUNC_F2                 (2U)

/**
 * @brief Select the function I2Sn F1
 * for the specific GPIO
 * 
 */
#define RP2040_GPIO_FUNC_F3                 (3U)

/**
 * @brief Select the function PWMn F1
 * for the specific GPIO
 * 
 */
#define RP2040_GPIO_FUNC_F4                 (4U)

/**
 * @brief Select the function SIO F1
 * for the specific GPIO
 * 
 */
#define RP2040_GPIO_FUNC_F5                 (5U)

/**
 * @brief Select the function PIO0 F1
 * for the specific GPIO
 * 
 */
#define RP2040_GPIO_FUNC_F6                 (6U)

/**
 * @brief Select the function PIO1 F1
 * for the specific GPIO
 * 
 */
#define RP2040_GPIO_FUNC_F7                 (7U)

/**
 * @brief input enable in
 * RP2040_GPIO_PAD_REG->pads[gpio_num]
 * 
 */
#define RP2040_GPIO_PAD_IE_BIT            (1U << 6U)

/**
 * @brief output disable in
 * RP2040_GPIO_PAD_REG->pads[gpio_num]
 * 
 */
#define RP2040_GPIO_PAD_OD_BIT            (1U << 7U)

// --------------------------------------------------------------------------------

INCLUDE_GPIO_REFRENCE(GPIO_PORT_A, GPIO_PIN_0) 
INCLUDE_GPIO_REFRENCE(GPIO_PORT_A, GPIO_PIN_1) 
INCLUDE_GPIO_REFRENCE(GPIO_PORT_A, GPIO_PIN_2)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_A, GPIO_PIN_3) 
INCLUDE_GPIO_REFRENCE(GPIO_PORT_A, GPIO_PIN_4)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_A, GPIO_PIN_5) 
INCLUDE_GPIO_REFRENCE(GPIO_PORT_A, GPIO_PIN_6) 
INCLUDE_GPIO_REFRENCE(GPIO_PORT_A, GPIO_PIN_7) 

INCLUDE_GPIO_REFRENCE(GPIO_PORT_B, GPIO_PIN_0) 
INCLUDE_GPIO_REFRENCE(GPIO_PORT_B, GPIO_PIN_1) 
INCLUDE_GPIO_REFRENCE(GPIO_PORT_B, GPIO_PIN_2)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_B, GPIO_PIN_3) 
INCLUDE_GPIO_REFRENCE(GPIO_PORT_B, GPIO_PIN_4)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_B, GPIO_PIN_5)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_B, GPIO_PIN_6)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_B, GPIO_PIN_7)

INCLUDE_GPIO_REFRENCE(GPIO_PORT_C, GPIO_PIN_0)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_C, GPIO_PIN_1)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_C, GPIO_PIN_2)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_C, GPIO_PIN_3)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_C, GPIO_PIN_4)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_C, GPIO_PIN_5)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_C, GPIO_PIN_6)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_C, GPIO_PIN_7)

INCLUDE_GPIO_REFRENCE(GPIO_PORT_D, GPIO_PIN_0)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_D, GPIO_PIN_1)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_D, GPIO_PIN_2)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_D, GPIO_PIN_3)
INCLUDE_GPIO_REFRENCE(GPIO_PORT_D, GPIO_PIN_4)

// --------------------------------------------------------------------------------

/**
 * @brief Structure to control the GPIO register of the RP2040
 * @see RP2040 Datasheet (build-date: 2021-03-05 / build-version: 9bf4a25-clean)
 * 
 */
typedef struct {

    /**
     * @brief GPIO status
     * See RP2040 datasheet ch. 2.19.6.1. page 267
     */
    io_ro_32 status;

    /**
     * @brief GPIO control including function select and overrides.
     * See RP2040 datasheet ch. 2.19.6.1. page 268
     */
    io_rw_32 ctrl;

} RP2040_GPIO_IO_STATUS_CTRL_REG;

/**
 * @brief Structure to give access to the status and cotnrol
 * register of available gpio-pins of the RP2040
 * 
 */
typedef struct {

    /**
     * @brief Status and control register of available GPIOS on the RP2040
     * @see RP2040_GPIO_IO_STATUS_CTRL_REG
     * 
     */
    RP2040_GPIO_IO_STATUS_CTRL_REG gpio[RP2040_NUM_GPIO_PINS];

} RP2040_GPIO_IO_REG;

/**
 * @brief PAD control register
 * See RP2040 datasheet ch. 2.19.6.3 page 320+
 * 
 */
typedef struct {

    /**
     * @brief Voltage select. Per bank control
     * See RP2040 datasheet ch. 2.19.6.3 page 321
     */
    io_rw_32 voltage_select;

    /**
     * @brief Pad control register
     * See RP2040 datasheet ch. 2.19.6.3 page 321
     */
    io_rw_32 pads[RP2040_NUM_GPIO_PINS];

} RP2040_GPIO_PAD_REG;

 /**
  * @brief Structure to access the gpio output register to set level and read level.
  * 
  */
typedef struct {

    /**
     * @brief Processor core identifier
     */
    io_ro_32 cpuid;

    /**
     * @brief Input value for GPIO pins, offset 0x004
     */
    io_ro_32 gpio_in;


    /**
     * @brief Input value for QSPI pins, offset 0x008
     */
    io_ro_32 gpio_hi_in;

    /**
     * @brief There is an additional 4 bytes offset.
     * Add this member too respect this space.
     * 
     */
    uint32_t _pad0;

    /**
     * @brief GPIO output value, offset 0x010
     */
    io_rw_32 gio_out;

    /**
     * @brief GPIO output value set, offset 0x014
     */
    io_rw_32 gpio_out_set;

    /**
     * @brief GPIO output value clear, offset 0x018
     */
    io_rw_32 gpio_out_clr;


    /**
     * @brief GPIO output value XOR (toggle)
     */
    io_wo_32 gpio_out_xor;
    
    /**
     * @brief GPIO output enable. Offset: 0x020
     */
    io_rw_32 gpio_out_en;

    /**
     * @brief GPIO output enable set, Offset: 0x024
     */
    io_rw_32 gpio_out_en_set;

    /**
     * @brief GPIO output enable clear, Offset: 0x028
     */
    io_rw_32 gpio_out_en_clr;

} RP2040_GPIO_SIO_REG;

// --------------------------------------------------------------------------------

/**
 * @brief Get the pin number of the given gpio pin-descriptor
 * 
 * @param p_pin_descr valid pin-descriptor with port-id and pin-id set
 * @return pin number in the range from 0 to 28
 */
static u8 gpio_driver_get_pin_number(const GPIO_DRIVER_PIN_DESCRIPTOR* p_pin_descr) {

    /**
     * @brief Realizes a direct access to the gpio-num.
     */
    static u8 gpio_num_array[5][9] = {
        {  0,  1,  2,  3,  4,  5,  6,  7}, // Port A
        {  8,  9, 10, 11, 12, 13, 14, 15}, // Port B
        { 16, 17, 18, 19, 20, 21, 22, 23}, // Port C
        { 24, 25, 26, 27, 28, 29, 30, 31}  // Port D
    };

    u8 pin_num = gpio_num_array[p_pin_descr->port_id][p_pin_descr->pin_id];

    // DEBUG_PASS("=====================================================");
    // DEBUG_TRACE_byte(p_pin_descr->port_id, "gpio_driver_get_pin_number() - GPIO-PORT:"); 
    // DEBUG_TRACE_byte(p_pin_descr->pin_id, "gpio_driver_get_pin_number() - GPIO-PIN:"); 
    // DEBUG_TRACE_byte(pin_num, "gpio_driver_get_pin_number() - GPIO-NUM:");

    return pin_num;
}

/**
 * @brief Returns pointer to access the gpio level register
 * 
 * @return Pointer to the gpio level register
 */
static inline RP2040_GPIO_SIO_REG* gpio_driver_get_sio_reg(void) {
    return ((RP2040_GPIO_SIO_REG*)(RP2040_SIO_REG_BASE_ADDRESS));
}

/**
 * @brief Gets the pointer to the control and status register
 * of the gpio of interest as RP2040_GPIO_IO_REG.
 * 
 * @return pointer to the gpio-control-regsiter of the rp2040 as RP2040_GPIO_IO_REG
 */
static inline RP2040_GPIO_IO_REG* gpio_driver_get_io_reg(void) {
    return ((RP2040_GPIO_IO_REG*)RP2040_IO_REG_BASE_ADDRESS);
}

/**
 * @brief Gets the pointer to the pads register
 * of the gpio of interest as RP2040_GPIO_PAD_REG.
 * 
 * @param p_pin_descr GPIO interface descriptor of the gpio-pin to use
 * @return pointer to the gpio-pads-regsiter of the rp2040 as RP2040_GPIO_PAD_REG
 */
static inline RP2040_GPIO_PAD_REG* gpio_driver_get_pad_reg(void) {
    return ((RP2040_GPIO_PAD_REG*)RP2040_PADS_BASE_ADDRESS);
}

/**
 * @brief Activates the defined fucntion on the given gpio-pin.
 * 
 * @param gpio_num number of the gpio where to set the function
 * @param pin_function function to activate on the gpio-pin
 */
static void gpio_driver_set_pin_function(const u8 gpio_num, u8 pin_function) {

    u32 new_func = 0U;

    switch (pin_function) {

        default: // no break;

        case GPIO_FUNCTION_GPIO:
            // no break;

        case GPIO_FUNCTION_5:  // SIO
            new_func = RP2040_GPIO_FUNC_F5; 
            DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_function() - FUNCTION 5 - GPIO-NUM:"); 
            break;

        case GPIO_FUNCTION_1: // SPI
            new_func = RP2040_GPIO_FUNC_F1; 
            DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_function() - FUNCTION 1 - GPIO-NUM:"); 
            break; 

        case GPIO_FUNCTION_2:  // UART
            new_func = RP2040_GPIO_FUNC_F2; 
            DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_function() - FUNCTION 2 - GPIO-NUM:"); 
            break;

        case GPIO_FUNCTION_3:  // I2C
            new_func = RP2040_GPIO_FUNC_F3; 
            DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_function() - FUNCTION 3 - GPIO-NUM:"); 
            break;

        case GPIO_FUNCTION_4:  // PWM
            new_func = RP2040_GPIO_FUNC_F4; 
            DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_function() - FUNCTION 4 - GPIO-NUM:"); 
            break;

        case GPIO_FUNCTION_6:  // PIO0
            new_func = RP2040_GPIO_FUNC_F6; 
            DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_function() - FUNCTION 6 - GPIO-NUM:"); 
            break;

        case GPIO_FUNCTION_7:  // PIO1
            new_func = RP2040_GPIO_FUNC_F7; 
            DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_function() - FUNCTION 7 - GPIO-NUM:"); 
            break;
    }

    // be sure input and output are enabled.
    cpu_bit_replace(
        &gpio_driver_get_pad_reg()->pads[gpio_num],         // register to modify
        RP2040_GPIO_PAD_OD_BIT | RP2040_GPIO_PAD_IE_BIT,    // mask to replace
        RP2040_GPIO_PAD_IE_BIT                              // bit value to use
    );

    // we want to set all other fields to zero,
    // to let the peripheral control this pin.
    gpio_driver_get_io_reg()->gpio[gpio_num].ctrl = new_func;
}

/**
 * @brief Resets the given gpio-pin
 * 
 * @param gpio_num number of the gpio to be resetted
 */
static void gpio_driver_reset_pin(const u8 gpio_num) {
    gpio_driver_get_sio_reg()->gpio_out_en_clr = (1U << gpio_num);
    gpio_driver_get_sio_reg()->gpio_out_clr = (1U << gpio_num);
}

/**
 * @brief Sets the given pin-level to the given pin
 * 
 * @param gpio_num gpio-number of the pin to change the level
 * @param pin_level level to applay to pin
 */
static void gpio_driver_set_pin_level(const u8 gpio_num, GPIO_DRIVER_LEVEL pin_level) {

    if (pin_level == GPIO_LEVEL_HIGH) {

        DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_level() - HIGH - GPIO-NUM:");
        gpio_driver_get_sio_reg()->gpio_out_set = (1U << gpio_num);

    } else if (pin_level == GPIO_LEVEL_LOW) {

        DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_level() - LOW - GPIO-NUM:");
        gpio_driver_get_sio_reg()->gpio_out_clr = (1U << gpio_num);

    } else {

        DEBUG_TRACE_byte(gpio_num, "gpio_driver_set_pin_level() - HIGH-Z - GPIO-NUM:");
        gpio_driver_get_sio_reg()->gpio_out_clr = (1U << gpio_num);
    }
}

/**
 * @brief Gets the actual physical level of the gpio-pin
 * 
 * @param gpio_num number of the pin to read the level from
 * @return The actual physical level
 */
static GPIO_DRIVER_LEVEL gpio_driver_get_pin_level(const u8 gpio_num) {
    if ( (gpio_driver_get_sio_reg()->gio_out & (1U << gpio_num)) != 0U) {
        return GPIO_LEVEL_HIGH;
    } else {
        return GPIO_LEVEL_LOW;
    }
}

/**
 * @brief Activates the pull-up for a specific pin.
 * 
 * @param gpio_num the pin where the pull-up will be activated
 * @param pull_up 1: activated pull-up / 0: deactivate pull-up
 */
static void gpio_driver_set_pin_pull_up(u8 gpio_num, u8 pull_up) {
    if (pull_up != 0) {
        gpio_driver_get_pad_reg()->pads[gpio_num] |= RP2040_GPIO_PAD_PULL_UP_EN;
    } else {
        gpio_driver_get_pad_reg()->pads[gpio_num] &= ~RP2040_GPIO_PAD_PULL_UP_EN;
    }
}

/**
 * @brief Activates the pull-up for a specific pin.
 * 
 * @param gpio_num the pin where the pull-down will be activated
 * @param pull_down 1: activated pull-down / 0: deactivate pull-down
 */
static void gpio_driver_set_pin_pull_down(u8 gpio_num, u8 pull_down) {
    if (pull_down != 0) {
        gpio_driver_get_pad_reg()->pads[gpio_num] |= RP2040_GPIO_PAD_PULL_DOWN_EN;
    } else {
        gpio_driver_get_pad_reg()->pads[gpio_num] &= ~RP2040_GPIO_PAD_PULL_DOWN_EN;
    }
}

/**
 * @brief Convinience function to set pull-up / pull-down together
 * 
 * @param gpio_num the pin where the change the pull-up / pull-down state
 * @param pull_up 1: activated pull-up / 0: deactivate pull-up
 * @param pull_down 1: activated pull-down / 0: deactivate pull-down
 */
static inline void gpio_driver_set_pin_pull_up_down(u8 gpio_num, u8 pull_up,u8 pull_down) {
    gpio_driver_set_pin_pull_up(gpio_num, pull_up);
    gpio_driver_set_pin_pull_down(gpio_num, pull_down);
}

// --------------------------------------------------------------------------------

/**
 * @see gpio_interface.h#gpio_driver_init
 */
void gpio_driver_init(void) {

    DEBUG_PASS("gpio_driver_init() - START");
    
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_A, GPIO_PIN_0)); // GPIO_00
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_A, GPIO_PIN_1)); // GPIO_01
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_A, GPIO_PIN_2)); // GPIO_02
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_A, GPIO_PIN_3)); // GPIO_03
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_A, GPIO_PIN_4)); // GPIO_04
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_A, GPIO_PIN_5)); // GPIO_05
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_A, GPIO_PIN_6)); // GPIO_06
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_A, GPIO_PIN_7)); // GPIO_07

    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_B, GPIO_PIN_0)); // GPIO_08
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_B, GPIO_PIN_1)); // GPIO_09
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_B, GPIO_PIN_2)); // GPIO_10
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_B, GPIO_PIN_3)); // GPIO_11
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_B, GPIO_PIN_4)); // GPIO_12
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_B, GPIO_PIN_5)); // GPIO_13
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_B, GPIO_PIN_6)); // GPIO_14
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_B, GPIO_PIN_7)); // GPIO_15

    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_C, GPIO_PIN_0)); // GPIO_16
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_C, GPIO_PIN_1)); // GPIO_17
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_C, GPIO_PIN_2)); // GPIO_18
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_C, GPIO_PIN_3)); // GPIO_19
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_C, GPIO_PIN_4)); // GPIO_20
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_C, GPIO_PIN_5)); // GPIO_21
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_C, GPIO_PIN_6)); // GPIO_22
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_C, GPIO_PIN_7)); // GPIO_23

    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_D, GPIO_PIN_0)); // GPIO_24
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_D, GPIO_PIN_1)); // GPIO_25
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_D, GPIO_PIN_2)); // GPIO_26
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_D, GPIO_PIN_3)); // GPIO_27
    gpio_driver_init_pin(GET_GPIO_REFERENCE(GPIO_PORT_D, GPIO_PIN_4)); // GPIO_28

    DEBUG_PASS("gpio_driver_init() - END");
}

/**
 * @see gpio_interface.h#gpio_driver_deinit
 */
void gpio_driver_deinit(void) {

}

/**
 * @see gpio_interface.h#gpio_driver_init_pin
 */
void gpio_driver_init_pin(GPIO_DRIVER_PIN_DESCRIPTOR* p_pin_descr) {

    if (GPIO_DRIVER_PIN_IS_DEACTIVATED(p_pin_descr)) {
        DEBUG_TRACE_byte(gpio_driver_get_pin_number(p_pin_descr), "gpio_driver_init_pin() - ignore pin:");
        return;
    }

    if (GPIO_GET_FUNCTION(p_pin_descr) != GPIO_FUNCTION_GPIO) {
        gpio_driver_set_pin_function(
            gpio_driver_get_pin_number(p_pin_descr),
            GPIO_GET_FUNCTION(p_pin_descr)
        );
        return;
    }

    u8 gpio_num = gpio_driver_get_pin_number(p_pin_descr);

    DEBUG_TRACE_byte(gpio_num,  "gpio_driver_init_pin() - Pin-Number:");
    DEBUG_TRACE_byte(p_pin_descr->pin_cfg,  "gpio_driver_init_pin() - Pin-Cfg:");

    GPIO_DRIVER_DIRECTION direction = GPIO_DRIVER_IS_OUTPUT(p_pin_descr) ?
                                        GPIO_DIRECTION_OUTPUT : GPIO_DIRECTION_INPUT;

    GPIO_DRIVER_LEVEL level = GPIO_DRIVER_IS_IDLE_LOW(p_pin_descr) ? GPIO_LEVEL_LOW :
                                GPIO_DRIVER_IS_IDLE_HIGH(p_pin_descr) ? GPIO_LEVEL_HIGH :
                                    GPIO_LEVEL_HIGH_Z;

    gpio_driver_reset_pin(gpio_num);
    gpio_driver_set_pin_function(gpio_num, GPIO_FUNCTION_GPIO);

    gpio_driver_set_direction(p_pin_descr, direction);
    gpio_driver_set_level(p_pin_descr, level);
}

/**
 * @see gpio_interface.h#gpio_driver_set_direction
 */
void gpio_driver_set_direction(GPIO_DRIVER_PIN_DESCRIPTOR* const p_pin_descr, const GPIO_DRIVER_DIRECTION direction) {

    if (GPIO_DRIVER_PIN_IS_DEACTIVATED(p_pin_descr) != 0U) {
        DEBUG_TRACE_byte(gpio_driver_get_pin_number(p_pin_descr),  "gpio_driver_set_direction() - GPIO IS DEACTIVATED - NUM:");
        return;
    }

    if (GPIO_GET_FUNCTION(p_pin_descr) != GPIO_FUNCTION_GPIO) {
        DEBUG_TRACE_byte(gpio_driver_get_pin_number(p_pin_descr),  "gpio_driver_set_direction() - NON-GPIO FUNCTION - NUM:");
        return;
    }

    u32 gpio_num = gpio_driver_get_pin_number(p_pin_descr);
    
    if (direction == GPIO_DIRECTION_OUTPUT) {
        DEBUG_TRACE_byte(gpio_num,  "gpio_driver_set_direction() - OUTPUT - GPIO-NUM:");
        gpio_driver_get_sio_reg()->gpio_out_en_set = (1U << gpio_num);
        GPIO_DRIVER_SET_OUTPUT(p_pin_descr);

    } else {
        DEBUG_TRACE_byte(gpio_num,  "gpio_driver_set_direction() - INPUT - GPIO-NUM:");
        gpio_driver_get_sio_reg()->gpio_out_en_clr = (1U << gpio_num);
        GPIO_DRIVER_SET_INPUT(p_pin_descr);
    }
}

/**
 * @see gpio_interface.h#gpio_driver_set_level
 */
void gpio_driver_set_level(GPIO_DRIVER_PIN_DESCRIPTOR* p_pin_descr, GPIO_DRIVER_LEVEL level) {

    if (GPIO_DRIVER_PIN_IS_DEACTIVATED(p_pin_descr) != 0U) {
        DEBUG_TRACE_byte(gpio_driver_get_pin_number(p_pin_descr),  "gpio_driver_set_level() - GPIO IS DEACTIVATED - NUM:");
        return;
    }

    if (GPIO_GET_FUNCTION(p_pin_descr) != GPIO_FUNCTION_GPIO) {
        DEBUG_TRACE_byte(gpio_driver_get_pin_number(p_pin_descr),  "gpio_driver_set_level() - NON-GPIO FUNCTION - NUM:");
        return;
    }

    u8 gpio_num = gpio_driver_get_pin_number(p_pin_descr);

    if (GPIO_DRIVER_PIN_IS_INVERTED(p_pin_descr) != 0U) {
        DEBUG_TRACE_byte(gpio_num,  "gpio_driver_set_level() - IS INVERTED - GPIO-NUM:");
        GPIO_DRIVER_INVERT_LEVEL(level);
    }

    if (GPIO_DRIVER_IS_OUTPUT(p_pin_descr) != 0U) {

        DEBUG_TRACE_byte(level,  "gpio_driver_set_level() - OUTPUT - LEVEL:");
        gpio_driver_set_pin_level(gpio_num, level);
        gpio_driver_set_pin_pull_up_down (gpio_num, 0, 0);

    } else {

        DEBUG_TRACE_byte(level,  "gpio_driver_set_level() - INPUT - LEVEL:");
        gpio_driver_set_pin_pull_up_down (
            gpio_num,
            (level == GPIO_LEVEL_HIGH) ? 1U : 0U, // PULL-UP
            (level == GPIO_LEVEL_LOW) ? 1U : 0U   // PULL-DOWN
        );
    }
}

/**
 * @see gpio_interface.h#gpio_driver_toggle_level
 */
void gpio_driver_toggle_level(GPIO_DRIVER_PIN_DESCRIPTOR* p_pin_descr) {

    if (GPIO_DRIVER_PIN_IS_DEACTIVATED(p_pin_descr) != 0U) {
        return;
    }

    if (GPIO_DRIVER_IS_OUTPUT(p_pin_descr) == 0U) {
        DEBUG_TRACE_byte(gpio_driver_get_pin_number(p_pin_descr), "gpio_driver_toggle_level() - This is not an output");
        return;
    }

    u8 gpio_num = gpio_driver_get_pin_number(p_pin_descr);
    GPIO_DRIVER_LEVEL pin_level = gpio_driver_get_pin_level(gpio_num);
    GPIO_DRIVER_INVERT_LEVEL(pin_level);
    gpio_driver_set_level(p_pin_descr, pin_level);
}

/**
 * @see gpio_interface.h#gpio_driver_get_level
 */
GPIO_DRIVER_LEVEL gpio_driver_get_level(GPIO_DRIVER_PIN_DESCRIPTOR* p_pin_descr) {
    
    if (GPIO_DRIVER_PIN_IS_DEACTIVATED(p_pin_descr) != 0U) {
        return GPIO_LEVEL_HIGH_Z;
    }

    GPIO_DRIVER_LEVEL pin_level = gpio_driver_get_pin_level(gpio_driver_get_pin_number(p_pin_descr));

    if (GPIO_DRIVER_PIN_IS_INVERTED(p_pin_descr) != 0U) {
        GPIO_DRIVER_INVERT_LEVEL(pin_level);
    }

    return pin_level;
}

/**
 * @see gpio_interface.h#gpio_driver_print_pin_state
 */
void gpio_driver_print_pin_state(GPIO_DRIVER_PIN_DESCRIPTOR* p_pin_descr) {
    (void) p_pin_descr;
}

/**
 * @see gpio_interface.h#gpio_driver_deactivate
 */
void gpio_driver_deactivate(GPIO_DRIVER_PIN_DESCRIPTOR* p_pin_descr) {
    p_pin_descr->pin_cfg |= GPIO_DEACTIVATE;
}

/**
 * @see gpio_interface.h#gpio_driver_activate
 */
void gpio_driver_activate(GPIO_DRIVER_PIN_DESCRIPTOR* p_pin_descr) {
    p_pin_descr->pin_cfg &= ~GPIO_DEACTIVATE;
    gpio_driver_init_pin(p_pin_descr);
}

// --------------------------------------------------------------------------------
