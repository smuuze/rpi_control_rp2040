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
 * @file    unittest_gpio_driver_rp2040.c
 * @author  Sebastian Lesse
 * @date    2022 / 09 / 02
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

// --------------------------------------------------------------------------------

#include "common/common_tools_bit_vector.h"

// --------------------------------------------------------------------------------

#define TEST_CASE_ID_INITIALIZATION             0
#define TEST_CASE_ID_SET_OUTPUT_LOW             1
#define TEST_CASE_ID_SET_INPUT_HIGH             2

// --------------------------------------------------------------------------------

#define UT_SIGNAL_TIMEOUT_MS    100

// --------------------------------------------------------------------------------

typedef struct {
    io_ro_32 status;
    io_rw_32 ctrl;
} UT_GPIO_IO_STATUS_CTRL_REG;

typedef struct {
    UT_GPIO_IO_STATUS_CTRL_REG gpio[RP2040_NUM_GPIO_PINS];
} UT_GPIO_IO_REG;

typedef struct {
    io_rw_32 voltage_select;
    io_rw_32 pads[RP2040_NUM_GPIO_PINS];
} UT_GPIO_PAD_REG;

typedef struct {
    io_ro_32 cpuid;
    io_ro_32 gpio_in;
    io_ro_32 gpio_hi_in;
    uint32_t _pad0;
    io_rw_32 gio_out;
    io_rw_32 gpio_out_set;
    io_rw_32 gpio_out_clr;
    io_wo_32 gpio_out_xor;
    io_rw_32 gpio_out_en;
    io_rw_32 gpio_out_en_set;
    io_rw_32 gpio_out_en_clr;
} UT_GPIO_SIO_REG;

// --------------------------------------------------------------------------------

static u8 counter_GET_SIO_REG = 0;
static u8 counter_GET_IO_REG = 0;
static u8 counter_GET_PADS_REG = 0;
static u8 counter_RESET_GPIO = 0;

/**
 * @brief Needed to compare the values written by the gpio-driver
 */
static u32 copy_of_gpio_out_en_clr = 0; // 1: DIRECTION INPUT
static u32 copy_of_gpio_out_en_set = 0; // 1: DIRECTION OUTPUT
static u32 copy_of_gpio_out_clr = 0;    // 1: LEVEL LOW
static u32 copy_of_gpio_out_set = 0;    // 1: LEVEL HIGH

static void unittest_reset_counter(void) {

    counter_GET_SIO_REG = 0;
    counter_GET_IO_REG = 0;
    counter_GET_PADS_REG = 0; 
    
    copy_of_gpio_out_en_clr = 0;
    copy_of_gpio_out_en_set = 0;
    copy_of_gpio_out_clr = 0;
    copy_of_gpio_out_set = 0;

    counter_RESET_GPIO = 0;
}

// --------------------------------------------------------------------------------

// stubs

UT_GPIO_SIO_REG ut_sio_reg;
UT_GPIO_IO_REG  ut_io_reg;
UT_GPIO_PAD_REG ut_pad_reg;

void* ut_get_RP2040_SIO_REG_BASE_ADDRESS(void) {

    DEBUG_PASS("ut_get_RP2040_SIO_REG_BASE_ADDRESS()");

    if (UT_GET_TEST_CASE_ID() == TEST_CASE_ID_INITIALIZATION) {

        /**
         * @brief on init the gpio driver performs several operations like
         * gpio_driver_get_sio_reg()->gpio_out_en_clr = (1U << gpio_num).
         * In the UT-testbench we do not know which value was written.
         * So we perform a copy of the actual value.
         * 
         */
        copy_of_gpio_out_en_clr |= ut_sio_reg.gpio_out_en_clr;
        copy_of_gpio_out_en_set |= ut_sio_reg.gpio_out_en_set;
        copy_of_gpio_out_clr    |= ut_sio_reg.gpio_out_clr;
        copy_of_gpio_out_set    |= ut_sio_reg.gpio_out_set;
    }

    counter_GET_SIO_REG += 1;
    return (void*)&ut_sio_reg;
}
 
void* ut_get_RP2040_IO_REG_BASE_ADDRESS(void) {
    counter_GET_IO_REG += 1;
    return (void*)&ut_io_reg;
}

void* ut_get_RP2040_PADS_REG_BASE_ADDRESS(void) {
    counter_GET_PADS_REG += 1;
    return (void*)&ut_pad_reg;
}

u16 time_mgmnt_gettime_u16(void) {
    return 0;
}

u8 time_mgmnt_istimeup_raw_u16(u16 time_reference, u16 time_interval) {
    return 0;
}

void rp2040_reset_gpio(void) {
    counter_RESET_GPIO += 1;
}

// --------------------------------------------------------------------------------
// PORT A
//---------     Name,           Port,           Bit,            Pin-Cfg      FUNCTION

    BUILD_GPIO ( GPIO_00,       GPIO_PORT_A,    GPIO_PIN_0,     GPIO_FUNCTION_2                         ) //    UART0 - TX
    BUILD_GPIO ( GPIO_01,       GPIO_PORT_A,    GPIO_PIN_1,     GPIO_FUNCTION_2                         ) //    UART0 - RX
    BUILD_GPIO ( GPIO_02,       GPIO_PORT_A,    GPIO_PIN_2,     GPIO_INPUT | GPIO_IDLE_HIGH             ) //
    BUILD_GPIO ( GPIO_03,       GPIO_PORT_A,    GPIO_PIN_3,     GPIO_OUTPUT | GPIO_IDLE_HIGH            ) //
    BUILD_GPIO ( GPIO_04,       GPIO_PORT_A,    GPIO_PIN_4,     GPIO_OUTPUT | GPIO_IDLE_LOW             ) // 
    BUILD_GPIO ( GPIO_05,       GPIO_PORT_A,    GPIO_PIN_5,     GPIO_OUTPUT | GPIO_IDLE_HIGH_Z          ) //
    BUILD_GPIO ( GPIO_06,       GPIO_PORT_A,    GPIO_PIN_6,     GPIO_INPUT | GPIO_IDLE_HIGH_Z           ) //
    BUILD_GPIO ( GPIO_07,       GPIO_PORT_A,    GPIO_PIN_7,     GPIO_INPUT | GPIO_IDLE_LOW              ) //

// --------------------------------------------------------------------------------
// PORT B
//---------     Name,           Port,           Bit,            Pin-Cfg      FUNCTION

    BUILD_GPIO ( GPIO_08,       GPIO_PORT_B,    GPIO_PIN_0,     GPIO_INPUT | GPIO_IDLE_HIGH ) //
    BUILD_GPIO ( GPIO_09,       GPIO_PORT_B,    GPIO_PIN_1,     GPIO_INPUT | GPIO_IDLE_HIGH ) //
    BUILD_GPIO ( GPIO_10,       GPIO_PORT_B,    GPIO_PIN_2,     GPIO_INPUT | GPIO_IDLE_HIGH ) // 
    BUILD_GPIO ( GPIO_11,       GPIO_PORT_B,    GPIO_PIN_3,     GPIO_INPUT | GPIO_IDLE_HIGH ) //
    BUILD_GPIO ( GPIO_12,       GPIO_PORT_B,    GPIO_PIN_4,     GPIO_INPUT | GPIO_IDLE_HIGH ) //
    BUILD_GPIO ( GPIO_13,       GPIO_PORT_B,    GPIO_PIN_5,     GPIO_INPUT | GPIO_IDLE_HIGH ) //
    BUILD_GPIO ( GPIO_14,       GPIO_PORT_B,    GPIO_PIN_6,     GPIO_INPUT | GPIO_IDLE_HIGH ) //
    BUILD_GPIO ( GPIO_15,       GPIO_PORT_B,    GPIO_PIN_7,     GPIO_INPUT | GPIO_IDLE_HIGH ) //

// --------------------------------------------------------------------------------
// PORT C
//---------     Name,           Port,           Bit,            Pin-Cfg      FUNCTION

    BUILD_GPIO ( GPIO_16,       GPIO_PORT_C,    GPIO_PIN_0,     GPIO_FUNCTION_GPIO ) // GPIO_IDLE_HIGH | GPIO_INPUT
    BUILD_GPIO ( GPIO_17,       GPIO_PORT_C,    GPIO_PIN_1,     GPIO_FUNCTION_1 ) //
    BUILD_GPIO ( GPIO_18,       GPIO_PORT_C,    GPIO_PIN_2,     GPIO_FUNCTION_2 ) //
    BUILD_GPIO ( GPIO_19,       GPIO_PORT_C,    GPIO_PIN_3,     GPIO_FUNCTION_3 ) //
    BUILD_GPIO ( GPIO_20,       GPIO_PORT_C,    GPIO_PIN_4,     GPIO_FUNCTION_4 ) //
    BUILD_GPIO ( GPIO_21,       GPIO_PORT_C,    GPIO_PIN_5,     GPIO_FUNCTION_5 ) //
    BUILD_GPIO ( GPIO_22,       GPIO_PORT_C,    GPIO_PIN_6,     GPIO_FUNCTION_6 ) // 
    BUILD_GPIO ( GPIO_23,       GPIO_PORT_C,    GPIO_PIN_7,     GPIO_FUNCTION_7 ) // 

// --------------------------------------------------------------------------------
// PORT D
//---------     Name,           Port,           Bit,            Pin-Cfg      FUNCTION

    BUILD_GPIO ( GPIO_24,       GPIO_PORT_D,    GPIO_PIN_0,     GPIO_INPUT | GPIO_IDLE_HIGH  ) //
    BUILD_GPIO ( GPIO_25,       GPIO_PORT_D,    GPIO_PIN_1,     GPIO_OUTPUT | GPIO_IDLE_HIGH ) //    ON_BOARD LED
    BUILD_GPIO ( GPIO_26,       GPIO_PORT_D,    GPIO_PIN_2,     GPIO_INPUT | GPIO_IDLE_HIGH  ) //
    BUILD_GPIO ( GPIO_27,       GPIO_PORT_D,    GPIO_PIN_3,     GPIO_OUTPUT | GPIO_IDLE_HIGH  ) //
    BUILD_GPIO ( GPIO_28,       GPIO_PORT_D,    GPIO_PIN_4,     GPIO_OUTPUT | GPIO_IDLE_LOW  ) //

// --------------------------------------------------------------------------------

// slots callbacksstatic IR_COMMON_COMMAND sony_ir_command;

// --------------------------------------------------------------------------------

// Signals / Slots

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

        UT_CHECK_IS_EQUAL(counter_GET_SIO_REG, 66);
        UT_CHECK_IS_EQUAL(counter_GET_IO_REG, 29);
        UT_CHECK_IS_EQUAL(counter_GET_PADS_REG, 69);

        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 1);

        // Check Pin Function
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[0].ctrl, 2);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[1].ctrl, 2);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[2].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[3].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[4].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[5].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[5].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[6].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[7].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[8].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[9].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[10].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[11].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[12].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[13].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[14].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[15].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[16].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[17].ctrl, 1);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[18].ctrl, 2);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[19].ctrl, 3);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[20].ctrl, 4);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[21].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[22].ctrl, 6);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[23].ctrl, 7);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[24].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[25].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[26].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[27].ctrl, 5);
        UT_CHECK_IS_EQUAL(ut_io_reg.gpio[28].ctrl, 5);

        /**
         * @brief Check pin directions set have been set/clear
         * check them all together - because after init all values have been set
         * 
         */
        u32 exp_sio_gpio_out_en_clr = 520224764; //BUILD_BIT_VECTOR_U32(1,1,1,1,0,0,0,0, 0,0,0,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,0,0);
        u32 exp_sio_gpio_out_en_set = 436207672; //BUILD_BIT_VECTOR_U32(0,0,0,1,1,0,1,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,1,1,1,1,0,0);

        UT_CHECK_IS_EQUAL(copy_of_gpio_out_en_clr, exp_sio_gpio_out_en_clr);
        UT_CHECK_IS_EQUAL(copy_of_gpio_out_en_set, exp_sio_gpio_out_en_set);

        /**
         * @brief Check pin levels set have been set/clear
         * check them all together - because after init all values have been set
         * 
         */
        u32 exp_sio_gpio_out_clr = 520224764; //BUILD_BIT_VECTOR_U32(1,1,1,1,0,0,0,0, 0,0,0,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,0,0);
        u32 exp_sio_gpio_out_set = 167772168; //BUILD_BIT_VECTOR_U32(0,0,0,0,1,0,1,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,1,0,0,0);

        UT_CHECK_IS_EQUAL(copy_of_gpio_out_clr, exp_sio_gpio_out_clr);
        UT_CHECK_IS_EQUAL(copy_of_gpio_out_set, exp_sio_gpio_out_set);

        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[0],  64); // GPIO_FUNCTION_2
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[1],  64); // GPIO_FUNCTION_2
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[2],  72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[3],  64); // GPIO_OUTPUT | GPIO_IDLE_HIGH
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[4],  64); // GPIO_OUTPUT | GPIO_IDLE_LOW
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[5],  64); // GPIO_OUTPUT | GPIO_IDLE_HIGH_Z
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[6],  64); // GPIO_INPUT | GPIO_IDLE_HIGH_Z
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[7],  68); // GPIO_INPUT | GPIO_IDLE_LOW
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[8],  72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[9],  72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[10], 72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[11], 72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[12], 72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[13], 72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[14], 72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[15], 72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[16], 72); // GPIO_FUNCTION_GPIO
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[17], 64); // GPIO_FUNCTION_1
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[18], 64); // GPIO_FUNCTION_2
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[19], 64); // GPIO_FUNCTION_3
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[20], 64); // GPIO_FUNCTION_4
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[21], 64); // GPIO_FUNCTION_5
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[22], 64); // GPIO_FUNCTION_6
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[23], 64); // GPIO_FUNCTION_7
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[24], 72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[25], 64); // GPIO_OUTPUT | GPIO_IDLE_HIGH
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[26], 72); // GPIO_INPUT | GPIO_IDLE_HIGH (PULL-UP ENABLED)
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[27], 64); // GPIO_OUTPUT | GPIO_IDLE_HIGH
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[28], 64); // GPIO_OUTPUT | GPIO_IDLE_LOW

    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Sets all gpios as outputs with low-level, except those who have a non-GPIO function
 * 
 */
static void TEST_CASE_set_as_output_low(void) {

    UT_START_TEST_CASE("Set Output Low")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_SET_OUTPUT_LOW);
        unittest_reset_counter();

        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_00_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[0], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_01_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[1], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_02_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 2));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 2));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[2], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;
        
        GPIO_03_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 3));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 3));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[3], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_04_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 4));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 4));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[4], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_05_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 5));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 5));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[5], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_06_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 6));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 6));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[6], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_07_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 7));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 7));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[7], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_08_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 8));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 8));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[8], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_09_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 9));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 9));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[9], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_10_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 10));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 10));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[10], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_11_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 11));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 11));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[11], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_12_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 12));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 12));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[12], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_13_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 13));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 13));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[13], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_14_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 14));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 14));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[14], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_15_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 15));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 15));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[15], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_16_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 16));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 16));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[16], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_17_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[17], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_18_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[18], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_19_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[19], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_20_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[20], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_21_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[21], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_22_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[22], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_23_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[23], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_24_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 24));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 24));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[24], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_25_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 25));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 25));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[25], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_26_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 26));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 26));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[26], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_27_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 27));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 27));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[27], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

        GPIO_28_drive_low();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 28));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_clr, (1 << 28));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[28], 64);
        UT_CHECK_IS_EQUAL(counter_RESET_GPIO, 0);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        ut_sio_reg.gpio_out_set = 0;
        ut_sio_reg.gpio_out_clr = 0;

    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Sets all gpios as inputs with PULL-UP enabled, except those who have a non-GPIO function
 * 
 */
static void TEST_CASE_set_as_input_high(void) {

    UT_START_TEST_CASE("Set Input High")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_SET_INPUT_HIGH);
        unittest_reset_counter();

        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        for (u8 i = 0; i < 29; i++) {
            UT_CHECK_IS_EQUAL(ut_pad_reg.pads[i], (1 << 6));
        }

        GPIO_00_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[0], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_01_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[1], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_02_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 2));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[2], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        
        GPIO_03_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 3));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[3], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_04_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 4));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[4], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_05_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 5));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[5], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_06_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 6));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[6], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_07_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 7));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[7], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_08_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 8));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[8], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_09_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 9));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[9], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_10_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 10));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[10], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_11_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 11));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[11], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_12_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 12));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[12], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_13_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 13));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[13], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_14_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 14));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[14], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_15_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 15));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[15], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_16_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 16));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[16], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_17_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[17], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_18_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[18], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_19_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[19], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_20_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[20], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_21_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[21], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_22_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[22], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_23_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[23], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_24_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 24));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[24], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_25_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 25));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[25], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_26_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 26));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[26], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_27_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 27));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[27], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_28_pull_up();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, (1 << 28));
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[28], 72);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Sets all gpios as outputs with high-level, except those who have a non-GPIO function
 * 
 */
static void TEST_CASE_set_as_output_high(void) {

    UT_START_TEST_CASE("Set Output High")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_SET_OUTPUT_LOW);
        unittest_reset_counter();

        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_00_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[0], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_01_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[1], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_02_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 2));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[2], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;
        
        GPIO_03_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 3));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[3], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_04_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 4));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[4], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_05_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 5));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[5], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_06_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 6));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[6], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_07_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 7));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[7], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_08_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 8));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[8], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_09_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 9));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[9], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_10_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 10));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[10], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_11_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 11));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[11], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_12_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 12));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[12], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_13_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 13));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[13], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_14_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 14));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[14], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_15_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 15));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[15], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_16_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 16));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[16], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_17_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[17], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_18_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[18], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_19_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[19], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_20_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[20], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_21_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[21], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_22_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[22], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_23_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, 0);
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[23], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_24_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 24));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[24], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_25_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 25));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[25], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_26_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 26));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[26], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_27_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 27));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[27], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

        GPIO_28_drive_high();
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_clr, 0);
        UT_CHECK_IS_EQUAL(ut_sio_reg.gpio_out_en_set, (1 << 28));
        UT_CHECK_IS_EQUAL(ut_pad_reg.pads[28], 64);
        ut_sio_reg.gpio_out_en_set = 0;
        ut_sio_reg.gpio_out_en_clr = 0;

    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

int main(void) {

    UT_GPIO_SIO_REG ut_sio_reg;
    UT_GPIO_IO_REG  ut_io_reg;
    UT_GPIO_PAD_REG ut_pad_reg;

    memset(&ut_sio_reg, 0x00, sizeof(UT_GPIO_SIO_REG));
    memset(&ut_io_reg, 0x00, sizeof(UT_GPIO_IO_REG));
    memset(&ut_pad_reg, 0x00, sizeof(UT_GPIO_PAD_REG));

    UT_START_TESTBENCH("Welcome the the UNITTEST for RP2040 GPIO-driver 1.0")
    {
        TEST_CASE_initialization();
        TEST_CASE_set_as_output_low();
        TEST_CASE_set_as_input_high();
        TEST_CASE_set_as_output_high();
    }
    UT_END_TESTBENCH()

    return UT_TEST_RESULT();
}

// --------------------------------------------------------------------------------
