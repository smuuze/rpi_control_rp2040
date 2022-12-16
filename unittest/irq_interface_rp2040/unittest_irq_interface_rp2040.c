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
 * @file    unittest_irq_interface_rp2040.c
 * @author  Sebastian Lesse
 * @date    2022 / 10 / 29
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
#include "driver/irq/irq_interface.h"

// --------------------------------------------------------------------------------

#define TEST_CASE_ID_INITIALIZATION                     0
#define TEST_CASE_ID_ADD_HANDLER_AND_EXECUTE_01         1
#define TEST_CASE_ID_ADD_HANDLER_AND_EXECUTE_02         2
#define TEST_CASE_ID_RAISE_EMPTY_IRQ                    3
#define TEST_CASE_ID_ACTIVATE_UNKNOW_IRQ                4

// --------------------------------------------------------------------------------

#define UT_SIGNAL_TIMEOUT_MS                            100

// --------------------------------------------------------------------------------

typedef struct {
    io_rw_32 icpr;
} RP2040_IRQ_CLEAR_PENDING_REG;

typedef struct {
    io_rw_32 iser;
} RP2040_IRQ_SET_ENABLE_REG;

typedef struct {
    io_rw_32 icer;
} RP2040_IRQ_CLEAR_ENABLE_REG;

// --------------------------------------------------------------------------------

static RP2040_IRQ_CLEAR_PENDING_REG ut_irq_clear_pending_reg;
static RP2040_IRQ_CLEAR_ENABLE_REG ut_irq_clear_enable_reg;
static RP2040_IRQ_SET_ENABLE_REG ut_irq_set_enable_reg;

// --------------------------------------------------------------------------------

static u8 counter_GET_UART0_REG = 0;
static u8 counter_IRQ_HANDLER_01_CALLBACK = 0;
static u8 counter_IRQ_HANDLER_02_CALLBACK = 0;
static u8 counter_IRQ_HANDLER_03_CALLBACK = 0;
static u8 counter_GET_ICPR_REG = 0;
static u8 counter_GET_ISER_REG = 0;
static u8 counter_GET_ICER_REG = 0;

static void unittest_reset_counter(void) {

    counter_GET_UART0_REG = 0;
    counter_IRQ_HANDLER_01_CALLBACK = 0;
    counter_IRQ_HANDLER_02_CALLBACK = 0;
    counter_IRQ_HANDLER_03_CALLBACK = 0;
    counter_GET_ICPR_REG = 0;
    counter_GET_ISER_REG = 0;
    counter_GET_ICER_REG = 0;

    ut_irq_clear_pending_reg.icpr = 0;
    ut_irq_clear_enable_reg.icer = 0;
    ut_irq_set_enable_reg.iser = 0;
}

// --------------------------------------------------------------------------------

// stubs

u16 time_mgmnt_gettime_u16(void) {
    return 0;
}

u8 time_mgmnt_istimeup_raw_u16(u16 time_reference, u16 time_interval) {
    return 0;
}

// --------------------------------------------------------------------------------

// Signals / Slots

void* ut_get_RP2040_IRQ_GET_CLEAR_PENDING_REG(void) {
    counter_GET_ICPR_REG += 1;
    return &ut_irq_clear_pending_reg;
}

void* ut_get_RP2040_IRQ_GET_SET_ENABLE_REG(void) {
    counter_GET_ISER_REG += 1;
    return &ut_irq_set_enable_reg;
}

void* ut_get_RP2040_IRQ_GET_CLEAR_ENABLE_REG(void) {
    counter_GET_ICER_REG += 1;
    return &ut_irq_clear_enable_reg;
}

// --------------------------------------------------------------------------------

/**
 * @brief IRQ Handler of the uart0 instance
 * implemented by the uart-driver
 * 
 */
extern void IRQ_20_Handler(void);

/**
 * @brief IRQ Handler of the uart1 instance
 * implemented by the uart-driver
 * 
 */
extern void IRQ_21_Handler(void);

// --------------------------------------------------------------------------------

void UT_IRQ_HANDLER_01_CALLBACK(void) {
    DEBUG_PASS("UT - UT_IRQ_HANDLER_01_CALLBACK");
    counter_IRQ_HANDLER_01_CALLBACK += 1;
}

IRQ_BUILD_HANDLER(UT_IRQ_HANDLER_01, &UT_IRQ_HANDLER_01_CALLBACK, 20, 0)

// --------------------------------------------------------------------------------

void UT_IRQ_HANDLER_02_CALLBACK(void) {
    DEBUG_PASS("UT - UT_IRQ_HANDLER_02_CALLBACK");
    counter_IRQ_HANDLER_02_CALLBACK += 1;
}

IRQ_BUILD_HANDLER(UT_IRQ_HANDLER_02, &UT_IRQ_HANDLER_02_CALLBACK, 20, 0)

// --------------------------------------------------------------------------------

void UT_IRQ_HANDLER_03_CALLBACK(void) {
    DEBUG_PASS("UT - UT_IRQ_HANDLER_03_CALLBACK");
    counter_IRQ_HANDLER_03_CALLBACK += 1;
}

IRQ_BUILD_HANDLER(UT_IRQ_HANDLER_03, &UT_IRQ_HANDLER_03_CALLBACK, 20, 0)

// --------------------------------------------------------------------------------

/**
 * @brief Initialize all GPIOS
 * Check if the values taht have been written to the cpu-regsiters match the expected values.
 */
static void TEST_CASE_initialization(void) {

    UT_START_TEST_CASE("Initialization")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_INITIALIZATION);
        unittest_reset_counter();

        initialization();

        UT_CHECK_IS_EQUAL(counter_GET_UART0_REG, 0);
    }
    UT_END_TEST_CASE()
}

/**
 * @brief Adds a single IRQ-handler to the irq-interface for irq-number 20
 * - Before activating the handler, the irq is raised.
 * - Then the irq-handler is activated and the irq is raised once again
 * - then the irq is deactivated (the handler rameins activated) and the irq is raised for another time
 */
static void TEST_CASE_add_handler_and_execute_01(void) {

    UT_START_TEST_CASE("Add handler and execute 01")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_ADD_HANDLER_AND_EXECUTE_01);
        unittest_reset_counter();

        // enable irq-20 and add handler
        IRQ_INTERFACE_RET_VAL ret_val = irq_set_enabled(20, IRQ_ENABLED);

        UT_CHECK_IS_EQUAL(ret_val, IRQ_INTERFACE_OK);
        UT_CHECK_IS_EQUAL(ut_irq_clear_pending_reg.icpr, (1 << 20));
        UT_CHECK_IS_EQUAL(ut_irq_clear_enable_reg.icer, 0);
        UT_CHECK_IS_EQUAL(ut_irq_set_enable_reg.iser, (1 << 20));
        UT_CHECK_IS_EQUAL(counter_GET_ICPR_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_ISER_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_ICER_REG, 0);

        ret_val = UT_IRQ_HANDLER_01_init();
        UT_CHECK_IS_EQUAL(ret_val, IRQ_INTERFACE_OK);
 
        // cannot add handler a second time
        ret_val = UT_IRQ_HANDLER_01_init();
        UT_CHECK_IS_EQUAL(ret_val, IRQ_INTERFACE_OCCUPIED);

        // adding a handler does not cause calling it
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_01_CALLBACK, 0);
        
        // reset all counters and registers
        unittest_reset_counter();

        // Raise IRQ 20 - the handler is deactivated so it must not be called
        IRQ_20_Handler();

        // activate the handler and the the irq once again
        UT_IRQ_HANDLER_01_set_enabled(IRQ_ENABLED);
        UT_CHECK_IS_EQUAL(counter_GET_ICPR_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_ISER_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_ICER_REG, 0);
        UT_CHECK_IS_EQUAL(ut_irq_clear_pending_reg.icpr, (1 << 20));
        UT_CHECK_IS_EQUAL(ut_irq_clear_enable_reg.icer, 0);
        UT_CHECK_IS_EQUAL(ut_irq_set_enable_reg.iser, (1 << 20));
        
        // now call the handler
        IRQ_20_Handler();
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_01_CALLBACK, 1);
        
        // reset all counters and registers
        unittest_reset_counter();

        // disable IRQ 20
        ret_val = irq_set_enabled(20, IRQ_DISABLED);

        UT_CHECK_IS_EQUAL(ret_val, IRQ_INTERFACE_OK);
        UT_CHECK_IS_EQUAL(ut_irq_clear_pending_reg.icpr, 0);
        UT_CHECK_IS_EQUAL(ut_irq_clear_enable_reg.icer, (1 << 20));
        UT_CHECK_IS_EQUAL(ut_irq_set_enable_reg.iser, 0);
        UT_CHECK_IS_EQUAL(counter_GET_ICPR_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_ISER_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_ICER_REG, 1);
    }
    UT_END_TEST_CASE()
}

/**
 * @brief Adds two more handler to the irq-number 20 and executes them
 * - all handler are activated before raising the irq
 * - then the second handler is disabled and the irq is raised once again
 */
static void TEST_CASE_add_handler_and_execute_02(void) {

    UT_START_TEST_CASE("Add handler and execute 02")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_ADD_HANDLER_AND_EXECUTE_02);
        unittest_reset_counter();

        // enable irq-20 and add handler
        IRQ_INTERFACE_RET_VAL ret_val = irq_set_enabled(20, IRQ_ENABLED);

        UT_CHECK_IS_EQUAL(ret_val, IRQ_INTERFACE_OK);
        UT_CHECK_IS_EQUAL(ut_irq_clear_pending_reg.icpr, (1 << 20));
        UT_CHECK_IS_EQUAL(ut_irq_clear_enable_reg.icer, 0);
        UT_CHECK_IS_EQUAL(ut_irq_set_enable_reg.iser, (1 << 20));
        UT_CHECK_IS_EQUAL(counter_GET_ICPR_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_ISER_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_ICER_REG, 0);

        UT_CHECK_IS_EQUAL(UT_IRQ_HANDLER_02_init(), IRQ_INTERFACE_OK);
        UT_CHECK_IS_EQUAL(UT_IRQ_HANDLER_03_init(), IRQ_INTERFACE_OK);

        // adding a handler does not cause calling it
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_01_CALLBACK, 0);
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_02_CALLBACK, 0);
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_03_CALLBACK, 0);

        // activate the handler and raise the irq
        UT_IRQ_HANDLER_01_set_enabled(IRQ_ENABLED);
        UT_IRQ_HANDLER_02_set_enabled(IRQ_ENABLED);
        UT_IRQ_HANDLER_03_set_enabled(IRQ_ENABLED);
        IRQ_20_Handler();

        // now the handler have been called
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_01_CALLBACK, 1);
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_02_CALLBACK, 1);
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_03_CALLBACK, 1);

        // disable second handler
        UT_IRQ_HANDLER_02_set_enabled(IRQ_DISABLED);

        // raise irq once again
        IRQ_20_Handler();

        // the second handler was not called
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_01_CALLBACK, 2);
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_02_CALLBACK, 1);
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_03_CALLBACK, 2);
    }
    UT_END_TEST_CASE()
}

/**
 * @brief Rises an irq that has no handler.
 */
static void TEST_CASE_raise_empty_irq(void) {

    UT_START_TEST_CASE("Raise empty irq")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_RAISE_EMPTY_IRQ);
        unittest_reset_counter();

        // first we need to enable the irq
        IRQ_INTERFACE_RET_VAL ret_val = irq_set_enabled(21, IRQ_ENABLED);

        UT_CHECK_IS_EQUAL(ret_val, IRQ_INTERFACE_OK);
        UT_CHECK_IS_EQUAL(ut_irq_clear_pending_reg.icpr, (1 << 21));
        UT_CHECK_IS_EQUAL(ut_irq_clear_enable_reg.icer, 0);
        UT_CHECK_IS_EQUAL(ut_irq_set_enable_reg.iser, (1 << 21));
        UT_CHECK_IS_EQUAL(counter_GET_ICPR_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_ISER_REG, 1);
        UT_CHECK_IS_EQUAL(counter_GET_ICER_REG, 0);

        // this irq has no handler
        IRQ_21_Handler();

        // avaible handler are not executed
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_01_CALLBACK, 0);
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_02_CALLBACK, 0);
        UT_CHECK_IS_EQUAL(counter_IRQ_HANDLER_03_CALLBACK, 0);
        
        // set registers back to zero
        unittest_reset_counter();

        // disable the IRQ
        ret_val = irq_set_enabled(21, IRQ_DISABLED);

        UT_CHECK_IS_EQUAL(ret_val, IRQ_INTERFACE_OK);
        UT_CHECK_IS_EQUAL(ut_irq_clear_pending_reg.icpr, 0);
        UT_CHECK_IS_EQUAL(ut_irq_clear_enable_reg.icer, (1 << 21));
        UT_CHECK_IS_EQUAL(ut_irq_set_enable_reg.iser, 0);
        UT_CHECK_IS_EQUAL(counter_GET_ICPR_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_ISER_REG, 0);
        UT_CHECK_IS_EQUAL(counter_GET_ICER_REG, 1);
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

/**
 * @brief Calls several unknown irqs
 * There is no modification on the current configuration made.
 */
static void TEST_CASE_activate_unknown_irq(void) {

    UT_START_TEST_CASE("Raise empty irq")
    {
        UT_SET_TEST_CASE_ID(TEST_CASE_ID_ACTIVATE_UNKNOW_IRQ);
        unittest_reset_counter();

        for (u8 i = RP2040_NUM_IRQ; i < 255; ++i) {

            IRQ_INTERFACE_RET_VAL ret_val = irq_set_enabled(i, IRQ_ENABLED);

            UT_CHECK_IS_EQUAL(ret_val, IRQ_INTERFACE_UNKNOWN);
            UT_CHECK_IS_EQUAL(ut_irq_clear_pending_reg.icpr, 0);
            UT_CHECK_IS_EQUAL(ut_irq_clear_enable_reg.icer, 0);
            UT_CHECK_IS_EQUAL(ut_irq_set_enable_reg.iser, 0);
            UT_CHECK_IS_EQUAL(counter_GET_ICPR_REG, 0);
            UT_CHECK_IS_EQUAL(counter_GET_ISER_REG, 0);
            UT_CHECK_IS_EQUAL(counter_GET_ICER_REG, 0);
        }
    }
    UT_END_TEST_CASE()
}

// --------------------------------------------------------------------------------

int main(void) {

    ut_irq_clear_pending_reg.icpr = 0;
    ut_irq_clear_enable_reg.icer = 0;
    ut_irq_set_enable_reg.iser = 0;

    UT_START_TESTBENCH("Welcome the the UNITTEST for RP2040 IRQ-interface 1.0")
    {
        TEST_CASE_initialization();
        TEST_CASE_add_handler_and_execute_01();
        TEST_CASE_add_handler_and_execute_02();
        TEST_CASE_raise_empty_irq();
        TEST_CASE_activate_unknown_irq();
    }
    UT_END_TESTBENCH()

    return UT_TEST_RESULT();
}

// --------------------------------------------------------------------------------
