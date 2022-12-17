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
 * @file    crt0.c
 * @author  Sebastian Lesse
 * @date    2022 / 08 / 07
 * @brief   c runtime functions (e.g. _start / _exit / _sbrk) for the RP2040
 * 
 */

// --------------------------------------------------------------------------------

#include "config.h"

// --------------------------------------------------------------------------------

#include "cpu.h"

// --------------------------------------------------------------------------------

/**
 * @brief Defines a macro that aliases the function prototype
 * to Default_Handler if the function is not defined.
 * 
 */
#define DEFAULT_HANDLER __attribute__((weak, alias("Default_Handler")))

/**
 * @brief Defines a macro that aliases the function prototype
 * to Fault_Handler if the function is not defined.
 * 
 */
#define FAULT_HANDLER __attribute__((weak, alias("Fault_Handler")))

// --------------------------------------------------------------------------------

#ifndef __SECTION_RESET
#define __SECTION_RESET __SECTION(".reset"))
#endif
#ifndef __SECTION_VECTORS
#define __SECTION_VECTORS __SECTION(".vectors")
#endif

#ifndef __SECTION_BINARY_DATA_HEADER
#define __SECTION_BINARY_DATA_HEADER __SECTION(".binary_info_header")
#endif

#ifndef __SECTION_DATA_CPY_TABLE
#define __SECTION_DATA_CPY_TABLE __SECTION(".data_cpy_table")
#endif

// --------------------------------------------------------------------------------

#define BINARY_INFO_MARKER_START 0x7188ebf2
#define BINARY_INFO_MARKER_END 0xe71aa390

// --------------------------------------------------------------------------------

/**
 * @brief declaration of main, it is called at the end of the _start
 * 
 */
extern int main(void);

// --------------------------------------------------------------------------------

/**
 * @brief Reset Handler to be called to initialize the RP2040
 * 
 */
void Reset_Handler(void);

/**
 * @brief Default handler to be called if no other
 * handler is available for an irq.
 * 
 */
void Default_Handler(void);

/**
 * @brief Fault Handler to be called in case of error.
 * Will not return.
 * 
 */
void Fault_Handler(void);

// --------------------------------------------------------------------------------

/**
 * @brief Prototype of the _exit function.
 * This function does not return.
 * 
 * @param status status of the exit
 */
void _exit(int status);

/**
 * @brief Prototype of the _sbrk function
 * 
 */
void* _sbrk(int incr);

// --------------------------------------------------------------------------------

/**
 * @brief Defines a type for the ISR's in the vector table
 * 
 */
typedef void (*RP2040_VECTOR_TABLE_ELEMENT)(void);

/**
 * @brief Defines a entry for the vector table.
 * The first entry will be the stack-pointer. 
 * The second entry is the reset-handler.
 * Then the additonal isr handler follow.
 * So, we define a union to it this typedefinition for both.
 * 
 */
typedef union {

    /**
     * @brief isr handler entry
     * 
     */
    RP2040_VECTOR_TABLE_ELEMENT isr;

    /**
     * @brief Stack-pointer entry
     * 
     */
    void* stack_top;

} RP2040_VECTOR_TABLE;

typedef void* VECTOR_TABLE_TYPE;

// --------------------------------------------------------------------------------

/**
 * @brief Prototypes of the system exception handlers.
 * If these handlers are not defined Fault_Handler() is used instead.
 * 
 */

FAULT_HANDLER void NMI_Handler(void);
FAULT_HANDLER void HardFault_Handler(void);
FAULT_HANDLER void Invalid_Handler(void);
FAULT_HANDLER void SVC_Handler(void);
FAULT_HANDLER void PendSV_Handler(void);
FAULT_HANDLER void SysTick_Handler(void);

// --------------------------------------------------------------------------------

/**
 * @brief  Prototypes of the peripheral handlers.
 * If these handlers are not defined Default_Handler() is used instead.
 * @see Datasheet ch 2.3.2 Interrupts on page 60 for the IRQ table
 */

DEFAULT_HANDLER void IRQ_00_Handler(void);
DEFAULT_HANDLER void IRQ_01_Handler(void);
DEFAULT_HANDLER void IRQ_02_Handler(void);
DEFAULT_HANDLER void IRQ_03_Handler(void);
DEFAULT_HANDLER void IRQ_04_Handler(void);
DEFAULT_HANDLER void IRQ_05_Handler(void);
DEFAULT_HANDLER void IRQ_06_Handler(void);
DEFAULT_HANDLER void IRQ_07_Handler(void);
DEFAULT_HANDLER void IRQ_08_Handler(void);
DEFAULT_HANDLER void IRQ_09_Handler(void);

DEFAULT_HANDLER void IRQ_10_Handler(void);
DEFAULT_HANDLER void IRQ_11_Handler(void);
DEFAULT_HANDLER void IRQ_12_Handler(void);
DEFAULT_HANDLER void IRQ_13_Handler(void);
DEFAULT_HANDLER void IRQ_14_Handler(void);
DEFAULT_HANDLER void IRQ_15_Handler(void);
DEFAULT_HANDLER void IRQ_16_Handler(void);
DEFAULT_HANDLER void IRQ_17_Handler(void);
DEFAULT_HANDLER void IRQ_18_Handler(void);
DEFAULT_HANDLER void IRQ_19_Handler(void);

DEFAULT_HANDLER void IRQ_20_Handler(void); // UART0-IRQ
DEFAULT_HANDLER void IRQ_21_Handler(void); // UART1-IRQ
DEFAULT_HANDLER void IRQ_22_Handler(void);
DEFAULT_HANDLER void IRQ_23_Handler(void);
DEFAULT_HANDLER void IRQ_24_Handler(void);
DEFAULT_HANDLER void IRQ_25_Handler(void);
DEFAULT_HANDLER void IRQ_26_Handler(void);
DEFAULT_HANDLER void IRQ_27_Handler(void);
DEFAULT_HANDLER void IRQ_28_Handler(void);
DEFAULT_HANDLER void IRQ_29_Handler(void);

DEFAULT_HANDLER void IRQ_30_Handler(void);
DEFAULT_HANDLER void IRQ_31_Handler(void);

// --------------------------------------------------------------------------------

/**
 * Sections:
 * 
 * |------------------------|
 * | .verctors              |
 * |------------------------|
 * | .binary_info_header    |
 * |------------------------|
 * | .reset                 |
 * |------------------------|
 * | .stack                 |
 * |------------------------|
 * | .heap                  |
 * 
 */

// --------------------------------------------------------------------------------

/**
 * @brief top position of the stack-pointer
 * comes from linker-script
 * 
 */
extern unsigned int __StackTop;

/**
 * @brief Start of the bss-section.
 * comes from linker-script
 * 
 */
extern unsigned int __bss_start__;

/**
 * @brief End of the bss-section.
 * Comes from linker-script
 * 
 */
extern unsigned int __bss_end__;

/**
 * @brief Limit of the stack-memory.
 * Comes from linker-script
 * 
 */
extern unsigned int __StackLimit;

/**
 * @brief Start-Address of the binary info
 * 
 */
extern unsigned int __binary_info_start;

/**
 * @brief End address of the binary info
 * 
 */
extern unsigned int __binary_info_end;

/**
 * @brief 
 * 
 */
extern unsigned int __data_start__; // TODO: description

/**
 * @brief 
 * 
 */
extern unsigned int __data_end__; // TODO: description

/**
 * @brief 
 * 
 */
extern unsigned int __etext; // TODO: description

/**
 * @brief 
 * 
 */
extern unsigned int __scratch_x_source__; // TODO: description

/**
 * @brief 
 * 
 */
extern unsigned int __scratch_x_start__; // TODO: description

/**
 * @brief 
 * 
 */
extern unsigned int __scratch_x_end__; // TODO: description

/**
 * @brief 
 * 
 */
extern unsigned int __scratch_y_source__; // TODO: description

/**
 * @brief 
 * 
 */
extern unsigned int __scratch_y_start__; // TODO: description

/**
 * @brief 
 * 
 */
extern unsigned int __scratch_y_end__; // TODO: description

// --------------------------------------------------------------------------------

/**
 * @brief The following vector-table will go into the .verctors section.
 * The name .vectors comes from the linker-script (*.ld)
 * 
 */
const RP2040_VECTOR_TABLE __VECTOR_TABLE[] __SECTION_VECTORS = {

    // First entry is the stack-pointer
    // _StackTop comes from linker-script
    {.stack_top = &__StackTop},

    // second entry is the reset-handler
    {.isr = Reset_Handler},

    // system exception handlers
    {.isr = NMI_Handler},
    {.isr = HardFault_Handler},
    {.isr = Invalid_Handler},
    {.isr = Invalid_Handler},
    {.isr = Invalid_Handler},
    {.isr = Invalid_Handler},
    {.isr = Invalid_Handler},
    {.isr = Invalid_Handler},
    {.isr = Invalid_Handler},
    {.isr = SVC_Handler},
    {.isr = Invalid_Handler},
    {.isr = Invalid_Handler},
    {.isr = PendSV_Handler},
    {.isr = SysTick_Handler},

    // // Other IRQ Handler
    {.isr = IRQ_00_Handler},
    {.isr = IRQ_01_Handler},
    {.isr = IRQ_02_Handler},
    {.isr = IRQ_03_Handler},
    {.isr = IRQ_04_Handler},
    {.isr = IRQ_05_Handler},
    {.isr = IRQ_06_Handler},
    {.isr = IRQ_07_Handler},
    {.isr = IRQ_08_Handler},
    {.isr = IRQ_09_Handler},

    {.isr = IRQ_10_Handler},
    {.isr = IRQ_11_Handler},
    {.isr = IRQ_12_Handler},
    {.isr = IRQ_13_Handler},
    {.isr = IRQ_14_Handler},
    {.isr = IRQ_15_Handler},
    {.isr = IRQ_16_Handler},
    {.isr = IRQ_17_Handler},
    {.isr = IRQ_18_Handler},
    {.isr = IRQ_19_Handler},

    {.isr = IRQ_20_Handler},
    {.isr = IRQ_21_Handler},
    {.isr = IRQ_22_Handler},
    {.isr = IRQ_23_Handler},
    {.isr = IRQ_24_Handler},
    {.isr = IRQ_25_Handler},
    {.isr = IRQ_26_Handler},
    {.isr = IRQ_27_Handler},
    {.isr = IRQ_28_Handler},
    {.isr = IRQ_29_Handler},

    {.isr = IRQ_30_Handler},
    {.isr = IRQ_31_Handler}
};

// --------------------------------------------------------------------------------

/**
 * @brief 
 * 
 */
const void* data_cpy_table[] __SECTION_DATA_CPY_TABLE = { // TODO: description
    &__etext,
    &__data_start__,
    &__data_end__,
    &__scratch_x_source__,
    &__scratch_x_start__,
    &__scratch_x_end__,
    &__scratch_y_source__,
    &__scratch_y_start__,
    &__scratch_y_end__,
    0
};

// --------------------------------------------------------------------------------

/**
 * @brief 
 * 
 */
 const void* binary_info_header[] __SECTION_BINARY_DATA_HEADER = { // TODO: description
     (void*)BINARY_INFO_MARKER_START,  // Start marker
     &__binary_info_start,      // Start Source-Addr
     &__binary_info_end,        // Start Destination-Addr
     data_cpy_table,            // End Destination-Addr
     (void*)BINARY_INFO_MARKER_END     // End marker
};

// --------------------------------------------------------------------------------

/**
 * @brief Entry point is always calling the reset-handler
 * 
 */
__SECTION(".reset._entry_point") void _entry_point(void) {
    Reset_Handler();
}

// --------------------------------------------------------------------------------

/**
 * @brief The Reset_Handler
 * 
 */
__SECTION(".reset.Reset_Handler") void Reset_Handler(void) {

    // copy init values from flash (ROM) to .data section
    unsigned int* p_data_start = (unsigned int*) data_cpy_table[1];
    unsigned int* p_data_end = (unsigned int*) data_cpy_table[2];
    unsigned int* p_dest = (unsigned int*) data_cpy_table[0];

    while (p_data_start != p_data_end) {
        
        *p_data_start = *p_dest;

        ++p_data_start;
        ++p_dest;
    }

    // initialie .bss section - set all data to zero
    unsigned int* p_bss_start = &__bss_start__; 
    unsigned int* p_bss_end   = &__bss_end__;

    while(p_bss_start != p_bss_end) {
        *p_bss_start = 0;
        ++p_bss_start;
    }

    main();
}

// --------------------------------------------------------------------------------

__SECTION(".reset.Default_Handler") void Default_Handler(void) {
    // do nothing
    cpu_breakpoint();
}

// --------------------------------------------------------------------------------

__SECTION(".reset.Fault_Handler") void Fault_Handler(void) {
    cpu_breakpoint();
    _exit(-1);
}

// --------------------------------------------------------------------------------

void __attribute__((noreturn)) _exit(__UNUSED__ int status) {

    #ifdef _config_CPU_EXIT_ENTER_PROTO
    _config_CPU_EXIT_ENTER_PROTO
    #endif // _config_CPU_EXIT_ENTER_PROTO

    #ifdef _config_CPU_EXIT_LOOP_PROTO
    _config_CPU_EXIT_LOOP_PROTO
    #endif // _config_CPU_EXIT_LOOP_PROTO

    #ifdef _config_CPU_EXIT_ENTER_FUNC
    _config_CPU_EXIT_ENTER_FUNC
    #endif

    while (1) {
        #ifdef _config_CPU_EXIT_LOOP_FUNC
        _config_CPU_EXIT_LOOP_FUNC
        #endif
    }
}

// --------------------------------------------------------------------------------

void* _sbrk(int incr) {
    (void) incr;
    return NULL;
}

// --------------------------------------------------------------------------------