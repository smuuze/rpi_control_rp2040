#ifndef   _config_H_ /* parse include file only once */
#define   _config_H_

//-------------------------------------------------------------------------

#include "cpu.h"

///-----------------------------------------------------------------------------

#define BOARD_DESCRIPTION_FILE                          "platine/board_UNITTEST.h"
#include "platine/board_UNITTEST.h"

//-------------------------------------------------------------------------

#define I2C_CLK_LIMIT 225000

//-------------------------------------------------------------------------

#define SIGNAL_SLOT_INTERFACE_SIGNAL_SEND_TIMEOUT_MS	0

// --------------------------------------------------------------------------------

/**
 * @brief Used for register of the rp2040 that are read-only
 * 
 */
typedef const volatile uint32_t io_ro_32;

/**
 * @brief Used for registers of the rp2040 that can be read and written.
 * 
 */
typedef volatile uint32_t io_rw_32;

/**
 * @brief Used for registers of the rp2040 that can only be written.
 * 
 */
typedef volatile uint32_t io_wo_32;

// --------------------------------------------------------------------------------

#ifndef __UNUSED__
#define __UNUSED__ __attribute__((unused))
#endif

#ifndef __USED__
#define __USED__ __attribute__((__used__))
#endif

#ifndef __PACKED__
#define __PACKED__ __attribute__((packed))
#endif

// --------------------------------------------------------------------------------

#ifndef __CONCAT
#define __CONCAT(a, b) a ## b
#endif

#ifndef __STRING
#define __STRING(x) #x
#endif

#ifndef __UNSIGNED
#define __UNSIGNED(x)   x##u
#endif

#ifndef __ALWAYS_INLINE
#define __ALWAYS_INLINE inline
#endif

// --------------------------------------------------------------------------------

/**
 * @brief The number of gpio-pins of the RP2040
 * 
 */
#define RP2040_NUM_GPIO_PINS        30

// --------------------------------------------------------------------------------

/**
 * @brief Unittest stub. Only performs *p_register |= bit_mask
 * 
 * @param address reference to the rigister where to set the bits
 * @param bit_mask bits to set in the register
 */
__ALWAYS_INLINE static void cpu_atomic_bit_set(io_rw_32* p_register, u32 bit_mask) {
    *p_register |= bit_mask;
}

// --------------------------------------------------------------------------------

/**
 * @brief Unittest stub. Only performs *p_register &= ~bit_mask;
 * 
 * @param address reference to the rigister where to clear the bits
 * @param bit_mask bits to clear in the register
 */
__ALWAYS_INLINE static void cpu_atomic_bit_clear(io_rw_32* p_register, u32 bit_mask) {
    *p_register &= ~bit_mask;
}

// --------------------------------------------------------------------------------

/**
 * @brief  Unittest stub. Only performs *p_register ^= bit_mask;
 * 
 * @param address reference to the rigister where to clear the bits
 * @param bit_mask bits to xor in the register
 */
__ALWAYS_INLINE static void cpu_atomic_bit_xor(io_rw_32* p_register, u32 bit_mask) {
    *p_register ^= bit_mask;
}

// --------------------------------------------------------------------------------

/**
 * @brief Replaces the bits of mask with the given bit-value.
 * 
 * @param address reference to the rigister where to replace the bits
 * @param bit_mask bits to be replaced
 * @param bit_value values to use for replacement
 */
__ALWAYS_INLINE static void cpu_bit_replace(io_rw_32* p_register, u32 bit_mask, u32 bit_value) {
    cpu_atomic_bit_xor(p_register, (*p_register ^ bit_value) & bit_mask);
}

//-------------------------------------------------------------------------

extern void* ut_get_RP2040_SIO_REG_BASE_ADDRESS(void);
extern void* ut_get_RP2040_IO_REG_BASE_ADDRESS(void);
extern void* ut_get_RP2040_PADS_REG_BASE_ADDRESS(void);

#define RP2040_SIO_REG_BASE_ADDRESS      ut_get_RP2040_SIO_REG_BASE_ADDRESS()
#define RP2040_IO_REG_BASE_ADDRESS       ut_get_RP2040_IO_REG_BASE_ADDRESS()
#define RP2040_PADS_BASE_ADDRESS        ut_get_RP2040_PADS_REG_BASE_ADDRESS()

//-------------------------------------------------------------------------

#include "../src/config_default.h"

#endif /* _config_H_ */
