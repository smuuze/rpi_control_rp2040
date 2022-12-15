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
 * @file    binary_info.c
 * @author  Sebastian Lesse
 * @date    2022 / 08 / 12
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

/**
 * @brief 
 * 
 */
typedef struct __PACKED__ _binary_info_core {
        uint16_t type;
        uint16_t tag;
} BINARY_INFO_CORE_TYPE;

/**
 * @brief 
 * 
 */
typedef struct __PACKED__ _binary_info_id_and_string {
        BINARY_INFO_CORE_TYPE core;
        u32 id;
        const char* value;
} BINARY_INFO_ID_AND_STR_TYPE;

// --------------------------------------------------------------------------------

/**
 * @brief 
 * 
 */
#define BINARY_INFO_TYPE_RAW_DATA                           1
#define BINARY_INFO_TYPE_SIZED_DATA                         2
#define BINARY_INFO_TYPE_BINARY_INFO_LIST_ZERO_TERMINATED   3
#define BINARY_INFO_TYPE_BSON                               4
#define BINARY_INFO_TYPE_ID_AND_INT                         5
#define BINARY_INFO_TYPE_ID_AND_STRING                      6

// --------------------------------------------------------------------------------

/**
 * @brief Available binary info identifications
 * 
 */
#define BINARY_INFO_ID_RP_PROGRAM_NAME                  0x02031c86
#define BINARY_INFO_ID_RP_PROGRAM_VERSION_STRING        0x11a9bc3a
#define BINARY_INFO_ID_RP_PROGRAM_BUILD_DATE_STRING     0x9da22254
#define BINARY_INFO_ID_RP_BINARY_END                    0x68f465de
#define BINARY_INFO_ID_RP_PROGRAM_URL                   0x1856239a
#define BINARY_INFO_ID_RP_PROGRAM_DESCRIPTION           0xb6a07c19
#define BINARY_INFO_ID_RP_PROGRAM_FEATURE               0xa1f4b453
#define BINARY_INFO_ID_RP_PROGRAM_BUILD_ATTRIBUTE       0x4275f0d3
#define BINARY_INFO_ID_RP_SDK_VERSION                   0x5360b3ab
#define BINARY_INFO_ID_RP_PICO_BOARD                    0xb63cffbb
#define BINARY_INFO_ID_RP_BOOT2_NAME                    0x7f8882e1

// --------------------------------------------------------------------------------

/**
 * @brief Creates a tag to be set into a binary info 
 * 
 */
#define _BINARY_INFO_TAG(c1, c2) (((((u32)c2) & 0xFFU) << 8u) | (((u32)c1) & 0xFFU))

// --------------------------------------------------------------------------------

/**
 * @brief Generates a bianry info variable name that contains the given line number
 * 
 */
#define _BINARY_INFO_VAR_NAME(line)         __CONCAT(__bi, line)
#define _BINARY_INFO_PTR_VAR_NAME(line)     __CONCAT(__bi_ptr, line)
#define _BINARY_INFO_CHECK_VAR_NAME(line)   __CONCAT(__bi_check, line)

// --------------------------------------------------------------------------------

/**
 * @brief 
 * 
 */
#define _BINARY_INFO_CHECK_BLOCK(x, line)   (x + _BINARY_INFO_CHECK_VAR_NAME(line))

/**
 * @brief 
 * 
 */
#define _BINARY_INFO_MARK_BLOCK(line)       static const __UNUSED__ u32 _BINARY_INFO_CHECK_VAR_NAME(line) = 0

// --------------------------------------------------------------------------------

/**
 * @brief 
 * 
 */
#define _BINARY_INFO_STR(_tag, _id, _value)                                             \
    static const BINARY_INFO_ID_AND_STR_TYPE _BINARY_INFO_VAR_NAME(__LINE__) = {        \
        .core = {                                                                       \
            .type = _BINARY_INFO_CHECK_BLOCK(BINARY_INFO_TYPE_ID_AND_STRING, __LINE__), \
            .tag = _tag,                                                                \
        },                                                                              \
        .id = _id,                                                                      \
        .value = _value,                                                                \
    }

// --------------------------------------------------------------------------------

#define __bi_decl(ptr_name, ref_block_core, section_prefix) \
    static const __USED__ __attribute__((section(section_prefix __STRING(ptr_name)))) BINARY_INFO_CORE_TYPE* ptr_name = ref_block_core

// --------------------------------------------------------------------------------

#define BINARY_INFO_BLOCK(block_definition)     _BINARY_INFO_MARK_BLOCK(__LINE__);                              \
                                                block_definition;                                               \
                                                __bi_decl(                                                      \
                                                    /*ptr_name*/        _BINARY_INFO_PTR_VAR_NAME(__LINE__),    \
                                                    /*ref_block_core*/  &_BINARY_INFO_VAR_NAME(__LINE__).core,  \
                                                    /*section_prefix*/  ".binary_info.keep."                    \
                                                )

// --------------------------------------------------------------------------------

/**
 * @brief 
 * 
 */
#define BINARY_INFO_PROGRAM_DESCRIPTION(program_description_str)        \
            _BINARY_INFO_STR (                                          \
                    /*TAG*/   _BINARY_INFO_TAG('R', 'P'),               \
                    /*ID*/    BINARY_INFO_ID_RP_PROGRAM_DESCRIPTION,    \
                    /*VALUE*/ program_description_str                   \
            )

// --------------------------------------------------------------------------------

#define _config_PROGRAM_DESCRIPTION "Test-Program"

#ifdef _config_PROGRAM_DESCRIPTION
BINARY_INFO_BLOCK(BINARY_INFO_PROGRAM_DESCRIPTION(_config_PROGRAM_DESCRIPTION));
#endif // _config_PROGRAM_DESCRIPTION

// --------------------------------------------------------------------------------