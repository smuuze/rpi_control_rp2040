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
 * @file    rp2040_reset.c
 * @author  Sebastian Lesse
 * @date    2022 / 08 / 15
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

#include "rp2040_reset.h"

// --------------------------------------------------------------------------------

/**
 * @see rp2040_reset.h#rp2040_reset_subsystem
 */
void rp2040_reset_subsystem(u32 sub_system_bits) {

    RP2040_RESET_REG* resets_hw = ((RP2040_RESET_REG*)RP2040_RESET_REGISTER_BASE);

    cpu_atomic_bit_set(&resets_hw->reset, sub_system_bits);
    cpu_atomic_bit_clear(&resets_hw->reset, sub_system_bits);

    while ((resets_hw->reset & sub_system_bits) != 0u) {
        cpu_watch_forever_loop();
    }
}

// --------------------------------------------------------------------------------
