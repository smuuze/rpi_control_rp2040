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
 * @file    rp2040_clock_frequencies.h
 * @author  Sebastian Lesse
 * @date    2022 / 08 / 20
 * @brief   Short description of this file
 * 
 */

// --------------------------------------------------------------------------------

#ifndef _H_rp2040_clock_frequencies_
#define _H_rp2040_clock_frequencies_

// --------------------------------------------------------------------------------

#include "cpu.h"

// --------------------------------------------------------------------------------

/**
 * @brief Get the frequency of the peripheral clock in numbers of MHZ.
 * E.g. if the curent clk-peri is set to 125 MHz this fucntions returns 125.
 * Only valid after clock_driver_init() was called.
 * 
 * @return the number of MHz of the current value of clk-peri
 */
u32 clock_driver_peripheral_clk_frequency(void);

// --------------------------------------------------------------------------------

#endif // _H_rp2040_clock_frequencies_

// --------------------------------------------------------------------------------
