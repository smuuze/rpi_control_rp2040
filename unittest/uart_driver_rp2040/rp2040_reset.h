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
 * @file    rp2040_reset.h
 * @author  Sebastian Lesse
 * @date    2022 / 08 / 15
 * @brief   Short description of this file
 * 
 */

// --------------------------------------------------------------------------------

#ifndef _H_rp2040_reset_
#define _H_rp2040_reset_

// --------------------------------------------------------------------------------

/**
 * @brief Resets the UART0 sub-system and waits until reset is finisehd.
 * 
 */
void rp2040_reset_uart0(void);

// --------------------------------------------------------------------------------

#endif // _H_rp2040_reset_

// --------------------------------------------------------------------------------
