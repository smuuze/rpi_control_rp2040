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
 * @file    boot2.c
 * @author  Sebastian Lesse
 * @date    2022 / 08 / 13
 * @brief   Short description of this file
 * 
 */

// --------------------------------------------------------------------------------

#include "cpu.h"

// --------------------------------------------------------------------------------

/**
 * @brief This is the stage 2 boot-loader.
 * It was read from the disassembly of the blinky-example.
 * 
 */
const u32 __boot2_start__[] __SECTION(".boot2") = { // TODO: description
    0x4b32b500, // blmi	10cad408 <__flash_binary_end+0xcab3a8>
    0x60582021, // subsvs	r2, r8, r1, lsr #32
    0x21026898, // ; <UNDEFINED> instruction: 0x21026898
    0x60984388, // addsvs	r4, r8, r8, lsl #7
    0x611860d8, // ldrsbvs	r6, [r8, -r8]
    0x4b2e6158, // blmi	10b9857c <__flash_binary_end+0xb9651c>
    0x60992100, // addsvs	r2, r9, r0, lsl #2
    0x61592102, // cmpvs	r9, r2, lsl #2
    0x22f02101, // rscscs	r2, r0, #1073741824	; 0x40000000
    0x492b5099, // stmdbmi	fp!, {r0, r3, r4, r7, ip, lr}
    0x21016019, // tstcs	r1, r9, lsl r0
    0x20356099, // mlascs	r5, r9, r0, r6
    0xf844f000, // ; <UNDEFINED> instruction: 0xf844f000
    0x42902202, // addsmi	r2, r0, #536870912	; 0x20000000
    0x2106d014, // tstcs	r6, r4, lsl r0
    0xf0006619, // ; <UNDEFINED> instruction: 0xf0006619
    0x6e19f834, // mrcvs	8, 0, APSR_nzcv, cr9, cr4, {1}
    0x66192101, // ldrvs	r2, [r9], -r1, lsl #2
    0x66182000, // ldrvs	r2, [r8], -r0
    0xf000661a, // ; <UNDEFINED> instruction: 0xf000661a
    0x6e19f82c, // cdpvs	8, 1, cr15, cr9, cr12, {1}
    0x6e196e19, // mrcvs	14, 0, r6, cr9, cr9, {0}
    0xf0002005, // ; <UNDEFINED> instruction: 0xf0002005
    0x2101f82f, // tstcs	r1, pc, lsr #16	; <UNPREDICTABLE>
    0xd1f94208, // mvnsle	r4, r8, lsl #4
    0x60992100, // addsvs	r2, r9, r0, lsl #2
    0x6019491b, // andsvs	r4, r9, fp, lsl r9
    0x60592100, // subsvs	r2, r9, r0, lsl #2
    0x481b491a, // ldmdami	fp, {r1, r3, r4, r8, fp, lr}
    0x21016001, // tstcs	r1, r1
    0x21eb6099, // ; <UNDEFINED> instruction: 0x21eb6099
    0x21a06619, // lslcs	r6, r9, r6
    0xf0006619, // ; <UNDEFINED> instruction: 0xf0006619
    0x2100f812, // tstcs	r0, r2, lsl r8	; <UNPREDICTABLE>
    0x49166099, // ldmdbmi	r6, {r0, r3, r4, r7, sp, lr}
    0x60014814, // andvs	r4, r1, r4, lsl r8
    0x60992101, // addsvs	r2, r9, r1, lsl #2
    0x2800bc01, // stmdacs	r0, {r0, sl, fp, ip, sp, pc}
    0x4700d000, // strmi	sp, [r0, -r0]
    0x49134812, // ldmdbmi	r3, {r1, r4, fp, lr}
    0xc8036008, // stmdagt	r3, {r3, sp, lr}
    0x8808f380, // stmdahi	r8, {r7, r8, r9, ip, sp, lr, pc}
    0xb5034708, // strlt	r4, [r3, #-1800]	; 0xfffff8f8
    0x20046a99, // mulcs	r4, r9, sl
    0xd0fb4201, // rscsle	r4, fp, r1, lsl #4
    0x42012001, // andmi	r2, r1, #1
    0xbd03d1f8, // stfltd	f5, [r3, #-992]	; 0xfffffc20
    0x6618b502, // ldrvs	fp, [r8], -r2, lsl #10
    0xf7ff6618, // ; <UNDEFINED> instruction: 0xf7ff6618
    0x6e18fff2, // mrcvs	15, 0, APSR_nzcv, cr8, cr2, {7}
    0xbd026e18, // stclt	14, cr6, [r2, #-96]	; 0xffffffa0
    0x40020000, // andmi	r0, r2, r0
    0x18000000, // stmdane	r0, {}	; <UNPREDICTABLE>
    0x00070000, // andeq	r0, r7, r0
    0x005f0300, // subseq	r0, pc, r0, lsl #6
    0x00002221, // andeq	r2, r0, r1, lsr #4
    0x180000f4, // stmdane	r0, {r2, r4, r5, r6, r7}
    0xa0002022, // andge	r2, r0, r2, lsr #32
    0x10000100, // andne	r0, r0, r0, lsl #2
    0xe000ed08,
    0x00000000,
    0x00000000,
    0x00000000,
    0x7a4eb274 //	bvc	113acad4 <__flash_binary_end+0x13aaa74>
};

// --------------------------------------------------------------------------------
