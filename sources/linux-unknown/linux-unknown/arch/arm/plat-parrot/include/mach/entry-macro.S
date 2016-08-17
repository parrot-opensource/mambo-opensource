/**
 * @file include/asm-arm/arch-parrot/entry-macro.S
 *
 * @author     ivan.djelic@parrot.com
 * @date       2007-06-11
 *
 * Low-level IRQ helper macros for Parrot platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <mach/map.h>

	.macro	disable_fiq
	.endm

	.macro  get_irqnr_preamble, base, tmp
		mov     \base, #PARROT_VA_VIC      @ VA VIC base = 0xfc000000
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm
        
	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp

		ldr     \irqstat, [\base]           @ VIC status
		rsb     \irqnr, \irqstat, #0        @ -status
		ands    \irqstat, \irqstat, \irqnr  @ status & -status
		clz     \irqnr, \irqstat
		rsb     \irqnr, \irqnr, #31

	.endm
