/**
 ******************************************************************************
 * @file  nand_ecc_arm.S
 * @brief Software ECC ARM assembly version
 *
 * Copyright (C) 2007 Parrot S.A.
 *
 * @author     raphael.lemoine@parrot.fr
 * @date       2007-03-28
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 ******************************************************************************
 */
	.text
	.align	2
	.global	ecc_512_compute_armv4t_step1
	.type	ecc_512_compute_armv4t_step1, %function

ecc_512_compute_armv4t_step1:
	stmfd	sp!, {r4, r5, r6, r7, lr}

	mov		ip, #0x1FC
	mov		r6, #0xFF
	ldr		r7, [r0, ip]
	mov		r4, r1
	mov		r2, #0
	mov		r3, #0
	mov		lr, #0

.loop:
	and		r5, r6, r7
	ldrb	r1, [r4, r5]
	eor		lr, lr, r2
	ands	r5, r1, #0x40
	and		r5, r6, r7, lsr #8
	eorne	r3, r3, ip
	ldrb	r2, [r4, r5]
	add		ip, ip, #1
	eor		lr, lr, r1
	ands	r5, r2, #0x40
	and		r5, r6, r7, lsr #16
	eorne	r3, r3, ip
	ldrb	r1, [r4, r5]
	add		ip, ip, #1
	eor		lr, lr, r2
	ands	r5, r1, #0x40
	and		r5, r6, r7, lsr #24
	eorne	r3, r3, ip
	ldrb	r2, [r4, r5]
	add		ip, ip, #1
	ands	r5, r2, #0x40
	eor		lr, lr, r1
	eorne	r3, r3, ip
	subs	ip, ip, #7
	ldrpl	r7, [r0, ip]
	bpl		.loop
	eor		lr, lr, r2

	add		r0, lr, r3, lsl #8
	ldmfd	sp!, {r4, r5, r6, r7, pc}
