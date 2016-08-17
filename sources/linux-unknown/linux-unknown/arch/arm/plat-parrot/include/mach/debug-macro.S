/**
 * @file linux/include/asm-arm/arch-parrot/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 2009 Parrot SA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
#include <mach/regs-uart.h>
		.macro	addruart, rx, tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx,      #0xd0000000
		movne	\rx,      #0xfd000000	@ virtual base
		orr	\rx, \rx, #(0x00010000*(CONFIG_DEBUG_PARROT_UART+7))

		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx, #_UART_TRX]
		.endm

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #_UART_STATUS]
		tst	\rd, #UART_STATUS_TXFILLED
		bne	1001b
		.endm

		.macro	busyuart,rd,rx
1001:		ldr	\rd, [\rx, #_UART_STATUS]
		tst	\rd, #UART_STATUS_TXEMPTY
		beq	1001b

		.endm
