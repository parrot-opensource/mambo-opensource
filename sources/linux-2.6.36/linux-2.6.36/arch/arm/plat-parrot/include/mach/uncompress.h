/**
 * @file linux/include/asm-arm/arch-parrot/uncompress.h
 *
 * Copyright (C) 2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2007-06-11
 *
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
 */

#include <mach/regs-uart.h>
#define UART_BASE	(0xd0000000|0x00010000*(CONFIG_DEBUG_PARROT_UART+7))
#define UART_TX		(*(volatile unsigned char *)(UART_BASE+_UART_TRX))
#define UART_STAT	(*(volatile unsigned char *)(UART_BASE+_UART_STATUS))

/*
 * This does not append a newline
 */
static inline void putc(int c)
{
#ifdef CONFIG_DEBUG_PARROT_UART_DEBUG_UNCOMPRESS
	while (UART_STAT & UART_STATUS_TXFILLED)
		barrier();

	UART_TX = c;
#endif
}

static inline void flush(void)
{
#ifdef CONFIG_DEBUG_PARROT_UART_DEBUG_UNCOMPRESS
	while ((UART_STAT & UART_STATUS_TXEMPTY) == 0)
		barrier();
#endif
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
