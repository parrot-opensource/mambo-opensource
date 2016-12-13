/**
 * @file linux/include/asm-arm/arch-parrot/irqs-p6.h
 * @brief Parrot6 interrupt vectors
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     matthieu.castet@parrot.com
 * @date       2008-06-16
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
 *
 */
#ifndef __ASM_ARCH_IRQS_P6_H
#define __ASM_ARCH_IRQS_P6_H

#define IRQ_P6_LCDC             (0)
#define IRQ_P6_CAN            	(1)
#define IRQ_P6_AAI            	(2)
#define IRQ_P6_MOST           	(3)
#define IRQ_P6_SDIO           	(4)
#define IRQ_P6_UART0            (5)
#define IRQ_P6_UART1            (6)
#define IRQ_P6_UART2            (7)
#define IRQ_P6_UART3            (8)
#define IRQ_P6_UARTSIM0         (9)
#define IRQ_P6_NANDMC           (10)
#define IRQ_P6_SPIX             (11)
#define IRQ_P6_I2CS             (12)
#define IRQ_P6_GPIO             (13)
#define IRQ_P6_DMAC             (14)
#define IRQ_P6_TICK             (15)
#define IRQ_P6_TIMER0           (16)
#define IRQ_P6_TIMER1           (17)
#define IRQ_P6_TIMER2        	(18)
#define IRQ_P6_TIMER3           (19)
#define IRQ_P6_H264             (20)
#define IRQ_P6_CAMIF0           (21)
#define IRQ_P6_CAMIF1           (22)
#define IRQ_P6_CV               (23)
#define IRQ_P6_MON           	(24)
#define IRQ_P6_PARINT          	(25)
#define IRQ_P6_COMMRX           (26)
#define IRQ_P6_COMMTX           (27)
#define IRQ_P6_I2CM             (28)
#define IRQ_P6_USB0             (29)
#define IRQ_P6_USB1             (30)
#define IRQ_P6_MPSDX            (31)

/* P6i interrupt vectors */
#define IRQ_P6I_RTC             (31)

#define FIQ_START		0

#endif /* __ASM_ARCH_IRQS_P6_H */
