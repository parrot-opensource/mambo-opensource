/**
 * @file linux/include/asm-arm/arch-parrot/irqs-p5.h
 * @brief Parrot5 interrupt vectors
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2004-11-16
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
#ifndef __ASM_ARCH_IRQS_P5_H
#define __ASM_ARCH_IRQS_P5_H

#define IRQ_P5_IRQ_EBC          (0)
#define IRQ_P5_FIQ_EBC          (1)
#define IRQ_P5_CAN            	(2)
#define IRQ_P5_AAI            	(3)
#define IRQ_P5_MOST           	(4)
#define IRQ_P5_MPSD0           	(5)
#define IRQ_P5_MPSD1            (6)
#define IRQ_P5_UART0            (7)
#define IRQ_P5_UART1            (8)
#define IRQ_P5_UART2            (9)
#define IRQ_P5_UARTSIM0         (10)
#define IRQ_P5_UARTSIM1         (11)
#define IRQ_P5_UARTSIM2         (12)
#define IRQ_P5_SPI              (13)
#define IRQ_P5_I2CS             (14)
#define IRQ_P5_GPIO0            (15)
#define IRQ_P5_GPIO1            (16)
#define IRQ_P5_TICK             (17)
#define IRQ_P5_TIMER0           (18)
#define IRQ_P5_TIMER1           (19)
#define IRQ_P5_TIMER2        	(20)
#define IRQ_P5_TIMER3           (21)
#define IRQ_P5_CAMIF            (22)
#define IRQ_P5_ME              	(23)
#define IRQ_P5_CV               (24)
#define IRQ_P5_MON           	(25)
#define IRQ_P5_RTCC             (26)
#define IRQ_P5_COMMRX           (27)
#define IRQ_P5_COMMTX           (28)
#define IRQ_P5_I2CM             (29)
#define IRQ_P5_FREE1            (30)
#define IRQ_P5_FREE2            (31)

/* Parrot5+ specific irqs */
#define IRQ_P5P_DCT             (1)
#define IRQ_P5P_DMAC            (16)
#define IRQ_P5P_USB0            (30)
#define IRQ_P5P_USB1            (31)

#endif /* __ASM_ARCH_IRQS_P5_H */
