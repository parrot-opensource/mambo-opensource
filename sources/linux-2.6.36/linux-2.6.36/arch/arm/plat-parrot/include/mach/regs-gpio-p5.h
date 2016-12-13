/**
 * @file linux/include/asm-arm/arch-parrot/regs-gpio-p5.h
 * @brief Parrot5 GPIO registers
 *
 * Copyright (C) 2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2006-02-15
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
#ifndef __ARCH_ARM_PARROT_REGS_GPIO_P5_H
#define __ARCH_ARM_PARROT_REGS_GPIO_P5_H

#define _P5_GPIO_P0007DR    0x0000       /* GPIO00...07 data */
#define _P5_GPIO_P0815DR    0x0400       /* GPIO08...15 data */
#define _P5_GPIO_P1623DR    0x0800       /* GPIO16...23 data */
#define _P5_GPIO_P2431DR    0x0C00       /* GPIO24...31 data */
#define _P5_GPIO_P3239DR    0x1000       /* GPIO32...39 data */
#define _P5_GPIO_P4047DR    0x1400       /* GPIO40...47 data */

#define _P5_GPIO_P0007DDR   0x1800       /* GPIO00...07 direction */
#define _P5_GPIO_P0815DDR   0x1804       /* GPIO08...15 direction */
#define _P5_GPIO_P1623DDR   0x1808       /* GPIO16...23 direction */
#define _P5_GPIO_P2431DDR   0x180C       /* GPIO24...31 direction */
#define _P5_GPIO_P3239DDR   0x1810       /* GPIO32...39 direction */
#define _P5_GPIO_P4047DDR   0x1814       /* GPIO40...47 direction */

#define _P5_GPIO_INTC1      0x1818       /* GPIO 1st IRQ configuration */
#define _P5_GPIO_INTC2      0x181C       /* GPIO 2nd IRQ configuration */

/* Registers bitwise definitions */

/* Interrupt configuration */
#define P5_GPIO_INTCX_POS   (0 << 6)     /* Positive edge */
#define P5_GPIO_INTCX_BOTH  (1 << 6)     /* Both edge */
#define P5_GPIO_INTCX_NEG   (2 << 6)     /* Negative edge */
#define P5_GPIO_INTCX_LEV   (3 << 6)     /* Level */
#define P5_GPIO_INTCX_LOW   (1 << 8)     /* Low Level (if bit is not set Low) */
#define P5_GPIO_INTCX_CLR   (1 << 9)     /* Clear */
#define P5_GPIO_INTCX_ENA   (1 << 10)    /* Enable */

#endif /* __ARCH_ARM_PARROT_REGS_GPIO_P5 */
