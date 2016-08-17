/**
 * @file linux/include/asm-arm/arch-parrot/regs-gpio-p5p.h
 * @brief Parrot5+ GPIO registers
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
#ifndef __ARCH_ARM_PARROT_REGS_GPIO_P5P_H
#define __ARCH_ARM_PARROT_REGS_GPIO_P5P_H

/* data registers */
#define _P5P_GPIO_P0007DR    0x0000       /* GPIO00...07 data */
#define _P5P_GPIO_P0815DR    0x0400       /* GPIO08...15 data */
#define _P5P_GPIO_P1623DR    0x0800       /* GPIO16...23 data */
#define _P5P_GPIO_P2431DR    0x0C00       /* GPIO24...31 data */
#define _P5P_GPIO_P3239DR    0x1000       /* GPIO32...39 data */
#define _P5P_GPIO_P4047DR    0x1400       /* GPIO40...47 data */
#define _P5P_GPIO_P4855DR    0x1800       /* GPIO48...55 data */
#define _P5P_GPIO_P5663DR    0x1C00       /* GPIO56...63 data */
#define _P5P_GPIO_P6471DR    0x2000       /* GPIO64...71 data */
#define _P5P_GPIO_P7279DR    0x2400       /* GPIO72...79 data */

/* data direction registers */
#define _P5P_GPIO_P0007DDR   0x2800       /* GPIO00...07 direction */
#define _P5P_GPIO_P0815DDR   0x2804       /* GPIO08...15 direction */
#define _P5P_GPIO_P1623DDR   0x2808       /* GPIO16...23 direction */
#define _P5P_GPIO_P2431DDR   0x280C       /* GPIO24...31 direction */
#define _P5P_GPIO_P3239DDR   0x2810       /* GPIO32...39 direction */
#define _P5P_GPIO_P4047DDR   0x2814       /* GPIO40...47 direction */
#define _P5P_GPIO_P4855DDR   0x2818       /* GPIO48...55 direction */
#define _P5P_GPIO_P5663DDR   0x281C       /* GPIO56...63 direction */
#define _P5P_GPIO_P6471DDR   0x2820       /* GPIO64...71 direction */
#define _P5P_GPIO_P7279DDR   0x2824       /* GPIO72...79 direction */

#define _P5P_GPIO_STATUS     0x2828       /* GPIO status */

#define _P5P_GPIO_INTC1      0x2C00       /* GPIO interrupt 1 control */
#define _P5P_GPIO_INTC2      0x2C04       /* GPIO interrupt 2 control */
#define _P5P_GPIO_INTC3      0x2C08       /* GPIO interrupt 3 control */
#define _P5P_GPIO_INTC4      0x2C0C       /* GPIO interrupt 4 control */
#define _P5P_GPIO_INTC5      0x2C10       /* GPIO interrupt 5 control */
#define _P5P_GPIO_INTC6      0x2C14       /* GPIO interrupt 6 control */
#define _P5P_GPIO_INTC7      0x2C18       /* GPIO interrupt 7 control */
#define _P5P_GPIO_INTC8      0x2C1C       /* GPIO interrupt 8 control */
#define _P5P_GPIO_INTC9      0x2C20       /* GPIO interrupt 9 control */
#define _P5P_GPIO_INTC10     0x2C24       /* GPIO interrupt 10 control */
#define _P5P_GPIO_INTC11     0x2C28       /* GPIO interrupt 11 control */
#define _P5P_GPIO_INTC12     0x2C2C       /* GPIO interrupt 12 control */
#define _P5P_GPIO_INTC13     0x2C30       /* GPIO interrupt 13 control */
#define _P5P_GPIO_INTC14     0x2C34       /* GPIO interrupt 14 control */
#define _P5P_GPIO_INTC15     0x2C38       /* GPIO interrupt 15 control */
#define _P5P_GPIO_INTC16     0x2C3C       /* GPIO interrupt 16 control */

/* Registers bitwise definitions */

/* Interrupt configuration */
#define _P5P_GPIO_INTCX_ENA  (1 << 7)     /* Enable */
#define _P5P_GPIO_INTCX_POS  (0 << 8)     /* Positive edge */
#define _P5P_GPIO_INTCX_BOTH (1 << 8)     /* Both edge */
#define _P5P_GPIO_INTCX_NEG  (2 << 8)     /* Negative edge */
#define _P5P_GPIO_INTCX_LEV  (3 << 8)     /* Level */
#define _P5P_GPIO_INTCX_LOW  (1 << 10)    /* Low Level (if bit is set) */
#define _P5P_GPIO_INTCX_CLR  (1 << 11)    /* Clear */

#endif /* __ARCH_ARM_PARROT_REGS_GPIO_P5P */
