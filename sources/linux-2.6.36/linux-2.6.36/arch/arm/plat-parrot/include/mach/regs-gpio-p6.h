/**
 * @file linux/include/asm-arm/arch-parrot/regs-gpio-p5p.h
 * @brief Parrot5+ GPIO registers
 *
 * Copyright (C) 2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @author     florent.bayendrian@parrot.com
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
#ifndef __ARCH_ARM_PARROT_REGS_GPIO_P6_H
#define __ARCH_ARM_PARROT_REGS_GPIO_P6_H

/* data registers */
#define _P6_GPIO_P0007DR    0x0000       /* GPIO00...07 data */
#define _P6_GPIO_P0815DR    0x0400       /* GPIO08...15 data */
#define _P6_GPIO_P1623DR    0x0800       /* GPIO16...23 data */
#define _P6_GPIO_P2431DR    0x0C00       /* GPIO24...31 data */
#define _P6_GPIO_P3239DR    0x1000       /* GPIO32...39 data */
#define _P6_GPIO_P4047DR    0x1400       /* GPIO40...47 data */
#define _P6_GPIO_P4855DR    0x1800       /* GPIO48...55 data */
#define _P6_GPIO_P5663DR    0x1C00       /* GPIO56...63 data */
#define _P6_GPIO_P6471DR    0x2000       /* GPIO64...71 data */
#define _P6_GPIO_P7279DR    0x2400       /* GPIO72...79 data */

/* data direction registers */
#define _P6_GPIO_P0007DDR   0x6000       /* GPIO00...07 direction */
#define _P6_GPIO_P0815DDR   0x6004       /* GPIO08...15 direction */
#define _P6_GPIO_P1623DDR   0x6008       /* GPIO16...23 direction */
#define _P6_GPIO_P2431DDR   0x600C       /* GPIO24...31 direction */
#define _P6_GPIO_P3239DDR   0x6010       /* GPIO32...39 direction */
#define _P6_GPIO_P4047DDR   0x6014       /* GPIO40...47 direction */
#define _P6_GPIO_P4855DDR   0x6018       /* GPIO48...55 direction */
#define _P6_GPIO_P5663DDR   0x601C       /* GPIO56...63 direction */
#define _P6_GPIO_P6471DDR   0x6020       /* GPIO64...71 direction */
#define _P6_GPIO_P7279DDR   0x6024       /* GPIO72...79 direction */

#define _P6_GPIO_STATUS     0x6200       /* GPIO status */
#define _P6_GPIO_DEBOUNCE	0x6204
#define _P6_GPIO_ROTATOR	0x6208
#define _P6_GPIO_ROTATOR_STATUS	0x620C

#define _P6_GPIO_INTC1      0x6400       /* GPIO interrupt 1 control */
#define _P6_GPIO_INTC2      0x6404       /* GPIO interrupt 2 control */
#define _P6_GPIO_INTC3      0x6408       /* GPIO interrupt 3 control */
#define _P6_GPIO_INTC4      0x640C       /* GPIO interrupt 4 control */
#define _P6_GPIO_INTC5      0x6410       /* GPIO interrupt 5 control */
#define _P6_GPIO_INTC6      0x6414       /* GPIO interrupt 6 control */
#define _P6_GPIO_INTC7      0x6418       /* GPIO interrupt 7 control */
#define _P6_GPIO_INTC8      0x641C       /* GPIO interrupt 8 control */
#define _P6_GPIO_INTC9      0x6420       /* GPIO interrupt 9 control */
#define _P6_GPIO_INTC10     0x6424       /* GPIO interrupt 10 control */
#define _P6_GPIO_INTC11     0x6428       /* GPIO interrupt 11 control */
#define _P6_GPIO_INTC12     0x642C       /* GPIO interrupt 12 control */
#define _P6_GPIO_INTC13     0x6430       /* GPIO interrupt 13 control */
#define _P6_GPIO_INTC14     0x6434       /* GPIO interrupt 14 control */
#define _P6_GPIO_INTC15     0x6438       /* GPIO interrupt 15 control */
#define _P6_GPIO_INTC16     0x643C       /* GPIO interrupt 16 control */

/* Registers bitwise definitions */

/* Interrupt configuration */
#define _P6_GPIO_INTCX_ENA	(1 << 20)    /* Enable */
#define _P6_GPIO_INTCX_POS	(0 << 16)    /* Positive edge */
#define _P6_GPIO_INTCX_BOTH	(1 << 16)    /* Both edge */
#define _P6_GPIO_INTCX_NEG	(2 << 16)    /* Negative edge */
#define _P6_GPIO_INTCX_LEV	(3 << 16)    /* Level */
#define _P6_GPIO_INTCX_LOW	(1 << 18)    /* Low Level (if bit is set) */
#define _P6_GPIO_INTCX_CLR	(1 << 19)    /* Clear */
#define _P6_GPIO_INTCX_DEBOUNCE (1 << 22)    /* Debounce enable */
#define _P6_GPIO_INTCX_FILTER	(1 << 23)    /* Filter mode */

#define _P6_GPIO_ROTATOR_NONE		0
#define _P6_GPIO_ROTATOR_NEG		1
#define _P6_GPIO_ROTATOR_BOTH		2
#define _P6_GPIO_ROTATOR_POS		3

#define _P6_GPIO_DEBOUNCE_START		0
#define _P6_GPIO_DEBOUNCE_END		7
#define _P6_GPIO_NODEBOUNCE_START	8
#define _P6_GPIO_NODEBOUNCE_END		19

#endif /* __ARCH_ARM_PARROT_REGS_GPIO_P6 */
