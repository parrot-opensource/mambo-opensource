/*
 *  linux/arch/arm/mach-parrot/devs.h
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
#ifndef __ASM_ARCH_PARROT_DEVS_H
#define __ASM_ARCH_PARROT_DEVS_H

#include <linux/platform_device.h>

extern struct platform_device parrot5_uart0_device;
extern struct platform_device parrot5_uart1_device;
extern struct platform_device parrot5_uart2_device;
extern struct platform_device p5p_nand_device;
extern struct platform_device p5p_dmac_device;
extern struct platform_device p5p_psd0_device;
extern struct platform_device p5p_psd1_device;
extern struct platform_device p5p_gpio;
extern struct platform_device p5p_aai_device;
extern struct platform_device parrot5_i2cm_device;


#endif
