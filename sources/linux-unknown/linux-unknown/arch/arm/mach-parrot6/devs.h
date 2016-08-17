/*
 *  linux/arch/arm/mach-parrot6/devs.h
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
#ifndef __ASM_ARCH_PARROT6_DEVS_H
#define __ASM_ARCH_PARROT6_DEVS_H

#include <linux/platform_device.h>

extern struct platform_device p6_uart0_device;
extern struct platform_device p6_uart1_device;
extern struct platform_device p6_uart2_device;
extern struct platform_device p6_uart3_device;
extern struct platform_device p6_nand_device;
extern struct platform_device p6_dmac_device;
extern struct platform_device p6_psd0_device;
extern struct platform_device p6_psd1_device;
extern struct platform_device p6_sdhci0_device;
extern struct platform_device p6_sdhci1_device;
extern struct platform_device p6_sdhci2_device;
extern struct platform_device p6_gpio;
extern struct platform_device p6_h264;
extern struct platform_device p6_aai_device;
extern struct platform_device p6_us_device;
extern struct platform_device p6_i2cm0_device;
extern struct platform_device p6_i2cm1_device;
extern struct platform_device p6_usb0_device;
extern struct platform_device p6_usb1_device;
extern struct platform_device p6_lcd_device;
extern struct platform_device p6_spi0_device;
extern struct platform_device p6_spi1_device;
extern struct platform_device p6_spi2_device;
extern struct platform_device p6_camif0_device;
extern struct platform_device p6_camif1_device;
extern struct platform_device p6_kbd_device;
extern struct platform_device dmamem_device;
extern struct platform_device p6_acpower_device;
extern struct platform_device p6_parint_device;
extern struct platform_device p6_i2cs_device;

extern struct platform_device p6i_usb0_device;
extern struct platform_device user_gpio;

#endif /* __ASM_ARCH_PARROT6_DEVS_H */
