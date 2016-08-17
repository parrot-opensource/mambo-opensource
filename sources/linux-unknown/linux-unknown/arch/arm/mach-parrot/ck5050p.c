/*
 *  linux/arch/arm/mach-parrot/parrot5.c
 *
 *  Copyright (C) 2007 Parrot S.A.
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

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/mmc.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/i2c.h>
#include <mach/dma-pl08x.h>

#include "timer.h"
#include "devs.h"

static struct map_desc parrot5_io_desc[] __initdata = {
	{
		.virtual	= PARROT5_VA_VIC,
		.pfn		= __phys_to_pfn(PARROT5_VIC),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT5_VA_MPMC,
		.pfn		= __phys_to_pfn(PARROT5_MPMC),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT5_VA_SYS,
		.pfn		= __phys_to_pfn(PARROT5_SYS),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT5_VA_GPIO,
		.pfn		= __phys_to_pfn(PARROT5_GPIO),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
};

static struct parrot5_i2cm_platform i2cm_platform_parrot5 = {
	.bus_freq	= 10*1000,
	.retries        = 1,
};

static struct mmc_platform_data mmc_platform_parrot5 = {
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
};


unsigned int pins_init_ck5050p[] = {
	UART0_RX,
	UART0_TX,
	UART1_RX,
	UART1_TX,
	UART2_RX,
	UART2_TX,
	UART2_RTS,
	UART2_CTS,
	SCL1,
	SDA1,
	AC_OUT1a,
	AC_IN1a,
	AC_OUT2a,
	AC_IN2a,
	AC_CLKa,
	AC_SYNCa,
	MCLKa,
	0,
};

/* PL08x DMA controller flow control table */
static const int pl08x_dma_flowctrl[_PL080_PERIPH_MAX][_PL080_PERIPH_MAX] = {
	/* source is PSD0 (and flow controller) */
	[_PL080_PERIPH_PSD0] = {
		[_PL080_PERIPH_PSD0] = -1,
		[_PL080_PERIPH_PSD1] = -1,
		[_PL080_PERIPH_MEM]  = _PL080_CXCONFIG_FLOWCNTRL_P2M_PERIPH,
	},
	/* source is PSD1 (and flow controller) */
	[_PL080_PERIPH_PSD1] = {
		[_PL080_PERIPH_PSD0] = -1,
		[_PL080_PERIPH_PSD1] = -1,
		[_PL080_PERIPH_MEM]  = _PL080_CXCONFIG_FLOWCNTRL_P2M_PERIPH,
	},
	/* source is memory */
	[_PL080_PERIPH_MEM] = {
		[_PL080_PERIPH_PSD0] = _PL080_CXCONFIG_FLOWCNTRL_M2P_PERIPH,
		[_PL080_PERIPH_PSD1] = _PL080_CXCONFIG_FLOWCNTRL_M2P_PERIPH,
		[_PL080_PERIPH_MEM]  = _PL080_CXCONFIG_FLOWCNTRL_M2M_DMAC,
	},
};

static struct platform_device *parrot5_devices[] __initdata = {
	&parrot5_uart0_device,
	&parrot5_uart1_device,
	&parrot5_uart2_device,
	//&p5p_nand_device,
	&p5p_gpio,
	&p5p_aai_device,
	&p5p_psd0_device,
	&parrot5_i2cm_device,
};

static void __init parrot5_map_io(void)
{
	iotable_init(parrot5_io_desc, ARRAY_SIZE(parrot5_io_desc));
	/* this should be done after static mapping */
}

static void __init parrot5_init_irq(void)
{
	vic_init(__io(PARROT5_VA_VIC), 0, ~0);
}

static void __init parrot5_init(void)
{
	parrot_init_pin(pins_init_ck5050p);
   __raw_writel(0x8003, PARROT_VA_SYS+_P5_SYS_IOCR4);

	parrot5_i2cm_device.dev.platform_data = &i2cm_platform_parrot5;
	p5p_psd0_device.dev.platform_data = &mmc_platform_parrot5;
	p5p_dmac_device.dev.platform_data = &pl08x_dma_flowctrl;

	platform_add_devices(parrot5_devices, ARRAY_SIZE(parrot5_devices));
}

MACHINE_START(CK5050P, "Parrot ck5050p(P5+) platform")
        /* Maintainer: Parrot S.A. */
        .phys_io	= PARROT5_UART0,
	.io_pg_offst	= (PARROT5_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT5_CS0_PHYS_BASE+0x100,
	.map_io		= parrot5_map_io,
	.init_irq	= parrot5_init_irq,
	.timer		= &parrot5_timer,
	.init_machine	= parrot5_init,
MACHINE_END
