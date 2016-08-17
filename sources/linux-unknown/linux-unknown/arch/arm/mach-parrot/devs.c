/*
 *  linux/arch/arm/mach-parrot/devs.c
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <mach/map.h>

#include "devs.h"

/* Parrot5/5+ devices */

/* Serial ports */

static struct resource parrot5_uart0_resource[] = {
	[0] = {
		.start = PARROT5_UART0,
		.end   = PARROT5_UART0+SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P5_UART0,
		.end   = IRQ_P5_UART0,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource parrot5_uart1_resource[] = {
	[0] = {
		.start = PARROT5_UART1,
		.end   = PARROT5_UART1+SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P5_UART1,
		.end   = IRQ_P5_UART1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource parrot5_uart2_resource[] = {
	[0] = {
		.start = PARROT5_UART2,
		.end   = PARROT5_UART2+SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P5_UART2,
		.end   = IRQ_P5_UART2,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device parrot5_uart0_device = {
	.name		= "parrot5-uart",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(parrot5_uart0_resource),
	.resource	= parrot5_uart0_resource,
};

struct platform_device parrot5_uart1_device = {
	.name		= "parrot5-uart",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(parrot5_uart1_resource),
	.resource	= parrot5_uart1_resource,
};

struct platform_device parrot5_uart2_device = {
	.name		= "parrot5-uart",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(parrot5_uart2_resource),
	.resource	= parrot5_uart2_resource,
};

/* Nand controller */
/* nand driver uses the five lower bits to control signal */
#define P5P_NAND_SIZE (1 << 5)
static struct resource p5p_nand_resource[] = {
	[0] = {
		.start = PARROT5_CS4_PHYS_BASE,
		.end   = PARROT5_CS4_PHYS_BASE+P5P_NAND_SIZE,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device p5p_nand_device = {
	.name		= "p5p-nand",
	.num_resources	= ARRAY_SIZE(p5p_nand_resource),
	.resource	= p5p_nand_resource,
};

/* Parrot5+ DMA controller */
static struct resource p5p_dmac_resource[] = {
	[0] = {
		.start = PARROT5_DMAC,
		.end   = PARROT5_DMAC+SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P5P_DMAC,
		.end   = IRQ_P5P_DMAC,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p5p_dmac_device = {
	.name		= "dma-pl08x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p5p_dmac_resource),
	.resource	= p5p_dmac_resource,
};

/* Parrot5+ PSD controllers */

static struct resource p5p_psd0_resource[] = {
	[0] = {
		.start = PARROT5_MPSD0,
		.end   = PARROT5_MPSD0+SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P5_MPSD0,
		.end   = IRQ_P5_MPSD0,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource p5p_psd1_resource[] = {
	[0] = {
		.start = PARROT5_MPSD1,
		.end   = PARROT5_MPSD1+SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P5_MPSD1,
		.end   = IRQ_P5_MPSD1,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p5p_psd0_device = {
	.name		= "p5p_psd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p5p_psd0_resource),
	.resource	= p5p_psd0_resource,
};

struct platform_device p5p_psd1_device = {
	.name		= "p5p_psd",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(p5p_psd1_resource),
	.resource	= p5p_psd1_resource,
};

/* Parrot5+ GPIO controller */

struct platform_device p5p_gpio = {
	.name             = "gpio",
	.id               = -1,
	.num_resources    = 0,
	.resource         = NULL,
};

/*
 * Parrot5+ AAI
 */
static struct resource p5p_aai_resource[] = {
	[0] = {
		.start = PARROT5_AAI,
		.end   = PARROT5_AAI+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P5_AAI,
		.end   = IRQ_P5_AAI,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p5p_aai_device = {
	.name		      = "aai",
	.id		      = 0,
	.num_resources	= ARRAY_SIZE(p5p_aai_resource),
	.resource	   = p5p_aai_resource,
};




/* Parrot I2C master controller */

static struct resource parrot5_i2cm_resource[] = {
	[0] = {
		.start = PARROT5_I2CM,
		.end   = PARROT5_I2CM + SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},

	[1] = {
		.start = IRQ_P5_I2CM,
		.end   = IRQ_P5_I2CM,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device parrot5_i2cm_device = {
	.name             = "parrot5-i2cm",
	.id               = 1,
	.num_resources    = ARRAY_SIZE(parrot5_i2cm_resource),
	.resource         = parrot5_i2cm_resource,
};
