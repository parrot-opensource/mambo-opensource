/*
 *  linux/arch/arm/mach-parrot6/devs.c
 *
 * Copyright (C) 2008 Parrot S.A.
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

/*
 * WARNING :
 * Note only real block should be set here, not virtual interface, ie there
 * shouldn't be resource with the same memory io.
 * We does that because multiplexing can be very complex and shouldn't be
 * managed by driver. Let's take an example with uart where RX,TX,RTS,CTS
 * can be output on config a,b,c. There is the trivial config all four pins
 * on the same config, but nothing forbid to mix config ie RX on config a,
 * TX on config b, RTS on config c and no CTS.
 * Doing all possible configuration, will make a *huge* number of config.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <mach/map.h>

#include "devs.h"

/* Parrot6 devices */

/* Serial ports */

static struct resource p6_uart0_resource[] = {
	[0] = {
		.start = PARROT6_UART0,
		.end   = PARROT6_UART0+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_UART0,
		.end   = IRQ_P6_UART0,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource p6_uart1_resource[] = {
	[0] = {
		.start = PARROT6_UART1,
		.end   = PARROT6_UART1+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_UART1,
		.end   = IRQ_P6_UART1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource p6_uart2_resource[] = {
	[0] = {
		.start = PARROT6_UART2,
		.end   = PARROT6_UART2+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_UART2,
		.end   = IRQ_P6_UART2,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource p6_uart3_resource[] = {
	[0] = {
		.start = PARROT6_UART3,
		.end   = PARROT6_UART3+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_UART3,
		.end   = IRQ_P6_UART3,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p6_uart0_device = {
	.name		= "parrot5-uart",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6_uart0_resource),
	.resource	= p6_uart0_resource,
};

struct platform_device p6_uart1_device = {
	.name		= "parrot5-uart",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(p6_uart1_resource),
	.resource	= p6_uart1_resource,
};

struct platform_device p6_uart2_device = {
	.name		= "parrot5-uart",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(p6_uart2_resource),
	.resource	= p6_uart2_resource,
};

struct platform_device p6_uart3_device = {
	.name		= "parrot5-uart",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(p6_uart3_resource),
	.resource	= p6_uart3_resource,
};

static struct resource p6_nand_resource[] = {
	[0] = {
		.start = PARROT6_NAND,
		.end   = PARROT6_NAND+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = PARROT6_NAND_DATA,
		.end   = PARROT6_NAND_DATA+SZ_1M-1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.start = IRQ_P6_NANDMC,
		.end   = IRQ_P6_NANDMC,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p6_nand_device = {
	.name		= "p6-nand",
	.num_resources	= ARRAY_SIZE(p6_nand_resource),
	.resource	= p6_nand_resource,
};

static struct resource p6_dmac_resource[] = {
	[0] = {
		.start = PARROT6_DMAC,
		.end   = PARROT6_DMAC+SZ_1M-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_DMAC,
		.end   = IRQ_P6_DMAC,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p6_dmac_device = {
	.name		= "dma-pl08x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6_dmac_resource),
	.resource	= p6_dmac_resource,
};

static struct resource p6_parint_resource[] = {
	[0] = {
		.start = PARROT6_PARINT,
		.end   = PARROT6_PARINT+SZ_1M-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_PARINT,
		.end   = IRQ_P6_PARINT,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p6_parint_device = {
	.name		= "p6-parint",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6_parint_resource),
	.resource	= p6_parint_resource,
	.dev = {
		.coherent_dma_mask	= DMA_32BIT_MASK
	},
};

/* Parrot6 PSD controllers */

static struct resource p6_psd0_resource[] = {
	[0] = {
		.start = PARROT6_MPSD0,
		.end   = PARROT6_MPSD0+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_MPSDX,
		.end   = IRQ_P6_MPSDX,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource p6_psd1_resource[] = {
	[0] = {
		.start = PARROT6_MPSD1,
		.end   = PARROT6_MPSD1+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_MPSDX,
		.end   = IRQ_P6_MPSDX,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p6_psd0_device = {
	.name		= "p5p-psd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6_psd0_resource),
	.resource	= p6_psd0_resource,
};

/* XXX merge this config */
struct platform_device p6_psd1_device = {
	.name		= "p5p-psd",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(p6_psd1_resource),
	.resource	= p6_psd1_resource,
};

/* Parrot6 SDHCI controller (from Arasan) */

#define SDHCI_SLOT_WIDTH 0x100
static struct resource p6_sdhci0_resource[] = {
	[0] = {
		.start = PARROT6_SDIO+0*SDHCI_SLOT_WIDTH,
		.end   = PARROT6_SDIO+1*SDHCI_SLOT_WIDTH-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_SDIO,
		.end   = IRQ_P6_SDIO,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource p6_sdhci1_resource[] = {
	[0] = {
		.start = PARROT6_SDIO+1*SDHCI_SLOT_WIDTH,
		.end   = PARROT6_SDIO+2*SDHCI_SLOT_WIDTH-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_SDIO,
		.end   = IRQ_P6_SDIO,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource p6_sdhci2_resource[] = {
	[0] = {
		.start = PARROT6_SDIO+2*SDHCI_SLOT_WIDTH,
		.end   = PARROT6_SDIO+3*SDHCI_SLOT_WIDTH-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_SDIO,
		.end   = IRQ_P6_SDIO,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p6_sdhci0_device = {
	.name		= "p6-sdhci",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6_sdhci0_resource),
	.resource	= p6_sdhci0_resource,
};

struct platform_device p6_sdhci1_device = {
	.name		= "p6-sdhci",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(p6_sdhci1_resource),
	.resource	= p6_sdhci1_resource,
};

struct platform_device p6_sdhci2_device = {
	.name		= "p6-sdhci",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(p6_sdhci2_resource),
	.resource	= p6_sdhci2_resource,
};

struct platform_device p6_gpio = {
	.name           = "gpio",
	.id             = -1,
	.num_resources  = 0,
	.resource       = NULL,
};

struct platform_device user_gpio = {
	.name          = "user_gpio",
	.id            = -1,
	.num_resources  = 0,
	.resource       = NULL,
	.dev = {
		.platform_data = NULL,
	},
};


/* Parrot6 H264 */
static struct resource p6_h264_resource[] = {
        [0] = {
                .start  = PARROT6_H264,
                .end    = PARROT6_H264+SZ_64K-1,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = IRQ_P6_H264,
                .end    = IRQ_P6_H264,
                .flags  = IORESOURCE_IRQ,
        }
};

struct platform_device p6_h264 = {
        .name           = "h264_p6",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(p6_h264_resource),
        .resource       = p6_h264_resource,
};

/* Parrot6 Advanced Audio Interface (AAI) */
static struct resource p6_aai_resource[] = {
	[0] = {
		.start  = PARROT6_AAI,
		.end    = PARROT6_AAI+SZ_64K-1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_P6_AAI,
		.end    = IRQ_P6_AAI,
		.flags  = IORESOURCE_IRQ,
	}
};


struct platform_device p6_us_device = {
	.name		= "ultra_snd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6_aai_resource),
	.resource	= p6_aai_resource,
	.dev = {
		.platform_data = NULL,
	},
};


struct platform_device p6_aai_device = {
	.name		= "aai",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6_aai_resource),
	.resource	= p6_aai_resource,
};

static struct resource p6_i2cs_resource[] = {
	[0] = {
		.start = PARROT6_I2CS,
		.end   = PARROT6_I2CS+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_I2CS,
		.end   = IRQ_P6_I2CS,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p6_i2cs_device = {
	.name             = "parrot5-i2cs",
	.id               = 0,
	.num_resources    = ARRAY_SIZE(p6_i2cs_resource),
	.resource         = p6_i2cs_resource,
};

/* Parrot I2C master controller */

static struct resource p6_i2cm0_resource[] = {
	[0] = {
		.start = PARROT6_I2CM0,
		.end   = PARROT6_I2CM0+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_I2CM,
		.end   = IRQ_P6_I2CM,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p6_i2cm0_device = {
	.name             = "parrot5-i2cm",
	.id               = 0,
	.num_resources    = ARRAY_SIZE(p6_i2cm0_resource),
	.resource         = p6_i2cm0_resource,
};

static struct resource p6_i2cm1_resource[] = {
	[0] = {
		.start = PARROT6_I2CM1,
		.end   = PARROT6_I2CM1+SZ_64K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_I2CM,
		.end   = IRQ_P6_I2CM,
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device p6_i2cm1_device = {
	.name             = "parrot5-i2cm",
	.id               = 1,
	.num_resources    = ARRAY_SIZE(p6_i2cm1_resource),
	.resource         = p6_i2cm1_resource,
};


/* Parrot6 USB controllers */
static struct resource p6_usb0_resource[] = {
	[0] = {
		.start = PARROT6_USB0,
		.end   = PARROT6_USB0 + SZ_1M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_USB0,
		.end   = IRQ_P6_USB0,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device p6_usb0_device = {
	.name		= "dwc_otg",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6_usb0_resource),
	.resource	= p6_usb0_resource,
};

static struct resource p6_usb1_resource[] = {
	[0] = {
		.start = PARROT6_USB1,
		.end   = PARROT6_USB1 + SZ_1M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_USB1,
		.end   = IRQ_P6_USB1,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device p6_usb1_device = {
	.name		= "dwc_otg",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(p6_usb1_resource),
	.resource	= p6_usb1_resource,
};

/* LCD Controller */

static struct resource p6_lcd_resource[] = {
	[0] = {
		.start = PARROT6_LCDC,
		.end   = PARROT6_LCDC + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_LCDC,
		.end   = IRQ_P6_LCDC,
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device p6_lcd_device = {
	.name		  = "p6-lcd",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(p6_lcd_resource),
	.resource	  = p6_lcd_resource,
	.dev = {
		.coherent_dma_mask	= DMA_32BIT_MASK
	},
};

/* SPI Controller */
static struct resource p6_spi0_resource[] = {
	[0] = {
		.start = PARROT6_SPI0,
		.end   = PARROT6_SPI0 + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_SPIX,
		.end   = IRQ_P6_SPIX,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource p6_spi1_resource[] = {
	[0] = {
		.start = PARROT6_SPI1,
		.end   = PARROT6_SPI1 + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_SPIX,
		.end   = IRQ_P6_SPIX,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource p6_spi2_resource[] = {
	[0] = {
		.start = PARROT6_SPI2,
		.end   = PARROT6_SPI2 + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_SPIX,
		.end   = IRQ_P6_SPIX,
		.flags = IORESOURCE_IRQ,
	},
};


struct platform_device p6_spi0_device = {
	.name		= "p6-spi",
	.id		= 0,
	.dev		= {
		.coherent_dma_mask = DMA_32BIT_MASK,
	},
	.num_resources	= ARRAY_SIZE(p6_spi0_resource),
	.resource	= p6_spi0_resource,
};

struct platform_device p6_spi1_device = {
	.name		= "p6-spi",
	.id		= 1,
	.dev		= {
		.coherent_dma_mask = DMA_32BIT_MASK,
	},
	.num_resources	= ARRAY_SIZE(p6_spi1_resource),
	.resource	= p6_spi1_resource,
};

struct platform_device p6_spi2_device = {
	.name		= "p6-spi",
	.id		= 2,
	.dev		= {
		.coherent_dma_mask = DMA_32BIT_MASK,
	},
	.num_resources	= ARRAY_SIZE(p6_spi2_resource),
	.resource	= p6_spi2_resource,
};

/* camif */
static struct resource p6_camif0_resources[] = {
	[0] = {
		.start	= PARROT6_CAMIF0,
		.end	= PARROT6_CAMIF0 + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_CAMIF0,
		.flags  = IORESOURCE_IRQ,
	},
	/* if nothing is set here, memory will be allocated from main
	   memory.
	 */
#if 0
	[2] = {
		/* place holder for contiguous memory */
		.start	= 0x40000000+96*1024*1024,
		.end	= 0x40000000+128*1024*1024-1,
		.flags	= IORESOURCE_MEM,
	},
#endif
};

struct platform_device p6_camif0_device = {
	.name		= "p6_camif",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6_camif0_resources),
	.resource	= p6_camif0_resources,
	.dev = {
		.coherent_dma_mask	= DMA_32BIT_MASK
	},
};

static struct resource p6_camif1_resources[] = {
	[0] = {
		.start	= PARROT6_CAMIF1,
		.end	= PARROT6_CAMIF1 + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_CAMIF1,
		.flags  = IORESOURCE_IRQ,
	},
	/* if nothing is set here, memory will be allocated from main
	   memory.
	 */
#if 0
	[2] = {
	},
#endif
};


struct platform_device p6_camif1_device = {
	.name		= "p6_camif",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(p6_camif1_resources),
	.resource	= p6_camif1_resources,
	.dev = {
		.coherent_dma_mask	= DMA_32BIT_MASK
	},
};


/* Parrot6 keyboard */
struct platform_device p6_kbd_device = {
	.name		= "p6_kbd_input",
	.id		= 0,
	.num_resources	= 0,
	.resource	= 0,
};
/* Parrot6 keyboard */

struct platform_device dmamem_device = {
	.name		= "dmamem",
	.id		= 0,
	.num_resources	= 0,
	.resource	= 0,
	.dev = {
		.coherent_dma_mask	= DMA_32BIT_MASK
	},
};

struct platform_device p6_acpower_device = {
	.name		= "p6-acpower",
	.id		= 0,
	.num_resources	= 0,
	.resource	= 0,
};


/* p6i usb */
static struct resource p6i_usb0_resource[] = {
	[0] = {
		.start = PARROT6_USB0,
		.end   = PARROT6_USB0 + SZ_1M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_P6_USB0,
		.end   = IRQ_P6_USB0,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 dma_mask = DMA_32BIT_MASK;

struct platform_device p6i_usb0_device = {
	.name		= "p6i-ehci",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p6i_usb0_resource),
	.resource	= p6i_usb0_resource,
	.dev = {
		.coherent_dma_mask	= DMA_32BIT_MASK,
		.dma_mask = &dma_mask,
	},
};

