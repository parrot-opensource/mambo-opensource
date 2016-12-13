/**
 * @file linux/include/asm-arm/arch-parrot/dma-pl08x.h
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2008-11-10
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
#ifndef __ASM_ARM_ARCH_DMA_PL08X_H
#define __ASM_ARM_ARCH_DMA_PL08X_H

#include <mach/regs-pl08x.h>
#include <asm/io.h>

/* define available DMA peripherals */
#if defined(CONFIG_ARCH_PARROT5)

#define _PL080_PERIPH_PSD0   (0)
#define _PL080_PERIPH_PSD1   (1)
#define _PL080_PERIPH_MEM    (2)
#define _PL080_PERIPH_MAX    (3)

#elif defined(CONFIG_ARCH_PARROT6)

#define _PL080_PERIPH_PARINT (0)
#define _PL080_PERIPH_NAND   (1)
#define _PL080_PERIPH_PSD0   (2)
#define _PL080_PERIPH_PSD1   (3)
#define _PL080_PERIPH_SPI0   (4)
#define _PL080_PERIPH_SPI1   (5)
#define _PL080_PERIPH_SPI2   (6)
#define _PL080_PERIPH_MEM    (7)
#define _PL080_PERIPH_MAX    (8)

#elif defined(CONFIG_ARCH_VERSATILE)

#define _PL080_PERIPH_SPI0   (0)
#define _PL080_PERIPH_MEM    (1)
#define _PL080_PERIPH_MAX    (2)

#endif

union pl08x_dma_cxctrl {
	struct {
		unsigned int transize:12;    /* transfer size (in width unit) */
		unsigned int sbsize:3;       /* source burst size */
		unsigned int dbsize:3;       /* destination burst size */
		unsigned int swidth:3;       /* source transfer width */
		unsigned int dwidth:3;       /* destination transfer width */
		unsigned int sahb:1;         /* source AHB master select */
		unsigned int dahb:1;         /* destination AHB master select */
		unsigned int si:1;           /* increment source address */
		unsigned int di:1;           /* increment destination address */
		unsigned int prot:3;         /* AHB protection */
		unsigned int i:1;            /* TC interrupt enable (ignored) */
	};
	u32 word;
};

struct pl08x_dma_cfg {
	union pl08x_dma_cxctrl  cxctrl;
	unsigned int            src_periph;
	unsigned int            dst_periph;
	u32                     src_addr;
	u32                     dst_addr;
	u32                     lli;
};

typedef void (*pl08x_dma_callback_t)(unsigned int chan, void *data, int status);

int pl08x_dma_request(unsigned int *channel, const char *devid,
		      pl08x_dma_callback_t callback, void *data);
int pl08x_dma_free(unsigned int channel);
int pl08x_dma_start(unsigned int channel, struct pl08x_dma_cfg *cfg);
int pl08x_dma_abort(unsigned int channel);
int pl08x_dma_wait(unsigned int channel);

#endif /* __ASM_ARM_ARCH_DMA_PL08X_H */
