/**
 * @file linux/include/asm-arm/arch-parrot/memory.h
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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H


#include <mach/platform.h>

#ifdef CONFIG_ARCH_PARROT5

/* Physical memory offset for P5/P5+ family */
#define PHYS_OFFSET	UL(0x10000000)

#else

/* Physical memory offset for P6 family */
#define PHYS_OFFSET	UL(0x40000000)

#endif

/*
 * Virtual view <-> DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *              address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *              to an address that the kernel can use.
 */
/*
#define __virt_to_bus(x)	(x - PAGE_OFFSET + PHYS_OFFSET)
#define __bus_to_virt(x)	(x - PHYS_OFFSET + PAGE_OFFSET)
*/
/* define DMA address size */
#define CONSISTENT_DMA_SIZE (SZ_2M*7)
#endif
