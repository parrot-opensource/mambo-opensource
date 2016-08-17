/**
 * @file linux/include/asm-arm/arch-parrot/map.h
 * @brief Parrot ASICs device virtual mapping
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
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
 *
 */
#ifndef __ARCH_ARM_PARROT_MAP_H
#define __ARCH_ARM_PARROT_MAP_H

#include "hardware.h"

/*
 * VMALLOC_END .. feffffff are free for static mapping
 * we set VMALLOC_END to PARROT_VA_VIC
 *
 * all physical I/O addresses PA=y0xxxxxx are remmapped onto VA=Fyxxxxxx
 */
#define P5_IOVA(_x) (0xf0000000+(((_x) >> 4) & 0x0f000000)+((_x) & 0x00ffffff))

#ifdef CONFIG_ARCH_PARROT5
#include "map-p5.h"

/* mappings common to all chips */
#define PARROT_VA_VIC     PARROT5_VA_VIC
#define PARROT_VA_SYS     PARROT5_VA_SYS
#define PARROT_VA_GPIO    PARROT5_VA_GPIO

#else
#include "map-p6.h"

/* mappings common to all chips */
#define PARROT_VA_VIC     PARROT6_VA_VIC
#define PARROT_VA_SYS     PARROT6_VA_SYS
#define PARROT_VA_GPIO    PARROT6_VA_GPIO

#endif

#endif /* __ARCH_ARM_PARROT_MAP_H */
