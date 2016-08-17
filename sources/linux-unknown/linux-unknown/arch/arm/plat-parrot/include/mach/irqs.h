/**
 * @file linux/include/asm-arm/arch-parrot/irqs.h
 * @brief Parrot ASICs interrupt vectors
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2004-11-16
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
#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

#ifdef CONFIG_ARCH_PARROT5
#include "irqs-p5.h"
#define NR_IRQS			(32)
#else
#include "irqs-p6.h"
#define NR_IRQS			(64)
#endif

#endif /* __ASM_ARCH_IRQS_H */
