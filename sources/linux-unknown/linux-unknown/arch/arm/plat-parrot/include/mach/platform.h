/**
 * @file linux/include/asm-arm/arch-parrot/platform.h
 * @brief Parrot ASICs device mapping and registers
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     yves.lemoine@parrot.com
 * @author     ivan.djelic@parrot.com
 * @date       2005-09-28
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
#ifndef __ARCH_ARM_PARROT_PLATFORM_H
#define __ARCH_ARM_PARROT_PLATFORM_H

#ifndef __ASSEMBLY__
/* may be we shouldn't include this here ... */
#include <linux/types.h> /* need for vic.h */
#endif
#include <asm/hardware/vic.h>

#include "sysc.h"
#include "irqs.h"

/* common definitions */

#include "regs-uart.h"

/* chip-specific definitions */

#ifdef CONFIG_ARCH_PARROT5

#include "regs-mpmc.h"
#include "regs-dma-p5p.h"
#include "platform-p5.h"

#else

#include "platform-p6.h"

#endif

#endif /* __ARCH_ARM_PARROT_PLATFORM_H */
