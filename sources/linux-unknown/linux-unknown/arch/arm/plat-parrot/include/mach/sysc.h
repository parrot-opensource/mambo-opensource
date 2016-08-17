/**
 * @file linux/include/asm-arm/arch-parrot/sysc.h
 * @brief Parrot System controller registers
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2005-11-17
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
#ifndef __ARCH_ARM_PARROT_SYSCONTROL_H
#define __ARCH_ARM_PARROT_SYSCONTROL_H


#ifdef CONFIG_ARCH_PARROT5

/* Parrot5/5+ */
#include "regs-sysc-p5.h"
#include "pins-p5.h"

#else

/* Parrot6 */
#include "regs-sysc-p6.h"
#include "pins-p6.h"

#endif

/* common defines */

/* ASIC chip identification strings */
#define _SYS_CHIPID                 (0x3c)
#define _SYS_CHIPID_P5              (0x312D3550) /* "P5-1" */
#define _SYS_CHIPID_P5P             (0x31503550) /* "P5P1" */
#define _SYS_CHIPID_P6              (0x30523650) /* "P6R0" */
#define _SYS_CHIPID_P6I             (0x30693650) /* "P6i0" */

#endif /* __ARCH_ARM_PARROT_SYSCONTROL_H */
