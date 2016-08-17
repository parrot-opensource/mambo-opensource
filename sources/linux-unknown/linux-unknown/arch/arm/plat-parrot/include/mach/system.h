/**
 * @file linux/include/asm-arm/arch-parrot/system.h
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
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/hardware.h>
#include <asm/io.h>
#include <mach/platform.h>
#include <mach/map.h>

static inline void arch_idle(void)
{
#ifdef CONFIG_SERIAL_DCC
/* XXX make this dynamic */
#warning disabling cpu_do_idle for dcc it support
#else
	cpu_do_idle();
#endif
}

static inline void arch_reset(char mode, const char *cmd)
{
	/* force a software reset */
#ifdef CONFIG_ARCH_PARROT5
	__raw_writel(P5_SYS_WDOGCTL_SYSSOFTRESET, PARROT5_VA_SYS+_P5_SYS_WDOGCTL);
#else
	__raw_writel(P6_SYS_WDOGCTL_SYSSOFTRESET, PARROT6_VA_SYS+_P6_SYS_WDOGCTL);
#endif
}

#endif
