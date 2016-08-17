/**
 * @file linux/include/asm-arm/arch-parrot/parrot.h
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
#ifndef __ASM_ARCH_PARROT_H
#define __ASM_ARCH_PARROT_H

#include <linux/kernel.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/system.h>
#include <mach/platform.h>
#include <mach/map.h>

/* Parrot chips utilities */

enum {
    PARROT_CHIP_UNKNOWN = 0,
    PARROT_CHIP_P5,
    PARROT_CHIP_P5P,
    PARROT_CHIP_P6,
    PARROT_CHIP_P6I,
};

#if defined(CONFIG_ARCH_VARIANT_PARROT6) && defined(CONFIG_ARCH_VARIANT_PARROT6I)
extern u32 parrot_chipid;
#elif defined(CONFIG_ARCH_VARIANT_PARROT6)
#define parrot_chipid _SYS_CHIPID_P6
#elif defined(CONFIG_ARCH_VARIANT_PARROT6I)
#define parrot_chipid _SYS_CHIPID_P6I
#else
#error unknow cpu
#endif

static inline u32 parrot_chip_id_raw(void)
{
	/* chip id register offset is the same for all chips */
	return (u32)__raw_readl(PARROT_VA_SYS+_SYS_CHIPID);
}

/* 
 * cached version of parrot_chip_id_raw, chip_id shouldn't dynamicly change ;) 
 */
static inline u32 parrot_chip_id(void)
{
#ifndef parrot_chipid
	if (parrot_chipid == 0)
		parrot_chipid = parrot_chip_id_raw();
#endif
	return parrot_chipid;
}

static inline int parrot_chip(void)
{
	int ret = PARROT_CHIP_UNKNOWN;

	u32 id = parrot_chip_id();

	if (id == _SYS_CHIPID_P6I) {
		ret = PARROT_CHIP_P6I;
	}
	else if (id == _SYS_CHIPID_P6) {
		ret = PARROT_CHIP_P6;
	}
	else if (id == _SYS_CHIPID_P5P) {
		ret = PARROT_CHIP_P5P;
	}
	else if (id == _SYS_CHIPID_P5) {
		ret = PARROT_CHIP_P5;
	}

	return ret;
}

#ifdef CONFIG_ARCH_PARROT5
#define parrot_family_is_p5() 1
#define parrot_family_is_p6() 0
#elif CONFIG_ARCH_PARROT6
#define parrot_family_is_p5() 0
#define parrot_family_is_p6() 1
#else
#error unknow family
#endif

static inline int parrot_chip_is_p5(void)
{
	if (!parrot_family_is_p5())
		return 0;

	return (parrot_chip_id() == _SYS_CHIPID_P5);
}

static inline int parrot_chip_is_p5p(void)
{
	if (!parrot_family_is_p5())
		return 0;

	return (parrot_chip_id() == _SYS_CHIPID_P5P);
}

static inline int parrot_chip_is_p6(void)
{
	if (!parrot_family_is_p6())
		return 0;

	return (parrot_chip_id() == _SYS_CHIPID_P6);
}

static inline int parrot_chip_is_p6i(void)
{
	if (!parrot_family_is_p6())
		return 0;

	return (parrot_chip_id() == _SYS_CHIPID_P6I);
}

static inline int parrot_board_rev(void)
{
	return system_rev & 0xff;
}

/* check if a pin is already selected */
static inline int parrot_is_select_pin(unsigned int pin)
{
	u32 old, addr, val;
	int shift = (pin & 0xf0) >> 3;

	addr = PARROT_VA_SYS+(pin >> 8);
	old = __raw_readl(addr);
	val = (old & ~(3 << shift)) | ((pin & 3) << shift);
	if (old != val) {
		return 0;
	}
	return 1;
}

static inline void parrot_select_pin(unsigned int pin)
{
	u32 old, addr, val;
	int shift = (pin & 0xf0) >> 3;

	addr = PARROT_VA_SYS+(pin >> 8);
	old = __raw_readl(addr);
	val = (old & ~(3 << shift)) | ((pin & 3) << shift);
	if (old != val) {
		printk(KERN_DEBUG "p6 mux select pin @0x%08x : 0x%08x->0x%08x\n", addr, old, val);
		__raw_writel(val, addr);
	}
}

static inline void parrot_init_pin(const unsigned int *pin)
{
	while (*pin) {
		parrot_select_pin(*pin);
		pin++;
	}
}

#endif /* __ASM_ARCH_PARROT_H */
