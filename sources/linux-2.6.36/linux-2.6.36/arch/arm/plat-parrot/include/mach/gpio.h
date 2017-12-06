/**
 * @file linux/include/asm-arm/arch-parrot/gpio.h
 * @brief Parrot ASICs GPIO interface
 *
 * Copyright (C) 2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2006-02-15
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
#ifndef __ARCH_ARM_PARROT_GPIO_REGS_H
#define __ARCH_ARM_PARROT_GPIO_REGS_H

#include "regs-gpio-p5.h"
#include "regs-gpio-p5p.h"
#include "regs-gpio-p6.h"

/* Useful macros for internal use */

#ifndef __ASSEMBLY__

#include <asm/io.h>
#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <linux/errno.h>

#define GPIO_IOCR_OFFSET(_gpio) (((_gpio) >> 4) << 2)

#ifdef CONFIG_ARCH_PARROT5

#define DDR_BASE  (parrot_chip_is_p5p()? _P5P_GPIO_P0007DDR : _P5_GPIO_P0007DDR)
#define DR_BASE   _P5_GPIO_P0007DR
#define GPIO_MODE 0
#define GPIO_IOCR(_gpio)						\
	(((_gpio) < 48)?						\
	 _P5_SYS_IOCR1  + GPIO_IOCR_OFFSET(_gpio) :			\
	 _P5P_SYS_IOCR5 + GPIO_IOCR_OFFSET((_gpio)-48))


#else /* Parrot6 */

#define DDR_BASE             _P6_GPIO_P0007DDR
#define DR_BASE              _P6_GPIO_P0007DR
#define GPIO_MODE            1
#define GPIO_IOCR(_gpio)     (_P6_SYS_IOCR1 + GPIO_IOCR_OFFSET(_gpio))

#endif

#define __DR(_x)             (DR_BASE + 0x400*((_x) >> 3))
#define __DR_SHIFT(_x)       ((_x) & 0x7)
#define __ADDR(_x)           (__DR(_x)+(1 << (2+__DR_SHIFT(_x))))
#define __DDR_SHIFT(_x)      ((_x) & 0x7)
#define __DDR(_x)            (DDR_BASE + 4*((_x) >> 3))

/* GPIO interface */
static inline int gpio_get_value(unsigned gpio)
{
	return __raw_readl(PARROT_VA_GPIO+__ADDR(gpio));
}

static inline void gpio_set_value(unsigned gpio, int value)
{
#ifdef CONFIG_HBRIDGE_SAFETY_HACK
	extern int pwm0_output_bkp;
	extern int pwm1_output_bkp;
	if (gpio == 38) {
		/* unknow or 0 */
		if (pwm0_output_bkp != 1 && value) {
			printk("trying to set gpio38 while pwm state %d (OE %d)\n", pwm0_output_bkp, gpio_get_value(42));
			return;
		}
	}
	if (gpio == 39) {
		/* unknow or 0 */
		if (pwm1_output_bkp != 1 && value) {
			printk("trying to set gpio39 while pwm state %d (OE %d)\n", pwm1_output_bkp, gpio_get_value(42));
			return;
		}
	}
	if (gpio == 42) {
		printk("changing OE %d\n", value);
		if (value == 0 && (pwm0_output_bkp == -1 || pwm1_output_bkp == -1)) {
			printk("trying to clear OE in undefined state\n");
			return;
		}
	}

#endif
	__raw_writel((value)? 0xff : 0x00, PARROT_VA_GPIO+__ADDR(gpio));
}

static inline int gpio_cansleep(unsigned gpio)
{
	return 0;
}

int gpio_to_irq(unsigned gpio);

#ifdef CONFIG_GPIOLIB
//XXX could always be included
#include <asm-generic/gpio.h>
#else
static inline int gpio_is_valid(int number)
{
	//XXX gpio 0 should be valid...
	    return number > 0;
}

static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned gpio)
{
	return;
}

int gpio_direction_input(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);


//int irq_to_gpio(unsigned irq);

#endif

#endif /* !__ASSEMBLY__ */

#endif /* __ARCH_ARM_PARROT_GPIO_REGS */
