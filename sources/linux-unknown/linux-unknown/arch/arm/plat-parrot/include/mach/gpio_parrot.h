/*
 * linux/arch/arm/mach-parrot6/include/mach/gpio_parrot.h
 *
 * Parrot6 GPIO interface
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     florent.bayendrian@parrot.com
 * @date       2008-03-11
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

#ifndef _GPIO_PARROT_H
#define _GPIO_PARROT_H

#include <mach/irqs.h>
#include <mach/irqs-p6.h>

#define _P6_GPIO_INTC_COUNT          20
#define _P6_ROTATOR_COUNT             2

// global IRQ for all GPIO
#if defined(CONFIG_ARCH_PARROT6)
#define IRQ_GPIO		IRQ_P6_GPIO
#elif defined(CONFIG_VERSATILE_PARROT6)
#define IRQ_GPIO		(IRQ_P6_GPIO+64)
#else
#error "unsupported platform"
#endif
// virtual IRQ
#define IRQ_GPIO_START		32
#define IRQ_GPIO_END		(IRQ_GPIO_START+_P6_GPIO_INTC_COUNT-1)
#define IRQ_ROTATOR_START	(IRQ_GPIO_END+1)
#define IRQ_ROTATOR_END         (IRQ_ROTATOR_START+_P6_ROTATOR_COUNT-1)

int gpio_request(unsigned gpio, const char *label);
void gpio_free(unsigned gpio);
int gpio_direction_input(unsigned gpio);
int gpio_get_value(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);
void gpio_set_value(unsigned gpio, int value);

typedef enum ROTATOR_EVENT_TAG {
	ROTATOR_EVENT_LEFT,
	ROTATOR_EVENT_RIGHT,
} ROTATOR_EVENT;

typedef enum GPIO_IRQ_MODE_TAG {
	GPIO_IRQ_POS,
	GPIO_IRQ_NEG,
	GPIO_IRQ_BOTH,
	GPIO_IRQ_LEVEL_H,
	GPIO_IRQ_LEVEL_L,
#if defined(CONFIG_ARCH_PARROT_USE_ROTATOR)
	GPIO_IRQ_ROTATOR_POS,
	GPIO_IRQ_ROTATOR_NEG,
	GPIO_IRQ_ROTATOR_BOTH,
#endif	
} GPIO_IRQ_MODE;

typedef enum GPIO_DEBOUNCE_MODE_TAG {
	GPIO_DEBOUNCE_NONE,
	GPIO_DEBOUNCE_NOISE,  /* noise detection + blind period */
	GPIO_DEBOUNCE_FILTER, /* filter input with GPIO_Debounce delay added */
} GPIO_DEBOUNCE_MODE;

int gpio_set_debounce_value(uint32_t value);

int gpio_interrupt_register(unsigned gpio,
                            GPIO_IRQ_MODE irq_mode,
			    GPIO_DEBOUNCE_MODE debounce_mode);

int gpio_interrupt_unregister(unsigned gpio);

enum ROTATOR_EVENT_TAG rotator_get_status(unsigned int rot);

#endif
