/*
 * linux/arch/arm/plat-parrot/gpio.c
 *
 * Parrot6 GPIO driver
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

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/irq.h>

#include <asm/irq.h>
#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include <mach/map.h>

#define _P6_GPIO_COUNT          162
#define _P6I_GPIO_COUNT          59

#if defined(CONFIG_ARCH_PARROT6) || defined(CONFIG_VERSATILE_PARROT6)

#define GPIO_COUNT	(parrot_chip_is_p6() ? _P6_GPIO_COUNT: _P6I_GPIO_COUNT)

//! GPIO IRQ source
struct gpio_src {
	unsigned gpio;
	bool active;
};

static struct {
	struct gpio_src		src[_P6_GPIO_INTC_COUNT];
	u32			rot_status;
} pgpio;

const uint32_t p6_irq_mode[] = {
	[GPIO_IRQ_POS]		= _P6_GPIO_INTCX_POS,
	[GPIO_IRQ_NEG]		= _P6_GPIO_INTCX_NEG,
	[GPIO_IRQ_BOTH]		= _P6_GPIO_INTCX_BOTH,
	[GPIO_IRQ_LEVEL_H]	= _P6_GPIO_INTCX_LEV,
	[GPIO_IRQ_LEVEL_L]	= _P6_GPIO_INTCX_LEV|_P6_GPIO_INTCX_LOW,
};

const uint32_t p6_debounce_mode[] = {
	[GPIO_DEBOUNCE_NONE]	= 0,
	[GPIO_DEBOUNCE_NOISE]	= _P6_GPIO_INTCX_DEBOUNCE,
	[GPIO_DEBOUNCE_FILTER]	= _P6_GPIO_INTCX_DEBOUNCE|_P6_GPIO_INTCX_FILTER
};

#define p6_mode(irq_mode, debounce_mode) \
    ((p6_irq_mode[irq_mode]|p6_debounce_mode[debounce_mode]))

/* Useful macros for internal use */

#include <asm/io.h>
#include <mach/platform.h>

int gpio_set_debounce_value(uint32_t value)
{
	int ret = 0;
	unsigned long flags;

	local_irq_save(flags);
	if (value <= 31) {
		__raw_writel(value, PARROT_VA_GPIO + _P6_GPIO_DEBOUNCE);
	} else {
		ret = -EINVAL;
	}
	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(gpio_set_debounce_value);

int gpio_interrupt_register(unsigned gpio,
			    GPIO_IRQ_MODE irq_mode,
			    GPIO_DEBOUNCE_MODE debounce_mode)
{
	int src, i;
	unsigned long flags;

	/* check if gpio is already mapped */
	src = gpio_to_irq(gpio);
	if (src >= 0) {
		printk("gpio is already mapped on irq %d\n", src);
		return src;
	}
	// sanity check
	if (gpio >= GPIO_COUNT)
		return -ENODEV;
	if (irq_mode < GPIO_IRQ_POS ||
#if defined(CONFIG_ARCH_PARROT_USE_ROTATOR)
		irq_mode > GPIO_IRQ_ROTATOR_BOTH
#else
		irq_mode > GPIO_IRQ_LEVEL_L
#endif
	   )
		return -EINVAL;
	if (debounce_mode < GPIO_DEBOUNCE_NONE ||
	    debounce_mode > GPIO_DEBOUNCE_FILTER)
		return -EINVAL;

	local_irq_save(flags);

	// allocate an interrupt source
	src = -1;
	if (debounce_mode == GPIO_DEBOUNCE_NONE) {
		for (i = _P6_GPIO_NODEBOUNCE_START;
				i <= _P6_GPIO_NODEBOUNCE_END; i++ ) {
			if (!pgpio.src[i].active) {
				src = i;
				break;
			}
		}
	}

	if (src == -1) {
		int start = _P6_GPIO_DEBOUNCE_START;

#if defined(CONFIG_ARCH_PARROT_USE_ROTATOR)
		if (irq_mode <= GPIO_IRQ_LEVEL_L)
			start += 2; // the two first slots are used by the rotator
		else {
			// an "irq_mode -= 5" should do the trick but I prefer to be on the safe side
			if (irq_mode == GPIO_IRQ_ROTATOR_BOTH) {
				__raw_writel(_P6_GPIO_ROTATOR_BOTH, PARROT_VA_GPIO + _P6_GPIO_ROTATOR);
				irq_mode = GPIO_IRQ_BOTH;
			} else if (irq_mode == GPIO_IRQ_ROTATOR_POS) {
				__raw_writel(_P6_GPIO_ROTATOR_POS, PARROT_VA_GPIO + _P6_GPIO_ROTATOR);
				irq_mode = GPIO_IRQ_POS;
			} else if (irq_mode == GPIO_IRQ_ROTATOR_NEG) {
				__raw_writel(_P6_GPIO_ROTATOR_NEG, PARROT_VA_GPIO + _P6_GPIO_ROTATOR);
				irq_mode = GPIO_IRQ_NEG;
			}
		}
#endif

		for (i = start; i <= _P6_GPIO_DEBOUNCE_END; i++ ) {
			if (!pgpio.src[i].active) {
				src = i;
				break;
			}
		}
	}

	if (src >= 0) {
		pgpio.src[src].gpio = gpio;
		pgpio.src[src].active = true;

		__raw_writel(p6_mode(irq_mode, debounce_mode)|gpio,
			     PARROT_VA_GPIO + _P6_GPIO_INTC1 + src*4);
		src += IRQ_GPIO_START;
	} else {
		src = -EINVAL;
	}

	local_irq_restore(flags);

	return src;
}
EXPORT_SYMBOL(gpio_interrupt_register);

int gpio_interrupt_unregister(unsigned gpio)
{
	int i, ret = -EINVAL;
	unsigned long flags;

	if (gpio >= GPIO_COUNT)
		return -ENODEV;

	local_irq_save(flags);

	for (i = 0; i < _P6_GPIO_INTC_COUNT; i++) {
		if (pgpio.src[i].active && pgpio.src[i].gpio == gpio) {
			pgpio.src[i].active = false;
			__raw_writel(0, PARROT_VA_GPIO + _P6_GPIO_INTC1 + i*4);
			ret = 0;
			break;
		}
	}

	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(gpio_interrupt_unregister);

int gpio_to_irq(unsigned gpio)
{
	int i, ret = -EINVAL;
	unsigned long flags;

	if (gpio >= GPIO_COUNT)
		return -EINVAL;

	local_irq_save(flags);

	for (i = 0; i < _P6_GPIO_INTC_COUNT; i++) {
		if (pgpio.src[i].active && pgpio.src[i].gpio == gpio) {
			ret = i + IRQ_GPIO_START;
			break;
		}
	}

	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(gpio_to_irq);

int irq_to_gpio(unsigned irq)
{
	unsigned long flags;
	int ret = -EINVAL;
	if (irq < IRQ_GPIO_START || irq >= IRQ_GPIO_START + IRQ_GPIO_END)
		return -EINVAL;

	local_irq_save(flags);
	if (pgpio.src[irq-IRQ_GPIO_START].active)
		ret = pgpio.src[irq-IRQ_GPIO_START].gpio;
	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(irq_to_gpio);

static void gpio_unmask_interrupt(unsigned int src)
{
	void __iomem *addr;
	u32 reg;

	addr = (void __iomem *) PARROT_VA_GPIO + _P6_GPIO_INTC1 + src*4;
	reg = __raw_readl(addr);
	__raw_writel(reg | _P6_GPIO_INTCX_ENA, addr);
}

static void gpio_mask_interrupt(unsigned int src)
{
	void __iomem *addr;
	u32 reg;

	addr = (void __iomem *) PARROT_VA_GPIO + _P6_GPIO_INTC1 + src*4;
	reg = __raw_readl(addr);
	__raw_writel(reg & ~_P6_GPIO_INTCX_ENA, addr);
}

static void gpio_ack_interrupt(unsigned int src)
{
	void __iomem *addr;
	u32 reg;

	addr = (void __iomem *) PARROT_VA_GPIO + _P6_GPIO_INTC1 + src*4;
	reg = __raw_readl(addr);
	__raw_writel(reg | _P6_GPIO_INTCX_CLR, addr);
	__raw_writel(reg & ~_P6_GPIO_INTCX_CLR, addr);
}

enum ROTATOR_EVENT_TAG rotator_get_status(unsigned int rot)
{
	enum ROTATOR_EVENT_TAG ret;

	if (pgpio.rot_status & (1<<rot)) {
		ret = ROTATOR_EVENT_RIGHT;
	} else {
		ret = ROTATOR_EVENT_LEFT;
	}
	return ret;
}
EXPORT_SYMBOL(rotator_get_status);

static void gpio_handle_virq(unsigned int irq, struct irq_desc *desc)
{
	uint32_t status;

	status =  __raw_readl(PARROT_VA_GPIO + _P6_GPIO_STATUS);

	if (!status)
		return;

	do {
		irq = ffs(status) -1;
		status &= ~(1 << irq);

		irq += IRQ_GPIO_START;
		desc = irq_desc + irq;
		desc_handle_irq(irq, desc);

	} while (status);
}

static void gpio_irq_ack(unsigned int irq)
{
	int id;

	if (irq <= IRQ_GPIO_END) {
		id = irq - IRQ_GPIO_START;
		gpio_ack_interrupt(id);
	} else {
		id = irq - IRQ_ROTATOR_START;
		pgpio.rot_status = __raw_readl(PARROT_VA_GPIO + _P6_GPIO_ROTATOR_STATUS);
		gpio_ack_interrupt(2*id);
		gpio_ack_interrupt(2*id+1);
	}
}

static void gpio_irq_mask(unsigned int irq)
{
	int id;

	if (irq <= IRQ_GPIO_END) {
		id = irq - IRQ_GPIO_START;
		gpio_mask_interrupt(id);
	} else {
		id = irq - IRQ_ROTATOR_START;
		pgpio.rot_status = __raw_readl(PARROT_VA_GPIO + _P6_GPIO_ROTATOR_STATUS);
		gpio_mask_interrupt(2*id);
		gpio_mask_interrupt(2*id+1);
	}
}

static void gpio_irq_unmask(unsigned int irq)
{
	int id;

	if (irq <= IRQ_GPIO_END) {
		id = irq - IRQ_GPIO_START;
		gpio_unmask_interrupt(id);
	} else {
		id = irq - IRQ_ROTATOR_START;
		gpio_unmask_interrupt(2*id);
		gpio_unmask_interrupt(2*id+1);
	}
}

static int
gpio_irq_type(unsigned int irq, unsigned int type)
{
	void __iomem *addr;
	u32 newvalue = 0;
	int gpio, src;
	unsigned long flags;
	/* Set the external interrupt to pointed trigger type */
	switch (type)
	{
		case IRQ_TYPE_NONE:
			printk(KERN_WARNING "No edge setting!\n");
			break;

		case IRQ_TYPE_EDGE_RISING:
			newvalue = _P6_GPIO_INTCX_POS;
			break;

		case IRQ_TYPE_EDGE_FALLING:
			newvalue = _P6_GPIO_INTCX_NEG;
			break;

		case IRQ_TYPE_EDGE_BOTH:
			newvalue = _P6_GPIO_INTCX_BOTH;
			break;

		case IRQ_TYPE_LEVEL_LOW:
			newvalue = _P6_GPIO_INTCX_LEV|_P6_GPIO_INTCX_LOW;
			break;

		case IRQ_TYPE_LEVEL_HIGH:
			newvalue = _P6_GPIO_INTCX_LEV;
			break;

		default:
			printk(KERN_ERR "No such irq type %d", type);
			return -1;
	}
	gpio = irq_to_gpio(irq);
	if (gpio < 0)
		return -1;
	src = irq - IRQ_GPIO_START;
	local_irq_save(flags);
	addr = (void __iomem *) PARROT_VA_GPIO + _P6_GPIO_INTC1 + src*4;
	newvalue |= __raw_readl(addr) & ~(0x70000);
	__raw_writel(newvalue, addr);
	local_irq_restore(flags);
	WARN_ON(gpio!=(newvalue&0xff));

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__set_irq_handler_unlocked(irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__set_irq_handler_unlocked(irq, handle_edge_irq);

	return 0;
}

static struct irq_chip vchip = {
	.name		= "p6-gpio",
	.ack		= gpio_irq_ack,
	.mask		= gpio_irq_mask,
	.unmask		= gpio_irq_unmask,
	.set_type	= gpio_irq_type,
};

/* Select GPIO multiplexing */
static inline u32 parrot_gpio_to_pin(unsigned gpio)
{
	u32 iocr = GPIO_IOCR(gpio);
	if (parrot_chip_is_p6i()) {
		/* rtc gpio */
		if (gpio >= 1 && gpio <= 12) {
			iocr = 0x19006C;
			gpio--;
		}
	}
	return (iocr << 8)|((gpio & 0xf) << 4)|GPIO_MODE;
}

static inline int parrot_gpio_select(unsigned gpio)
{
#if !defined(CONFIG_ARCH_VERSATILE)
	u32 pin = parrot_gpio_to_pin(gpio);
	if (!parrot_is_select_pin(pin)) {
		printk(KERN_INFO "gpio %d is not selected.\n" 
				"Please do it in your board code\n", gpio);
		/*XXX remove this when everybody select gpio pin in board code */
		/* for p6i gpio should be selected ... */
#if 1
		if ((parrot_chip_is_p6() && gpio < 34) ||
			 parrot_chip_is_p6i()) {
			printk(KERN_ERR "not auto selecting this critical pin\n");
			return 1;
		}
		parrot_select_pin(pin);
#else
		return 1;
#endif
	}
#endif
	return 0;
}

static inline int p6_gpio_direction_input(unsigned gpio)
{
	int ret = 0;
	u32 val, ddr;
	unsigned long flags;

	local_irq_save(flags);

	if (parrot_gpio_select(gpio)) {
		ret = -ENODEV;
		goto exit;
	}

	ddr = PARROT_VA_GPIO+__DDR(gpio);
	val = __raw_readl(ddr) & ~(1 << __DDR_SHIFT(gpio));
	__raw_writel(val, ddr);

exit:
	local_irq_restore(flags);

	return ret;
}

static inline int p6_gpio_direction_output(unsigned gpio, int value)
{
	int ret = 0;
	u32 val, ddr;
	unsigned long flags;

	local_irq_save(flags);

	if (parrot_gpio_select(gpio)) {
		ret = -ENODEV;
		goto exit;
	}

	ddr = PARROT_VA_GPIO+__DDR(gpio);
	val = __raw_readl(ddr) | (1 << __DDR_SHIFT(gpio));
	__raw_writel(val, ddr);

	/* gpio is now in output mode; if it was previously in input mode,
	   its present value is the same as before its direction changed */

	/* now set new value */
	gpio_set_value(gpio, value);

exit:
	local_irq_restore(flags);

	return ret;
}

#ifdef CONFIG_GPIOLIB
static int gpio_direction_input_(struct gpio_chip *chip, unsigned gpio)
{
	return p6_gpio_direction_input(gpio);
}

static int gpio_direction_output_(struct gpio_chip *chip, unsigned gpio, int value)
{
	return p6_gpio_direction_output(gpio, value);
}

static void gpio_set_(struct gpio_chip *chip, unsigned offset, int value)
{
	gpio_set_value(offset, value);
}

static int gpio_get_(struct gpio_chip *chip, unsigned offset)
{
	return gpio_get_value(offset);
}

static struct gpio_chip gchip = {
	.direction_input = gpio_direction_input_,
	.direction_output = gpio_direction_output_,
	.set = gpio_set_,
	.get = gpio_get_,
	.base = 0,
	.ngpio = 0,
	.can_sleep = 0,
};
#else
int gpio_direction_input(unsigned gpio)
{
	return p6_gpio_direction_input(gpio);
}
EXPORT_SYMBOL(gpio_direction_input);

int gpio_direction_output(unsigned gpio, int value)
{
	return p6_gpio_direction_output(gpio, value);
}
EXPORT_SYMBOL(gpio_direction_output);
#endif

static int __init parrot6_gpio_init(void)
{
	int i;

	printk(KERN_INFO "Parrot6 GPIO driver $Revision: 1.7 $\n");

	set_irq_chained_handler(IRQ_GPIO, gpio_handle_virq);
	for (i = IRQ_GPIO_START; i <= IRQ_ROTATOR_END; i++) {
		set_irq_chip(i, &vchip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}

	for (i = 0; i < _P6_GPIO_INTC_COUNT; i++)
		pgpio.src[i].active = 0;

#ifdef CONFIG_GPIOLIB
	gchip.ngpio = GPIO_COUNT;
	gpiochip_add(&gchip);
#endif

	return 0;
}

core_initcall(parrot6_gpio_init);
#endif
