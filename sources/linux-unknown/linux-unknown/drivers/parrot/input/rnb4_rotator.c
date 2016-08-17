/**
 * @file   rnb4_rotator.c
 * @brief  Kernel rotator driver for Rnb4 (P6)
 *
 * @author francois.muller@parrot.com
 * @date   2010-03-01
 *
 * Copyright (C) 2010 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/time.h>

#include <mach/gpio.h>
#include <mach/gpio_parrot.h>

#define DRIVER_NAME "rnb4_rotator"
#define DRIVER_VERSION 0x001

#define NB_ROT_INT 2

struct rnb4_rot_input {
    struct input_dev	*input;		//!< input layer private data
    int irq[NB_ROT_INT];
    int gpio[NB_ROT_INT];
};

/* button isr */
static irqreturn_t rnb4_rot_input_irq(int irq, void *dev_id)
{
    struct rnb4_rot_input *rnb4_rot_input = dev_id;

#if defined(CONFIG_ARCH_PARROT_USE_ROTATOR)

    if(irq == IRQ_ROTATOR_START)
    {
    	if(rotator_get_status(0) == ROTATOR_EVENT_LEFT)
    	{
			input_report_key(rnb4_rot_input->input, KEY_DOWN, 1);
			input_report_key(rnb4_rot_input->input, KEY_DOWN, 0);
			input_sync(rnb4_rot_input->input);
    	}
    	else
    	{
			input_report_key(rnb4_rot_input->input, KEY_UP, 1);
			input_report_key(rnb4_rot_input->input, KEY_UP, 0);
			input_sync(rnb4_rot_input->input);
    	}
    }

#else //defined(CONFIG_ARCH_PARROT_USE_ROTATOR)

/***********************************************************
 *  software management of a 4 state rotator:
 *  - wait for each state to occur before sending the event
 *  - if an unexpected event occur, return in a wait for the first state
 *  (in this case a rnb4_rot:ERR will be issued)
 ***********************************************************/

    int button = irq - IRQ_GPIO_START, cur_dir;
    static int last = 0xFF, direction = 0xFF, valid = 0xFF;

    if(irq == rnb4_rot_input->irq[0])
    	cur_dir = gpio_get_value(rnb4_rot_input->gpio[0]);
    else
    	cur_dir = gpio_get_value(rnb4_rot_input->gpio[1]);

    if(cur_dir > 0)
    	cur_dir = 1;

    if(valid == 0xFF)
    {
    	last = button;	direction = cur_dir;	valid = 0;
    }
    else if (valid == 0)
    { // change of irq and same gpio level
    	if(last != button && direction == cur_dir)
    	{
    		valid = 1;
    	}
    	else
    	{ // restart from here
    		printk("rnb4_rot:ERR\n");
    		last = button;	direction = cur_dir;	valid = 0;
    	}
    }
    else if (valid == 1)
    { // same  irq and different gpio level
    	if(last == button && direction != cur_dir)
    	{
    		valid = 2;
    	}
    	else
    	{ // restart from here
    		last = button;	direction = cur_dir;	valid = 0;
    		printk("rnb4_rot:ERR\n");
    	}
    }
    else
    {// different irq and same gpio level
    	if(last != button && direction != cur_dir)
    	{
        	if(last > button)
        	{
    			input_report_key(rnb4_rot_input->input, KEY_UP, 1);
    			input_report_key(rnb4_rot_input->input, KEY_UP, 0);
    			input_sync(rnb4_rot_input->input);
        	}
        	else
        	{
    			input_report_key(rnb4_rot_input->input, KEY_DOWN, 1);
    			input_report_key(rnb4_rot_input->input, KEY_DOWN, 0);
    			input_sync(rnb4_rot_input->input);
        	}
        	last = 0xFF;
        	direction = 0xFF;
        	valid = 0xFF;
    	}
    	else
    	{ // restart from here
    		printk("rnb4_rot:ERR\n");
    		last = button;	direction = cur_dir;	valid = 0;
    	}
    }

#endif //defined(ARCH_PARROT_USE_ROTATOR)

    return IRQ_HANDLED;
}

/* add a button to the input layer and configure it in interrupt mode
 * with noise filter
 */
static int add_button(int gpio, int keycode, struct rnb4_rot_input *p6_kbd)
{
	int debounced = GPIO_DEBOUNCE_FILTER;
    int irq, err = -1;

    gpio_direction_input(gpio);
#if defined(CONFIG_ARCH_PARROT_USE_ROTATOR)
    irq = gpio_interrupt_register(gpio, GPIO_IRQ_ROTATOR_NEG, debounced);
#else
    irq = gpio_interrupt_register(gpio, GPIO_IRQ_BOTH, debounced);
#endif

    if (irq >= IRQ_GPIO_START)
    {
    	set_bit(keycode, p6_kbd->input->keybit);
#if defined(CONFIG_ARCH_PARROT_USE_ROTATOR)
    	if(irq == IRQ_GPIO_START + 1) // wait for the second gpio to register the only irq
    	{
    	    err = request_irq(IRQ_ROTATOR_START, rnb4_rot_input_irq, IRQF_TRIGGER_NONE, DRIVER_NAME, p6_kbd);
    	}
#else
    	err = request_irq(irq, rnb4_rot_input_irq, IRQF_TRIGGER_NONE, DRIVER_NAME, p6_kbd);

        if(p6_kbd->irq[0] == 0xFF)
        {
        	p6_kbd->irq[0] = irq;	p6_kbd->gpio[0] = gpio;
        }
        else
        {
        	p6_kbd->irq[1] = irq;	p6_kbd->gpio[1] = gpio;
        }
#endif
    }

    return err;
}

static void clean_irq_and_input(struct rnb4_rot_input *rnb4_rot_input)
{
    free_irq(rnb4_rot_input->irq[0], rnb4_rot_input);
    gpio_interrupt_unregister(rnb4_rot_input->gpio[0]);
    free_irq(rnb4_rot_input->irq[1], rnb4_rot_input);
    gpio_interrupt_unregister(rnb4_rot_input->gpio[1]);

    input_unregister_device(rnb4_rot_input->input);
    input_free_device(rnb4_rot_input->input);
}

static int rnb4_rotator_probe(struct platform_device *pdev)
{
    struct rnb4_rot_input *rnb4_rot_input;
    struct input_dev *input_dev;
    int err = 0;

    rnb4_rot_input = kzalloc(sizeof(struct rnb4_rot_input), GFP_KERNEL);

    if (!rnb4_rot_input) {
        return -ENOMEM;
    }

    input_dev = input_allocate_device();

    if (!input_dev) {
        err = -ENOMEM;
        goto input_failure;
    }

    /* input layer init */
    rnb4_rot_input->input = input_dev;

    rnb4_rot_input->irq[0] = 0xFF; rnb4_rot_input->gpio[0] = 0xFF;
    rnb4_rot_input->irq[1] = 0xFF; rnb4_rot_input->gpio[1] = 0xFF;

    input_dev->name = DRIVER_NAME;
    input_dev->phys = DRIVER_NAME"/input0";

    input_dev->id.bustype = BUS_HOST;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0001;
    input_dev->id.version = DRIVER_VERSION;
    input_dev->evbit[0] = BIT(EV_KEY);

    add_button(118, KEY_UP, rnb4_rot_input);
    add_button(119, KEY_DOWN, rnb4_rot_input);

    err = input_register_device(rnb4_rot_input->input);

    if (err)
        goto no_enter_irq;

    platform_set_drvdata(pdev, rnb4_rot_input);
    dev_info(&pdev->dev, "driver loaded\n");

    goto ok;

no_enter_irq:
    clean_irq_and_input(rnb4_rot_input);
input_failure:
    kfree(rnb4_rot_input);
ok:
    return err;
}

static int rnb4_rotator_remove(struct platform_device *pdev)
{
    struct rnb4_rot_input *rnb4_rot_input = platform_get_drvdata(pdev);

    clean_irq_and_input(rnb4_rot_input);
    platform_set_drvdata(pdev, NULL);
    kfree(rnb4_rot_input);
    dev_info(&pdev->dev, "driver removed\n");
    return 0;
}

static struct platform_driver rnb4_rotator = {
    .probe      = rnb4_rotator_probe,
    .remove     = rnb4_rotator_remove,
    .driver     = {.name = "rnb4_rotator"},
};

static int __devinit rnb4_rotator_init(void)
{
    return platform_driver_register(&rnb4_rotator);
}

static void __exit rnb4_rotator_exit(void)
{
    platform_driver_unregister(&rnb4_rotator);
}

module_init(rnb4_rotator_init);
module_exit(rnb4_rotator_exit);


MODULE_AUTHOR("Francois MULLER <francois.muller@parrot.com>");
MODULE_DESCRIPTION("Rnb4 Rotator Driver");
MODULE_LICENSE("GPL");
