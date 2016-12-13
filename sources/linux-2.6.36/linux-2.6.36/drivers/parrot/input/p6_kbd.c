/**
 * @file   p6_kbd.c
 * @brief  Kernel button driver for P6
 *
 * @author olivier.bienvenu@parrot.com
 * @date   2009-06-11
 *
 * Copyright (C) 2009 Parrot S.A.
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
#include <mach/p6_kbd.h>

#define DRIVER_NAME "p6_kbd_input"
#define DRIVER_VERSION 0x001

#define MAX_GPIO_INT 20

struct p6_kbd_input {
    struct input_dev                *input;                         //!< input layer private data
    struct p6_platform_kbd_input    kbd_input[MAX_GPIO_INT];
    int                             counter[MAX_GPIO_INT];          //!< a counter
    suseconds_t                     press_time[MAX_GPIO_INT];       //!< a time for checking with max_pressed_time
};

//!< gpio state
enum state {
    STATE_RELEASED,
    STATE_PRESSED,
};

static void reset_button_counter(struct p6_kbd_input *p6_kbd_input)
{
    int i;

    for (i = 0; i < MAX_GPIO_INT; i++)
        p6_kbd_input->counter[i] = 0;
}

/* button isr */
static irqreturn_t p6_kbd_input_irq(int irq, void *dev_id)
{
    struct p6_kbd_input *p6_kbd_input = dev_id;
    int button = irq - IRQ_GPIO_START;
    struct p6_platform_kbd_input *kbd = &p6_kbd_input->kbd_input[button];
    int state;

    dev_dbg(&(p6_kbd_input->input->dev),"irq %d UP\n", button);

    if (kbd->inverted == 1)
        state = gpio_get_value(kbd->gpio) ? STATE_RELEASED : STATE_PRESSED;
    else
        state = gpio_get_value(kbd->gpio) ? STATE_PRESSED : STATE_RELEASED;

    if (kbd->max_pressed_time == 0) {
        if (state == STATE_PRESSED) {
            dev_dbg(&(p6_kbd_input->input->dev), "Button Down %d\n", p6_kbd_input->counter[button]);
            if (p6_kbd_input->counter[button] >= kbd->delay) {
            //Key Pressed Event
                input_report_key(p6_kbd_input->input, kbd->keycode, 1);
                input_sync(p6_kbd_input->input);

                if (kbd->no_long_press) {
                //Key Released Event
                    input_report_key(p6_kbd_input->input, kbd->keycode, 0);
                    input_sync(p6_kbd_input->input);
                    reset_button_counter(p6_kbd_input);
                }
            }
        }
        else
        {
            dev_dbg(&(p6_kbd_input->input->dev), "Button Up %d\n", p6_kbd_input->counter[button]);
            if (p6_kbd_input->counter[button] >= kbd->delay) {
                if (!kbd->no_long_press) {
                //Key Released Event
                    input_report_key(p6_kbd_input->input, kbd->keycode, 0);
                    input_sync(p6_kbd_input->input);
                    reset_button_counter(p6_kbd_input);
                }
                else {
                    //Event already sent
                }
            }
            else {
                p6_kbd_input->counter[button]++;
            }
        }
    }
    else {
        struct timeval tv;
        suseconds_t current_time;
        do_gettimeofday(&tv);
        
        current_time = tv.tv_sec * USEC_PER_SEC + tv.tv_usec;
    
        if (state == STATE_PRESSED) {
            dev_dbg(&(p6_kbd_input->input->dev), "Button Down %ld \n", current_time);
            p6_kbd_input->press_time[button] = current_time;
        }
        else
        {
            dev_dbg(&(p6_kbd_input->input->dev), "Button Up %ld \n", current_time);
            if (current_time - p6_kbd_input->press_time[button] < kbd->max_pressed_time) {
            //Key Pressed Event
                input_report_key(p6_kbd_input->input, kbd->keycode, 1);
                input_sync(p6_kbd_input->input);

            //Key Released Event
                input_report_key(p6_kbd_input->input, kbd->keycode, 0);
                input_sync(p6_kbd_input->input);

                reset_button_counter(p6_kbd_input);
            }
        }
    }

    return IRQ_HANDLED;
}

/* add a button to the input layer and configure it in interrupt mode
 * with noise filter
 */
static int add_button(struct p6_kbd_input *p6_kbd, struct p6_platform_kbd_input *kbd)
{
    int irq, err = -1;

    gpio_direction_input(kbd->gpio);

    irq = gpio_interrupt_register(kbd->gpio, GPIO_IRQ_BOTH, kbd->debounced);

    if (irq >= 0) {
        memcpy(&p6_kbd->kbd_input[irq - IRQ_GPIO_START], kbd, sizeof(struct p6_platform_kbd_input));
        p6_kbd->counter[irq - IRQ_GPIO_START] = 0;
        p6_kbd->press_time[irq - IRQ_GPIO_START] = 0;
        err = request_irq(irq, p6_kbd_input_irq, IRQF_TRIGGER_NONE, DRIVER_NAME, p6_kbd);
        set_bit(kbd->keycode, p6_kbd->input->keybit);
    }

    return err;
}

static void clean_irq_and_input(struct p6_kbd_input *p6_kbd_input)
{
    int it;

    /* release all irq */
    for (it = 0; it <MAX_GPIO_INT; it++) {
        int gpio = p6_kbd_input->kbd_input[it].gpio;
        if (gpio) {
            free_irq(it + IRQ_GPIO_START, p6_kbd_input);
            gpio_interrupt_unregister(gpio);
        }
    }

    input_unregister_device(p6_kbd_input->input);
    input_free_device(p6_kbd_input->input);
}

static int p6_input_probe(struct platform_device *pdev)
{
    struct p6_kbd_input *p6_kbd_input;
    struct input_dev *input_dev;
    struct p6_platform_kbd_input *p6_kbd_input_info = pdev->dev.platform_data;
    int err = 0;

    if (!p6_kbd_input_info) {
        return -EINVAL;
    }

    p6_kbd_input = kzalloc(sizeof(struct p6_kbd_input), GFP_KERNEL);

    if (!p6_kbd_input) {
        return -ENOMEM;
    }

    input_dev = input_allocate_device();

    if (!input_dev) {
        err = -ENOMEM;
        goto input_failure;
    }

    /* input layer init */
    p6_kbd_input->input = input_dev;
    input_dev->name = DRIVER_NAME;
    input_dev->phys = DRIVER_NAME"/input0";

    input_dev->id.bustype = BUS_HOST;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0001;
    input_dev->id.version = DRIVER_VERSION;
    input_dev->evbit[0] = BIT(EV_KEY);
    input_dev->evbit[0] |= BIT(EV_REP);

    gpio_set_debounce_value(22); // 26.9ms debounce dead time

    while (p6_kbd_input_info->keycode) {
        err = add_button(p6_kbd_input, p6_kbd_input_info);
        if (err)
            goto no_enter_irq;
        p6_kbd_input_info++;
    }

    err = input_register_device(p6_kbd_input->input);

    if (err)
        goto no_enter_irq;

    platform_set_drvdata(pdev, p6_kbd_input);
    dev_info(&pdev->dev, "driver loaded\n");

    goto ok;

no_enter_irq:
    clean_irq_and_input(p6_kbd_input);
input_failure:
    kfree(p6_kbd_input);
ok:
    return err;
}

static int p6_input_remove(struct platform_device *pdev)
{
    struct p6_kbd_input *p6_kbd_input = platform_get_drvdata(pdev);

    clean_irq_and_input(p6_kbd_input);
    platform_set_drvdata(pdev, NULL);
    kfree(p6_kbd_input);
    dev_info(&pdev->dev, "driver removed\n");
    return 0;
}

static struct platform_driver p6_input_driver = {
    .probe      = p6_input_probe,
    .remove     = p6_input_remove,
    .driver     = {.name = "p6_kbd_input"},
};

static int __devinit p6_input_init(void)
{
    return platform_driver_register(&p6_input_driver);
}

static void __exit p6_input_exit(void)
{
    platform_driver_unregister(&p6_input_driver);
}

module_init(p6_input_init);
module_exit(p6_input_exit);


MODULE_AUTHOR("Olivier BIENVENU <olivier.bienvenu@parrot.com>");
MODULE_DESCRIPTION("P6 Keyboard Driver");
MODULE_LICENSE("GPL");
