/**
********************************************************************************
* @file gpio.c
* @brief gpio generic driver
*
* Copyright (C) 2007 Parrot S.A.
*
* @author     Matthieu CASTET <matthieu.castet@parrot.com>
* @date       2007-06-20
********************************************************************************
*/

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sched.h>

/* generic gpio API inclide */
#include <asm/gpio.h>

#include "gpio2_ioctl.h"

/* our whitelist gpio include */
#define GPIO_DRIVER_NAME "gpio driver"

#define GPIO_IRQ_MODE
#ifdef GPIO_IRQ_MODE
#include <linux/completion.h>
#include <linux/interrupt.h>
struct gpio_info
{
	int irq_pin;
	struct completion completion;
};

irqreturn_t gpio_handler(int irq, void *data)
{
		struct gpio_info *info = data;
		complete(&info->completion);
		return IRQ_HANDLED;
}
#endif

static int *allowed_pins;
static int validate_gpio(int pin, int is_out)
{
	int *pins = allowed_pins;
	if (!pins)
		return 1;

	while (*pins != -1) {
		if (*pins == pin)
			return 1;
		pins++;
	}
	printk(KERN_DEBUG "%s(%d:%d)gpio2 %d is filtered dir %s\n",
			current->comm, current->pid, current->tgid,
			pin, is_out?"out":"in");

	return 0;
}
static int gpio_open(struct inode *inode, struct file *filp)
{

	filp->private_data = NULL;
#ifdef GPIO_IRQ_MODE
	{
		struct gpio_info *info;
		info = kzalloc(sizeof(struct gpio_info), GFP_KERNEL);
		if (!info) {
			return -ENOMEM;
		}
		filp->private_data = info;
	}
#endif
	return 0;
}

static int gpio_release(struct inode *inode, struct file *filp)
{
#ifdef GPIO_IRQ_MODE
	struct gpio_info *info = (struct gpio_info *)filp->private_data;

	if (info->irq_pin) {
		free_irq(gpio_to_irq(info->irq_pin), info);
		gpio_free(info->irq_pin);
		info->irq_pin = 0;
	}
	kfree(info);
#endif
	filp->private_data = NULL;
	return 0;
}



static long gpio_ioctl(struct file *filp,
                       unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd)
	{
		case GPIO_DIRECTION:
			{
				struct gpio_direction dir;
				if (copy_from_user(&dir, (void __user *)arg, sizeof(dir))) {
					ret = -EFAULT;
					break;
				}
				if (validate_gpio(dir.pin, 1) == 0) {
					ret = -EPERM;
					break;
				}
#if 0
				if (gpio_request(dir.pin, GPIO_DRIVER_NAME) != 0) {
					ret = -EBUSY;
					break;
				}
#endif
				switch (dir.mode) {
					case GPIO_INPUT:
						ret = gpio_direction_input(dir.pin);
						break;
					case GPIO_OUTPUT_LOW:
						printk(KERN_DEBUG "%s(%d:%d)gpio2_direction_output(%d,0)\n",
								current->comm, current->pid, current->tgid,
								dir.pin);
						ret = gpio_direction_output(dir.pin, 0);
						break;
					case GPIO_OUTPUT_HIGH:
						printk(KERN_DEBUG "%s(%d:%d)gpio2_direction_output(%d,1)\n",
								current->comm, current->pid, current->tgid,
								dir.pin);
						ret = gpio_direction_output(dir.pin, 1);
						break;
				}
				//gpio_free(dir.pin);
			}
			break;
		case GPIO_READ:
			{
				struct gpio_data data;
				if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
					ret = -EFAULT;
					break;
				}

				if (validate_gpio(data.pin, 0) == 0) {
					ret = -EPERM;
					break;
				}
				/*
				if (gpio_request(data.pin, GPIO_DRIVER_NAME) != 0) {
					ret = -EBUSY;
					break;
				}*/
				data.value = gpio_get_value(data.pin);
				//gpio_free(data.pin);
				if (copy_to_user((void __user *)arg, &data, sizeof(data))) {
					ret = -EFAULT;
					break;
				}
			}
			break;
		case GPIO_WRITE:
			{
				struct gpio_data data;
				if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
					ret = -EFAULT;
					break;
				}
				if (validate_gpio(data.pin, 1) == 0) {
					ret = -EPERM;
					break;
				}

				/*
				if (gpio_request(data.pin, GPIO_DRIVER_NAME) != 0) {
					ret = -EBUSY;
					break;
				}
				*/
				
				gpio_set_value(data.pin, data.value);
				//gpio_free(data.pin);
			}
			break;
#ifdef GPIO_IRQ_MODE
			/* XXX need a mutex */
        case GPIO_IRQ_INIT:
			{
				struct gpio_irq data;
				struct gpio_info *info = (struct gpio_info *)filp->private_data;
				int flags = 0;

				/* XXX pin 0 may be valid */
				if (info->irq_pin) {
					ret = -EBUSY;
					break;
				}
				if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
					ret = -EFAULT;
					break;
				}
				if (data.mode < 0 || data.mode > GPIO_IRQ_TYPE_LEVEL_LOW) {
					ret = -EINVAL;
					break;
				}
				switch (data.mode) {
					case GPIO_IRQ_TYPE_EDGE_RISING:
						flags = IRQF_TRIGGER_RISING;
						break;
					case GPIO_IRQ_TYPE_EDGE_FALLING:
						flags = IRQF_TRIGGER_FALLING;
						break;
					case GPIO_IRQ_TYPE_EDGE_BOTH:
						flags = IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING;
						break;
#if 0
					/* not supported because we don't ack IRQ in handler */
					case GPIO_IRQ_TYPE_LEVEL_HIGH:
						flags = IRQF_TRIGGER_HIGH;
						break;
					case GPIO_IRQ_TYPE_LEVEL_LOW:
						flags = IRQF_TRIGGER_LOW;
						break;
#endif
					default:
					ret = -EINVAL;
				}
				if (ret)
					break;

				/* if (gpio_request(data.pin, GPIO_DRIVER_NAME) != 0) {
					ret = -EBUSY;
					break;
					}*/

				/* struct completion */
				init_completion(&info->completion);

				if (request_irq(gpio_to_irq(data.pin), gpio_handler, flags, "gpio", info)) {
					ret = -EBUSY;
					printk(KERN_ERR "request irq failed, may be irq wasn't registered with in board code\n");
					//gpio_free(data.pin);
					break;
				}
				info->irq_pin = data.pin;
			}
			break;
		case GPIO_IRQ_FREE:
			{
				struct gpio_irq data;
				struct gpio_info *info = (struct gpio_info *)filp->private_data;

				if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
					ret = -EFAULT;
					break;
				}
				/* XXX pin 0 may be valid */
				if (!info->irq_pin || info->irq_pin != data.pin) {
					ret = -EBUSY;
					break;
				}

				free_irq(gpio_to_irq(data.pin), info);
				gpio_free(data.pin);
				info->irq_pin = 0;
			}
			break;
		case GPIO_IRQ_WAIT:
			{
				struct gpio_irq data;
				struct gpio_info *info = (struct gpio_info *)filp->private_data;

				if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
					ret = -EFAULT;
					break;
				}
				/* XXX pin 0 may be valid */
				if (!info->irq_pin || info->irq_pin != data.pin) {
					ret = -EBUSY;
					break;
				}
				wait_for_completion_interruptible(&info->completion);
				init_completion(&info->completion);
				//INIT_COMPLETION(info->completion);
			}
			break;
#endif
		default:
			ret = -ENOTTY;
	}
	return ret;
}

struct file_operations gpio_fops = {
	.unlocked_ioctl = gpio_ioctl,
	.open           = gpio_open,
	.release        = gpio_release,
};

static struct miscdevice gpio_miscdev = {
	.minor = 64 + 2,
	.name  = "gpio",
	.fops  = &gpio_fops,
};

static int init_device(void)
{
	return misc_register(&gpio_miscdev);
}

static int gpio_probe(struct platform_device *pdev)
{
    int err = 0;
	allowed_pins = pdev->dev.platform_data;
	if (!allowed_pins)
		printk(KERN_INFO "gpio2 : no filter\n");

	err = init_device();
	if (err < 0)
		goto err;

	return 0;
err:
	return err;
}

static int gpio_remove(struct platform_device *pdev)
{
	/* we are protected by module refcount.
	 * this call is possible only if there is no user
	 */
	misc_deregister(&gpio_miscdev);
	dev_info(&pdev->dev, "driver removed\n");

	return 0;
}

static struct platform_driver gpio_driver = {
	.probe		= gpio_probe,
	.remove		= gpio_remove,
#if 0
	.suspend	= gpio_suspend,
	.resume		= gpio_resume,
#endif
	.driver		= {
		.name	= "gpio",
	},
};

static int __devinit gpio_init(void)
{
	return platform_driver_register(&gpio_driver);
}

static void __exit gpio_exit(void)
{
	platform_driver_unregister(&gpio_driver);
}

module_init(gpio_init);
module_exit(gpio_exit);


MODULE_AUTHOR("PARROT SA by Matthieu CASTET <matthieu.castet@parrot.com>");
MODULE_DESCRIPTION("gpio driver");
MODULE_LICENSE("GPL");
