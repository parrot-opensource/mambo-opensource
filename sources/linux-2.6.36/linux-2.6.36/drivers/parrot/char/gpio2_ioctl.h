/**
********************************************************************************
* @file gpio_ioctl.h
* @brief gpio ioctl
*
* Copyright (C) 2007 Parrot S.A.
*
* @author     Matthieu CASTET <matthieu.castet@parrot.com>
* @date       2007-06-20
********************************************************************************
*/

/**
 * \mainpage 
 *
 * This is a gpio interface for userspace application.
 *
 * With this API userspace application can configure, and read/write on gpio
 * pins. There no support for interrupt mode ATM.
 *
 * The configuration is done via ioctl on a file descriptor, and then read/write
 * is done by the standard function read(2)/write(2). This means you need a
 * file descriptor per gpio.
 *
 * Userspace can't access all gpio for security reason, there is a whitelist
 * in the kernel, that contains the list of allowed gpio and what mode can be
 * used (read and/or write).
 *
 * On the kernel side it relies on the generic gpio API
 * (see Documentation/gpio.txt). This interface should work on any
 * plaform without needed to add extra arch depend code.
 * But not that the pin numbers are platform dependant : there is the
 * same number as kernel API.
 *
 * See gpio_ioctl.h for the API documentation.
 *
 * There is a example valid_gpio.c that can be found in the
 * example section.
 * \example valid_gpio.c
 */

#ifndef _GPIO_IOCTL_H
#define _GPIO_IOCTL_H 1

#include <asm/ioctl.h>

enum gpio_mode {
	GPIO_INPUT = 0,				//!< Pin configured for input
	GPIO_OUTPUT_LOW,			//!< Pin configured for output with low level
	GPIO_OUTPUT_HIGH,			//!< Pin configured for output with high level
};

enum gpio_irq_mode {
	GPIO_IRQ_TYPE_EDGE_RISING = 0,
	GPIO_IRQ_TYPE_EDGE_FALLING = 1,
	GPIO_IRQ_TYPE_EDGE_BOTH = 2,
	GPIO_IRQ_TYPE_LEVEL_HIGH = 3,
	GPIO_IRQ_TYPE_LEVEL_LOW = 4,
};

struct gpio_direction {
	int pin;
	enum gpio_mode mode;
};

struct gpio_data {
	int pin;
	int value;
};

struct gpio_irq {
	int pin;
	enum gpio_irq_mode mode;
};

#define GPIO_MAGIC 'p'
/** GPIO_DIRECTION
 * select the gpio pin to use
 *
 * @param pin (in int)
 * @see ioctl_gpio_request
 */
#define GPIO_DIRECTION _IOW(GPIO_MAGIC, 0, struct gpio_direction)
/** GPIO_READ
 * release the gpio in use
 *
 * @param pin (in int)
 * @see ioctl_gpio_release
 */
#define GPIO_READ _IOWR(GPIO_MAGIC, 1, struct gpio_data)
/** GPIO_WRITE
 * set the pin mode as output
 *
 * This allow to read/write the pin data with configuring the pin as output
 *
 * @param val (in int) init value
 * @see ioctl_gpio_direction_output
 */
#define GPIO_WRITE _IOW(GPIO_MAGIC, 2, struct gpio_data)
#define GPIO_IRQ_INIT _IOW(GPIO_MAGIC, 3, struct gpio_irq)
#define GPIO_IRQ_FREE _IOW(GPIO_MAGIC, 4, struct gpio_irq)
#define GPIO_IRQ_WAIT _IOW(GPIO_MAGIC, 5, struct gpio_irq)

#endif
