/*
 *
 *  Copyright (C) 2009 Parrot S.A.
 *
 * @author     matthieu.castet@parrot.com
 * @date       2009-05
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

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/input.h>
#include <linux/i2c.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>
#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/p6_kbd.h>
#include <mach/usb.h>
#include <mach/gpio_parrot.h>

#include "timer.h"
#include "devs.h"
#include "parrot6.h"

static unsigned int pins_init_p6[] = {
	/* uart 0 console */
	UART0_DEFAULT,
	/* uart 2 BT */
	UART2_DEFAULT,
	/* i2c : pmu */
	I2CM0_DEFAULT,
	/* usb : only usb0 */
	USB0_DEFAULT,
	USB1_DEFAULT,
	/* lcd */
	LCD24_NODE_DEFAULT,
	/* nand */
	NAND8_DEFAULT,
	/* spi 2 ecran */
	SPI2_DEFAULT,
	/* wifi revA, sdcard revB */
	SD0_DEFAULT,
	/* sdcard revA, wifi revB */
	MC1_DEFAULT,
	/* aai */
	AAI_IO0_DEFAULT,
	/* BL pwm */
	PWM13b,
	PWM14b,
	0,
};

#define MAX_PRESSED_TIME    0
#define DELAY_TIME          0
#define REFUSE_LONG_PRESS   0
#define ACCEPT_LONG_PRESS   1

static struct p6_platform_kbd_input kbd_layout[] = {
    {.gpio =  98, //HOME
     .keycode = 83, //0x53 Send a KEYCODE_NOTIFICATION
     .debounced = GPIO_DEBOUNCE_NOISE,
     .no_long_press = REFUSE_LONG_PRESS,
     .inverted = 0,
     .delay = DELAY_TIME,
     .max_pressed_time = MAX_PRESSED_TIME},

    {.gpio =  88, //MENU
     .keycode = 59, //0x3B
     .debounced = GPIO_DEBOUNCE_NOISE,
     .no_long_press = REFUSE_LONG_PRESS,
     .inverted = 0,
     .delay = DELAY_TIME,
     .max_pressed_time = MAX_PRESSED_TIME},

    {.gpio = 131, //BACK
     .keycode = 158, //0x9E
     .debounced = GPIO_DEBOUNCE_NOISE,
     .no_long_press = REFUSE_LONG_PRESS,
     .inverted = 0,
     .delay = DELAY_TIME,
     .max_pressed_time = MAX_PRESSED_TIME},

    {.gpio =  97, //POWER
     .keycode = 116, //0x74
     .debounced = GPIO_DEBOUNCE_NOISE,
     .no_long_press = REFUSE_LONG_PRESS,
     .inverted = 0,
     .delay = DELAY_TIME,
     .max_pressed_time = MAX_PRESSED_TIME},

    {.gpio = 124, //ENTER
     .keycode = 232, //0xE8
     .debounced = GPIO_DEBOUNCE_NOISE,
     .no_long_press = REFUSE_LONG_PRESS,
     .inverted = 0,
     .delay = DELAY_TIME,
     .max_pressed_time = MAX_PRESSED_TIME},

    {.gpio = 102, //LEFT
     .keycode = 105, //0x69
     .debounced = GPIO_DEBOUNCE_NONE,
     .no_long_press = ACCEPT_LONG_PRESS,
     .inverted = 0,
     .delay = 3,
     .max_pressed_time = MAX_PRESSED_TIME},

    {.gpio = 103, //RIGHT
     .keycode = 106, //0x6A
     .debounced = GPIO_DEBOUNCE_NONE,
     .no_long_press = ACCEPT_LONG_PRESS,
     .inverted = 0,
     .delay = 3,
     .max_pressed_time = MAX_PRESSED_TIME},

    {.gpio = 104, //UP
     .keycode = 103, //0x67
     .debounced = GPIO_DEBOUNCE_NONE,
     .no_long_press = ACCEPT_LONG_PRESS,
     .inverted = 0,
     .delay = 3,
     .max_pressed_time = MAX_PRESSED_TIME},

    {.gpio = 105, //DOWN
     .keycode = 108, //0x6C
     .debounced = GPIO_DEBOUNCE_NONE,
     .no_long_press = ACCEPT_LONG_PRESS,
     .inverted = 0,
     .delay = 3,
     .max_pressed_time = MAX_PRESSED_TIME},

    {.gpio = 0,
     .keycode = 0,
     .debounced = 0,
     .no_long_press = 0,
     .inverted = 0,
     .delay = 0,
     .max_pressed_time = 0}, // Last entry of the array
};


static dwc_otg_info_t usb0_info = {
	.ctrl_mode = 1,
	.sof_filter = 7,
	.reset_pin = 95,
	.vbus_detection = 1,
	.fiq_enable = 0,
};

static dwc_otg_info_t usb1_info = {
	.ctrl_mode = 1,
	.sof_filter = 7,
	.reset_pin = 96,
	.vbus_detection = 0,
	.fiq_enable = 0,
};

static void p6_lcd_init(void)
{
	/* lcd power */
	gpio_direction_output(91, 1);
	/* pwm, should be removed when pwm driver is used */
	//gpio_direction_output(92, 1);
	p6_lcd_device.dev.platform_data = &p6_lcd_devices_info_10p_parelia;

	/* useless as i2c system is not ready here */
#if 0
	printk("pmu %d\n", i2c0_read(0x49, 0xa));
	i2c0_write(0x49, 0xa, 0x8e);
	printk("pmu %d\n", i2c0_read(0x49, 0xa));
#endif
}

static struct platform_device *p6_devices[] __initdata = {
	&p6_uart0_device,
	&p6_uart2_device,
	&p6_dmac_device,
	&p6_nand_device,
	&p6_gpio,
	&p6_i2cm0_device,
	&p6_usb0_device,
	&p6_lcd_device,
	&p6_spi0_device,
	&p6_spi2_device,
	&p6_kbd_device,
	&p6_aai_device,
	&dmamem_device,
	&p6_acpower_device,
	&p6_sdhci0_device,
};

static struct platform_device *okia_devices[] __initdata = {
	&p6_usb1_device,
};

static void __init p6dev_init(void)
{
	p6_init();

	parrot_init_pin(pins_init_p6);

	/* disable phy here to make vbus power down */
	gpio_direction_output(95, 0);

	/* eth is on spi0 */
	eth_on_jtag_init();
	/* disable NAND flash write protection on P6 dev */
	gpio_direction_output(34, 1);
	i2c_register_board_info(0, p6mu_rtc_i2c_board_info,
				ARRAY_SIZE(p6mu_rtc_i2c_board_info));

	p6_lcd_init();
	p6_kbd_device.dev.platform_data = &kbd_layout;

	/* usb controller configuration */
	p6_usb0_device.dev.platform_data = &usb0_info;

	if (machine_is_okia()) {
		p6_usb1_device.dev.platform_data = &usb1_info;
		platform_add_devices(okia_devices, ARRAY_SIZE(okia_devices));
	}
	platform_add_devices(p6_devices, ARRAY_SIZE(p6_devices));
}

MACHINE_START(PARROT_PARELIA, "Parelia Parrot dev platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= p6dev_init,
MACHINE_END

MACHINE_START(PARROT_OKIA, "Okia Parrot platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= p6dev_init,
MACHINE_END
