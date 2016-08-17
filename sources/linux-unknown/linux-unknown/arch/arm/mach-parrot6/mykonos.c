/*
 *  linux/arch/arm/mach-parrot6/mykonos.c
 *
 *  Copyright (C) 2009 Parrot S.A.
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
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/p6_kbd.h>
#include <mach/gpio_parrot.h>
#include <mach/usb.h>

#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

#include "timer.h"
#include "devs.h"
#include "parrot6.h"
#include "camerasensor.h"

static unsigned int pins_init_p6[] = {
	/* PWM camera */
	PWM06b,
	PWM00a,
	/* uart */
	UART0_DEFAULT,
	/* pic */
	UART2_RXTX_DEFAULT,
	/* i2c */
	I2CM0_DEFAULT,
	I2CM1_DEFAULT,
	/* usb */
	USB0_DEFAULT,
	/* camif 0 */
	CAM0_DEFAULT,
	CAM1_DEFAULT,
	/* nand */
	NAND8_DEFAULT,
	/* wifi */
	SD1_DEFAULT,
	0,
};

static unsigned int pins_init_p6_hw08[] = {
	/* PWM motors */
	PWM10a,
	PWM11a,
	PWM12a,
	PWM13a,
	0,
};

static unsigned int pins_init_p6_hw10[] = {
	/* Motors GPIO */
	GPIO_067,
	GPIO_068,
	GPIO_069,
	GPIO_070,
	/*Motors UART*/
	UART1_RXTX_DEFAULT,
	0,
};

static struct p6_platform_kbd_input kbd_layout[] = {
    {.gpio =  108, //INIT_CONF_USINE
     .keycode = 101, //0x65
     .debounced = GPIO_DEBOUNCE_NOISE,
     .no_long_press = 0,
     .inverted = 0},

    {.gpio =  158, //RESET_APPAIRAGE
     .keycode = 102, //0x66
     .debounced = GPIO_DEBOUNCE_NOISE,
     .no_long_press = 0,
     .inverted = 0},

    {.gpio = 0,
     .keycode = 0,
     .debounced = 0,
     .no_long_press = 0,
     .inverted = 0}, // Last entry of the array
};

static dwc_otg_info_t usb0_info = {
	.ctrl_mode = 2,
	.sof_filter = 7,
	.reset_pin = -1,
	.vbus_detection = 0,
	.fiq_enable = 0,
};

static struct soc_camera_platform_info ov772x_info0 = {
	.iface = 0,
	.get_param = ov772x_get_param,
};

static struct platform_device camif_sensor0 = {
	.name		= "soc_cam_plat_p6",
	 .id = 0,
	.dev	= {
		.platform_data	= &ov772x_info0,
	},
};

static struct soc_camera_platform_info cresyn_info1 = {
	.iface = 1,
	.get_param = cresyn_get_param,
};

static struct platform_device camif_sensor1 = {
	.name		= "soc_cam_plat_p6",
	.id = 1,
	.dev	= {
		.platform_data	= &cresyn_info1,
	},
};

static void p6_camif_init(void)
{
	gpio_direction_output(101, 0);  // video_PWDN_H
	gpio_direction_output(109, 1);  // video_CE_H
	gpio_direction_output(57, 1);  //CE cam 1
}

static struct platform_device *p6_devices[] __initdata = {
	&p6_uart0_device,
	&p6_uart2_device,
	&p6_dmac_device,
	&p6_nand_device,
	&p6_gpio,
	&p6_h264,
	&p6_i2cm0_device,
	&p6_i2cm1_device,
	&p6_usb0_device,
	&camif_sensor0,
	&p6_camif0_device,
	&camif_sensor1,
	&p6_camif1_device,
	&dmamem_device,
	&p6_sdhci1_device,
	&p6_spi0_device,
	&p6_kbd_device,
};

static struct platform_device *p6_devices_hw10[] __initdata = {
	&p6_uart1_device,
};

static void __init p6dev_init(void)
{
	p6_init();
	parrot_init_pin(pins_init_p6);
	/* 0x7 is first hw, 0x2 is next ... */
	if (parrot_board_rev() == 0x7)
		parrot_init_pin(pins_init_p6_hw08);
	else
		parrot_init_pin(pins_init_p6_hw10);

	eth_on_jtag_init();
	/* disable NAND flash write protection on P6 dev */
	gpio_direction_output(34, 1);

	/* Reset_Wlan */
	gpio_direction_output(43, 1);

	/* red LED  on*/
	gpio_direction_output(63, 1);

	/* green LED  off*/
	gpio_direction_output(64, 0);

	gpio_direction_input(106);  // Cutout
	gpio_direction_output(107, 1);  // Motor Enable

	/* usb controller configuration */
	p6_usb0_device.dev.platform_data = &usb0_info;

	p6_camif_init();

	p6_kbd_device.dev.platform_data = &kbd_layout;

	platform_add_devices(p6_devices, ARRAY_SIZE(p6_devices));

	if (parrot_board_rev() != 0x7)
		platform_add_devices(p6_devices_hw10, ARRAY_SIZE(p6_devices_hw10));


	// setting SDIO clock to 17.3Mhz to avoid pertubation on wifi channels.
	__raw_writel(P6_SYS_156MHZ_SDIO_17_3MHZ,
			PARROT6_VA_SYS+_P6_SYS_SDCARD);
}

MACHINE_START(PARROT_MYKONOS, "Mykonos Parrot platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= p6dev_init,
MACHINE_END
