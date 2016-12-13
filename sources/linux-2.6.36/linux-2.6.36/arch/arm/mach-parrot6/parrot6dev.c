/*
 *  linux/arch/arm/mach-parrot6/p6.c
 *
 *  Copyright (C) 2008 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2008-11-05
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
#include <linux/i2c.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/usb.h>

#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "camerasensor.h"

//#define USE_PSD // only for a patched p6dev board (DevP6-12)
#define EXTERNAL_ETH // boitier JTAG
//#define SDHCI1

static unsigned int pins_init_p6[] = {
	/* uart */
	UART0_DEFAULT,
	/* UART1_RTS used for ethernet chip reset on revision B */
	UART1_RXTX_DEFAULT,
	/* uart 2 is multiplexed with spi */
	UART2_DEFAULT,
	/* i2c */
	I2CM0_DEFAULT,
	I2CM1_DEFAULT,
	/* i2s */
	AAI_IO0_DEFAULT,
	/* i2s asynchronous inputs
	AC_SYNC25,
	AC_CLK25,
	AC_SYNC36,
	AC_CLK36,
	*/
	AC_OUT1,
	AC_IN1,
	AC_IN2,
	AC_IN5,
	/* bt pcm */
	AAI_PCM0_DEFAULT,
	/* usb */
	USB0_DEFAULT,
	USB1_DEFAULT,
	/* lcd */
	LCD24_DEFAULT,
	/* camif 0 */
	//CAM0_DEFAULT,
	/* nand */
	NAND8_DEFAULT,
	/* spi */
#ifndef EXTERNAL_ETH
	SPI0_DEFAULT,
#endif
#ifdef SDHCI1
	SD1_DEFAULT,
#else
	SD0_DEFAULT,
#endif
	/* psd */
#ifdef USE_PSD
	MC1_DEFAULT,
#endif
	0,
};

#if 0
static struct soc_camera_platform_info ov772x_info = {
	.iface = 0,
	.get_param = ov772x_get_param,
};

static struct platform_device camif_sensor = {
	.name		= "soc_cam_plat_p6",
	.dev	= {
		.platform_data	= &ov772x_info,
	},
};

static void p6_camif_init(void)
{
	gpio_direction_output(101, 0);
}
#endif

static void p6_lcd_init(void)
{
/* enable this if you have a not lcd patched card */
#if 0
	/* enable vcc */
	gpio_direction_output(102, 0);
	/* enable pci */
	gpio_direction_output(103, 1);
#else
	/* enable vcc */
	gpio_direction_output(123, 0);
	/* enable pci */
	gpio_direction_output(122, 1);
#endif
	if (parrot_board_rev() == 0) {
		/* enable pwm */
		gpio_direction_input(92);
		gpio_direction_output(93, 1);
			}
	else {
		/* enable pwm */
		gpio_direction_input(100);
		gpio_direction_output(83, 1);
	}
#if 0
	//local patch for lcd + camif0
	gpio_direction_input(124);
	gpio_direction_output(125, 1);
#endif
	p6_lcd_device.dev.platform_data = &p6_lcd_devices_info_7p_avea;
}

static struct platform_device *p6_devices[] __initdata = {
	&p6_uart0_device,
	&p6_uart1_device,
	&p6_uart2_device,
	&p6_dmac_device,
	&p6_nand_device,
	&p6_gpio,
	&p6_aai_device,
	&p6_i2cm0_device,
	&p6_i2cm1_device,
	&p6_usb0_device,
	&p6_usb1_device,
	&p6_lcd_device,
#if 0
	/* some pin of camif0 are multiplexed with aai ... */
	&camif_sensor,
	&p6_camif0_device,
#endif
	&p6_spi0_device,
#ifdef SDHCI1
	&p6_sdhci1_device,
#else
	&p6_sdhci0_device,
#endif
#ifdef USE_PSD
	&p6_psd0_device,
#endif
	&dmamem_device,
	&p6_acpower_device,
};

static dwc_otg_info_t usb0_info = {
	.ctrl_mode = 1,
	.speed = 0,
	.sof_filter = 7,
	.reset_pin = 97,
	.vbus_detection = 1,
	.fiq_enable = 0,
};

static dwc_otg_info_t usb1_info = {
	.ctrl_mode = 1,
	.speed = 1,
	.sof_filter = 0,
	.reset_pin = -1,
	.vbus_detection = 0,
	.fiq_enable = 0,
};

static DEFINE_MUTEX(usbctrl0_mutex);
static DEFINE_MUTEX(usbctrl1_mutex);

/* modify speed parameter for USB controller */
static ssize_t usbctrl0_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char * buf, size_t n)
{
	unsigned int value = ~0;

	mutex_lock(&usbctrl0_mutex);

	if (sscanf(buf, "%u", &value) != 1 ||
	    (value != 0 && value != 1)) {
		printk(KERN_ERR "usbctrl_store: Invalid value\n");
		return -EINVAL;
	}

	if (usb0_info.speed == value) {
		mutex_unlock(&usbctrl0_mutex);
		return n;
	}

	/* unregister */
	platform_device_unregister(&p6_usb0_device);

	/*register */
	usb0_info.speed = value;
	p6_device_init(&p6_usb0_device.dev, &usb0_info);
	platform_device_register(&p6_usb0_device);

	mutex_unlock(&usbctrl0_mutex);

	return n;
}

static ssize_t usbctrl0_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%u\n", usb0_info.speed);
}

static ssize_t usbctrl1_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char * buf, size_t n)
{
	unsigned int value = ~0;

	mutex_lock(&usbctrl1_mutex);

	if (sscanf(buf, "%u", &value) != 1 ||
	    (value != 0 && value != 1)) {
		printk(KERN_ERR "usbctrl_store: Invalid value\n");
		return -EINVAL;
	}

	if (usb1_info.speed == value) {
		mutex_unlock(&usbctrl1_mutex);
		return n;
	}

	/* unregister */
	platform_device_unregister(&p6_usb1_device);

	/*register */
	usb1_info.speed = value;
	p6_device_init(&p6_usb1_device.dev, &usb1_info);
	platform_device_register(&p6_usb1_device);

	mutex_unlock(&usbctrl1_mutex);

	return n;
}

static ssize_t usbctrl1_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%u\n", usb1_info.speed);
}

static struct kobj_attribute usbctrl0_attr = __ATTR(usbctrl0, 0644, usbctrl0_show, usbctrl0_store);
static struct kobj_attribute usbctrl1_attr = __ATTR(usbctrl1, 0644, usbctrl1_show, usbctrl1_store);

static struct attribute *attrs[] = {
	&usbctrl0_attr.attr,
	&usbctrl1_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *usbctrl_kobj;

static void __init p6dev_init(void)
{
	int ret;

	p6_init();
	parrot_init_pin(pins_init_p6);

	/* eth is on spi0 */
	if (parrot_board_rev() == 0) {
		gpio_direction_output(99, 1); /* Reset PIN */
	}
	else {
		i2c_register_board_info(0, p6mu_rtc_i2c_board_info,
				ARRAY_SIZE(p6mu_rtc_i2c_board_info));
		gpio_direction_output(120, 1); /* reset pin for revision B */
	}

#ifdef EXTERNAL_ETH
	printk(KERN_CRIT "******\n");
	printk(KERN_CRIT "Ethernet is on jtag.\n");
	printk(KERN_CRIT "Comment EXTERNAL_ETH in arch/arm/mach-parrot6/parrot6dev.c\n");
	printk(KERN_CRIT "to use the on board controller\n");
	printk(KERN_CRIT "******\n");
	eth_on_jtag_init();
#else
	eth_on_spi0_init();
#endif
	/* disable NAND flash write protection on P6 dev */
	gpio_direction_output(34, 1);

	p6_lcd_init();

	/* usb controller configuration */
	p6_usb0_device.dev.platform_data = &usb0_info;
	p6_usb1_device.dev.platform_data = &usb1_info;

	platform_add_devices(p6_devices, ARRAY_SIZE(p6_devices));

	usbctrl_kobj = kobject_create_and_add("usbctrl", NULL);
	if (!usbctrl_kobj) {
		printk(KERN_ERR "usbctrl_kobj creation failed\n");
		return;
	}

	ret = sysfs_create_group(usbctrl_kobj, &attr_group);
	if (ret) {
		printk(KERN_ERR "sysfs_create_group failed: %d\n", ret);
	}
}

MACHINE_START(PARROT_P6DEV, "P6Dev Parrot platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= p6dev_init,
MACHINE_END
