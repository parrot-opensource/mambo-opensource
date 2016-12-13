/**
 *
 *       @file  rnb4.c
 *
 *      @brief  RnB4 plateform specific init
 *
 *     @author  Gregor Boirie <gregor.boirie@parrot.com>
 *     @author  Francois Muller <francois.muller@parrot.com>
 *       @date  13-May-2009
 *
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>
#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include <mach/mmc.h>
#include <mach/usb.h>
#include <mach/i2c.h>
#include <mach/spi.h>

#include "timer.h"
#include <linux/syscalls.h>
#include "devs.h"
#include "parrot6.h"
#include <linux/unistd.h>
#include <linux/delay.h>

static const unsigned int pins_init_p6[] = {
	UART0_RXTX_DEFAULT,
	UART2_DEFAULT,
	I2CM0_DEFAULT,
	I2CM1_DEFAULT,
	/* i2s */
	AAI_IO0_DEFAULT,
	AC_OUT0,
	AC_OUT1,
	AC_IN0,
	AC_IN1,
	AC_IN2,
	AAI_PCM0_DEFAULT,
	USB0_DEFAULT,
	USB1_DEFAULT,
	LCD18_DEFAULT,
	NAND8_DEFAULT,
	SD0_DEFAULT,
//	SD1_DEFAULT,
	SPI2_DEFAULT,
	/* gpio */
	GPIO_098,
	GPIO_121,
	GPIO_120,
	GPIO_034,

	MMC_GPIO_DEFAULT,
	ETH_JTAG,
	/* reset usb */
	GPIO_095,
	GPIO_096,
	/* overcurrent usb */
	GPIO_089,
	GPIO_090,
	/* backlight */
	PWM00b,
	PWM01b,
	PWM13b,
	/* subwoofer */
	PWM15b,
	0,
};

static dwc_otg_info_t usb0_info = {
	.ctrl_mode = 1,
	.speed = 1,
	.sof_filter = 7,
	//.reset_pin = -1, /* can't do the reset here because of hub */
	.reset_pin = 95,
	.vbus_detection = 0,
	.fiq_enable = 0,
	.overcurrent_disable = 1,
};

static dwc_otg_info_t usb1_info = {
	.ctrl_mode = 1,
	.speed = 1,
	.sof_filter = 0,
	.reset_pin = 96,
	.vbus_detection = 0,
	.fiq_enable = 0,
};

static struct parrot5_i2cm_platform i2cm_platform_rnb4 = {
	.bus_freq	= 50*1000,
	.retries        = 1,
};

static struct p6_spi_config rnb4_lcd_dat = {
	.tsetupcs_ns = 20,
	.tholdcs_ns  = 60,
};

static const struct spi_board_info rnb4_lcd_info[] = {
	{
		.modalias = "r61509v",
		.max_speed_hz = 2000000,
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = NULL,
		.controller_data = &rnb4_lcd_dat,
		.mode = SPI_MODE_3,
	},
};

static void p6_lcd_init(void)
{
	/* enable vcc */
	gpio_direction_output(98, 1);

	p6_lcd_device.dev.platform_data = &p6_lcd_devices_info_rnb4;
	spi_register_board_info(rnb4_lcd_info, ARRAY_SIZE(rnb4_lcd_info));
}


static struct platform_device rnb4_rotator = {
	.name           = "rnb4_rotator",
	.id             = -1,
	.num_resources  = 0,
	.resource       = NULL,
};

static struct platform_device *p6_devices[] __initdata = {
	&p6_uart0_device,
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
	&p6_spi0_device,
	&p6_spi2_device,
	&p6_sdhci0_device,
	&p6_acpower_device,
	&rnb4_rotator,
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

	/* register */
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

	/* register */
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

	/* phy0 and hub reset are on the same pin. hub got vbus_det set to 5v.
	   May be enabling hub when phy is not ready can cause problem.
	   Doing this seem to work better.
	 */
	/* power down C8000 */
	gpio_direction_output(120, 0); /* reset */
	gpio_direction_output(121, 0); /* power */
	/* reset phy0 and hub */
	gpio_direction_output(95, 0);

	msleep(100);
	/* power the board : hub, ... */
	gpio_direction_output(121, 1); /* Enable power */
	gpio_direction_output(120, 1); /* reset */

	// This debounce is used by both the rotator and the keyboard
	gpio_set_debounce_value(14); // 0,105 ms debounce dead time

	gpio_interrupt_register(102, GPIO_IRQ_POS, GPIO_DEBOUNCE_NOISE);

	/* specific I2C frequency */
	p6_i2cm1_device.dev.platform_data = &i2cm_platform_rnb4;
	i2c_register_board_info(0, p6mu_rtc_i2c_board_info,
			ARRAY_SIZE(p6mu_rtc_i2c_board_info));

	/* eth is on spi0c */
	eth_on_jtag_init();

	/* disable NAND flash write protection on P6 dev */
	gpio_direction_output(34, 1);

	p6_lcd_init();

	if (parrot_force_usb_device)
		usb0_info.ctrl_mode = 2;
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

MACHINE_START(PARROT_RNB4, "RnB4 Parrot dev platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= p6dev_init,
MACHINE_END
