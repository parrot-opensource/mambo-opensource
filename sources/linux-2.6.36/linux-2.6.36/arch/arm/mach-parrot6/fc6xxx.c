/**
 *
 *       @file  fc6xxx.c
 *
 *      @brief  FC6XXX plateform specific init
 *
 *     @author  Gregor Boirie <gregor.boirie@parrot.com>
 *       @date  13-May-2009
 *
 *        $Id: fc6xxx.c,v 1.14 2009-11-16 13:02:32 dguilloteau Exp $
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
#include <mach/mmc.h>
#include <mach/usb.h>

#include "timer.h"
#include "devs.h"
#include "parrot6.h"

//#define FC6XXX_NEW_LCD

static unsigned int pins_init_p6[] = {
	UART0_DEFAULT,
	/* BT */
	UART2_DEFAULT,
	/* uart debug on jtag connector */
	UART1_RXTX_DEFAULT,
	UART3_RXTX_DEFAULT,
	/* i2c */
	I2CM0_DEFAULT,
	I2CM1_DEFAULT,
	/* i2s */
	AAI_IO0_DEFAULT,
	AC_OUT1,
	AC_IN1,
	AC_IN2,
	/* bt pcm */
	AAI_PCM0_DEFAULT,
	/* usb */
	USB0_DEFAULT,
	USB1_DEFAULT,
	/* lcd */
	LCD24_DEFAULT,
	/* nand */
	NAND8_DEFAULT,
	/* wifi */
	SD0_DEFAULT,
	/* sdcard */
	MC1_DEFAULT,
	0,
};

static dwc_otg_info_t usb0_info = {
	.ctrl_mode = 1,
	.speed = 0,
	.sof_filter = 0,
	.reset_pin = 95, //GPIO_095
	.vbus_detection = 1,
	.fiq_enable = 0,
};

static dwc_otg_info_t usb1_info = {
	.ctrl_mode = 1,
	.speed = 1,
	.sof_filter = 0,
	.reset_pin = 96, //GPIO_096
	.vbus_detection = 0,
	.fiq_enable = 0,
};

static void p6_lcd_init(void)
{
	/* enable vcc */
	gpio_direction_output(93, 0);
	/* enable pci */
	gpio_direction_output(92, 1);
	/* enable pwm */
	gpio_direction_output(91, 1);

#ifdef FC6XXX_NEW_LCD
	p6_lcd_device.dev.platform_data = &p6_lcd_devices_info_fc6xxx;
#else
	p6_lcd_device.dev.platform_data = &p6_lcd_devices_info_7p_avea;
#endif
}

static struct platform_device *p6_devices[] __initdata = {
	&p6_uart0_device,
	&p6_uart1_device,
	&p6_uart2_device,
	&p6_uart3_device,
	&p6_dmac_device,
	&p6_nand_device,
	&p6_gpio,
	&p6_aai_device,
	&p6_i2cm0_device,
	&p6_i2cm1_device,
	&p6_usb0_device,
	&p6_usb1_device,
	&p6_lcd_device,
	&p6_psd0_device,
	&p6_spi0_device,
	&p6_sdhci0_device,
	&p6_acpower_device,
};

static void __init p6dev_init(void)
{
	p6_init();
	parrot_init_pin(pins_init_p6);
	/* eth is on spi0 */
	eth_on_jtag_init();

	i2c_register_board_info(0, p6mu_rtc_i2c_board_info,
			ARRAY_SIZE(p6mu_rtc_i2c_board_info));

	/* disable NAND flash write protection on P6 dev */
	gpio_direction_output(34, 1);
	/* Reset_Wlan */
	gpio_direction_output(39, 1);

	/* usb controller configuration */
	/* XXX vbus doesn't seem to work on some board ... */
	if (parrot_force_usb_device)
		usb0_info.ctrl_mode = 2;

	p6_usb0_device.dev.platform_data = &usb0_info;
	p6_usb1_device.dev.platform_data = &usb1_info;

	p6_lcd_init();

	platform_add_devices(p6_devices, ARRAY_SIZE(p6_devices));
}

MACHINE_START(PARROT_FC6XXX, "fc6xxx Parrot dev platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= p6dev_init,
MACHINE_END
