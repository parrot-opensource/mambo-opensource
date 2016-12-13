/*
 *  Copyright (C) 2014 Parrot S.A.
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
#include <mach/regs-rtc-p6i.h>
#include <mach/usb-p6i.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"

#include <linux/serial_max3100.h>
#include <linux/serial_core.h>
#include <mach/aai.h>
#include <asm/irq.h>
#include <mach/gpio_parrot.h>

static char * aai_clock_mode = "";
static char * aai_sync_freq = "";
static char * usb_speed = "";

static unsigned int pins_init_p6i[] = {
	/* uart */
	P6I_UART0_DEFAULT,
	P6I_UART1_RXTX_DEFAULT,
	P6I_UART2_RXTX_DEFAULT,
	P6I_NAND8_DEFAULT,
	P6I_I2CM1_DEFAULT,
	P6I_I2S_SYNC,
	P6I_AAI_PCM0_DEFAULT,
	P6I_I2S_IN1,
	P6I_I2S_IN2,
	P6I_I2S_OUT0,
	P6I_I2S_OUT1,
	P6I_GPIO_006,
	0,
};


static struct parrot_aai_platform_data aai_platform_data;

struct p6i_usb2_platform_data_s fc6000p_usb2_platform_data = {
	/* board specific information */
	.operating_mode = P6P_USB2_DR_HOST,
	.phy_mode       = P6P_USB2_PHY_UTMI,
	.port_enables   = 1,
	.force_full_speed = 0,
};

static struct platform_device *p6i_devices[] __initdata = {
	&p6_uart0_device,
	&p6_uart1_device,
	&p6_uart2_device,
	&p6_nand_device,
	&p6_aai_device,
	&p6_i2cm1_device,
	&p6i_usb0_device,
	/* virtual device */
	&p6_dmac_device,
	&p6_gpio,
};

static void init_aai(char *aai_clock_mode)
{
	int auto_mclk = 0;
	int auto_clk = 0;
	/*Auto : let aai driver select/unselect both pins clocks */
	if(!strcmp(aai_clock_mode,"auto_clock"))
		auto_mclk = auto_clk = 1;
	/*Auto_no_master : let aai driver select/unselect only bit clock pin
	  master clock remains unselected */
	else if(!strcmp(aai_clock_mode,"auto_clock_no_master"))
		auto_clk = 1;
	/*Default : bit and master clock always selected*/
	else {
		parrot_select_pin(P6I_I2S_CLK);
		parrot_select_pin(P6I_MCLK);
	}

	if(!strcmp(aai_sync_freq,"44100"))
		aai_platform_data.sync_freq = 44100;
	else
		aai_platform_data.sync_freq = 48000;


	aai_platform_data.i2s_bit_clock_control = auto_clk;
	aai_platform_data.i2s_master_clock_control = auto_mclk;

	p6_aai_device.dev.platform_data = &aai_platform_data;
}

static void __init p6idev_init(void)
{
	/* force nand to 1.8 V */
	__raw_writel(1, PARROT6_VA_SYS + _P6I_SYS_HVPS);

	p6i_init();
	parrot_init_pin(pins_init_p6i);
	p6i_set_pads_i2c(1);

	/* disable NAND flash write protection on P6 dev */
	gpio_direction_output(29, 1);

	/* Antenna detection */
	gpio_direction_input(8);

	init_aai(aai_clock_mode);

	//Set GPIO Wifi antenna as High-Z
	/* u32 tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0x00c00000); */
	/* __raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS); */

	//Disable SCHMITT triger control for both BT and Wifi antenna
	/* tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_SLEW_SCHMITT) & ~(0x00000A00); */
	/* __raw_writel(tmp, PARROT6I_VA_RTC+_RTC_SLEW_SCHMITT); */

	/* set drive strengh for usb */
	p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_PREEMDEPTH,0);
	p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_ENPRE,1);
	p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVAMPLITUDE,1);

	if (!strcmp(usb_speed, "full")) {
		printk("usb_speed = full (%s)\n", usb_speed);
		fc6000p_usb2_platform_data.force_full_speed = 1;
	} else {
		printk("usb_speed != full (%s)\n", usb_speed);
		fc6000p_usb2_platform_data.force_full_speed = 0;
	}
	p6i_usb0_device.dev.platform_data = &fc6000p_usb2_platform_data;
}

static void __init fc6000p_init(void)
{
	p6idev_init();
	/* gpio for overcurrent detection. Not use ATM */
	gpio_direction_input(3);


	p6i_eth_on_jtag_init();

	platform_add_devices(p6i_devices, ARRAY_SIZE(p6i_devices));
}

static int __init fc6000p_aai_clocks(char *str)
{
	aai_clock_mode = str;
	return 1;
}
__setup("fc6000p_aai_clocks=", fc6000p_aai_clocks);

static int __init fc6000p_aai_sync(char *str)
{
	aai_sync_freq = str;
	return 1;
}
__setup("fc6000p_aai_sync=", fc6000p_aai_sync);

static int __init fc6000p_usb_speed(char *str)
{
	usb_speed = str;
	return 1;
}
__setup("fc6000p_usb_speed=", fc6000p_usb_speed);

MACHINE_START(PARROT_FC6000P, "FC6000P Parrot platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6i_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= fc6000p_init,
MACHINE_END

