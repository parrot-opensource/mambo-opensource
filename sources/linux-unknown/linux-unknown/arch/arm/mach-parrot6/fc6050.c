/*
 *  linux/arch/arm/mach-parrot6/parrot6idev.c
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
#include <mach/regs-rtc-p6i.h>
#include <mach/usb-p6i.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"

#include <linux/serial_max3100.h>
#include <linux/serial_core.h>
#include <mach/spi.h>
#include <mach/aai.h>
#include <asm/irq.h>
#include <mach/gpio_parrot.h>

static char * aai_clock_mode = "";
static char * aai_sync_freq = "";
static char * sd_mode = "";
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
	P6I_GPIO_002, /* marvell/BT reset */
	P6I_GPIO_008, /* antenna detection */
	GPIO_059, /* nRSTOUT not connected */
	GPIO_037, /* IPod Reset */
	0,
};

static unsigned int pins_init_fc6050w[] = {
	P6I_GPIO_003, /* overcurrent detect */
	P6I_USB_PWR_ON,
	GPIO_048, /* GPIO_1 */
	GPIO_049, /* GPIO_2 */
	P6I_GPIO_005, /* GPIO_3 */
	GPIO_036, /* GPIO_MUTE */
	P6I_SD0_DEFAULT,	
	0,
};

static unsigned int pins_init_fc6050b_sdcard[] = {
	P6I_GPIO_001, /* Disable 32k out, not connected on intel chip */
	P6I_GPIO_003, /* SD WP */
	P6I_GPIO_004, /* SD CD */
	P6I_SD0_DEFAULT,	
	0,
};

static unsigned int pins_init_fc6050b_spi[] = {
	P6I_GPIO_001, /* Disable 32k out, not connected on intel chip */
	P6I_GPIO_003, /* SD WP : uart irq */
	P6I_SPI2_DEFAULT,	
	0,
};

static struct parrot_aai_platform_data aai_platform_data;

struct p6i_usb2_platform_data_s fc6050_usb2_platform_data = {
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
	&p6_spi2_device, /* for jtag-eth/spi uart */
	/* virtual device */
	&p6_dmac_device,
	&p6_gpio,
};

static struct platform_device *p6i_sdhci_devices[] __initdata = {
	&p6_sdhci0_device,
};

static struct parrot_mmc_platform_data p6i_mmc_wifi_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	/* wifi : not cd and wp pins */
};

static struct parrot_mmc_platform_data p6i_mmc_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.wp_pin = 3,
	.cd_pin = 4,
};

static struct p6_spi_config p6i_spi_uart_controller_data = {
	.tsetupcs_ns = 10,
	.tholdcs_ns  = 10,
};

static struct plat_max3100 max3100_plat_data = {
        .loopback  	= 0,	  	
        .crystal	= 0,     // 1,8432 Mhz external crystal
        .poll_time	= 100,   //TODO must be check according to the CPU load
};

static struct spi_board_info p6i_spi_uart_board_info[] = {
	{
		.modalias = "max3100_p6",
		.max_speed_hz = 3000000 ,
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = &max3100_plat_data,
		.controller_data = &p6i_spi_uart_controller_data,
		.mode = SPI_MODE_0, //MAX3100 spec
                //.irq                    = ,//TODO complete IRQ information		
	},
};

static void __init p6i_spi_uart_init(void)
{
	printk("spi uart\n");
	gpio_direction_input(3);
	p6i_spi_uart_board_info[0].irq=gpio_interrupt_register(3, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
	spi_register_board_info(p6i_spi_uart_board_info,
				ARRAY_SIZE(p6i_spi_uart_board_info));
}

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

	/*Marvell reset*/
	gpio_direction_output(2, 0);

	init_aai(aai_clock_mode);

	//Set GPIO Wifi antenna as High-Z
	u32 tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0x00c00000);
	__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

	//Disable SCHMITT triger control for both BT and Wifi antenna
	tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_SLEW_SCHMITT) & ~(0x00000A00);
	__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_SLEW_SCHMITT);

	if (!strcmp(usb_speed, "full")) {
		printk("usb_speed = full (%s)\n", usb_speed);
		fc6050_usb2_platform_data.force_full_speed = 1;
	} else {
		printk("usb_speed != full (%s)\n", usb_speed);
		fc6050_usb2_platform_data.force_full_speed = 0;
	}
	p6i_usb0_device.dev.platform_data = &fc6050_usb2_platform_data;
}

static void __init fc6050w_init(void)
{
	p6idev_init();
	/* gpio for overcurrent detection. Not use ATM */
	gpio_direction_input(3);

	p6i_set_pads_sdcard(50000000);
	
	p6i_eth_on_jtag_init();
	
	p6_sdhci0_device.dev.platform_data = &p6i_mmc_wifi_platform_data;
	parrot_init_pin(pins_init_fc6050w);

	platform_add_devices(p6i_sdhci_devices, ARRAY_SIZE(p6i_sdhci_devices));
	platform_add_devices(p6i_devices, ARRAY_SIZE(p6i_devices));
}

static void __init fc6050b_init(void)
{
	p6idev_init();

	if(!strcmp(sd_mode,"none"))
	{
		//use internal weak pull-down for SD pads
		u32 value = __raw_readl(PARROT6_VA_SYS + _P6I_SYS_P1_1);
		value = (value & ~(0xF80));
		__raw_writel(value, PARROT6_VA_SYS + _P6I_SYS_P1_1);

		value = __raw_readl(PARROT6_VA_SYS + _P6I_SYS_P2_2);
		value = (value | 0x7C0000);
		__raw_writel(value, PARROT6_VA_SYS + _P6I_SYS_P2_2);
	}
	else
	{
		/* get SD_DATA2 <=> GPIO_034 value to know if we use SPI
		 * 1 : pull up use SD card
		 * 0 : pull down use SPI2
		 */
		parrot_select_pin(GPIO_034);
		gpio_direction_input(34);

		if (gpio_get_value(34)!=0) {
			p6i_set_pads_sdcard(50000000);
			p6_sdhci0_device.dev.platform_data = &p6i_mmc_platform_data;
			parrot_init_pin(pins_init_fc6050b_sdcard);
			p6i_eth_on_jtag_init();
			platform_add_devices(p6i_sdhci_devices, ARRAY_SIZE(p6i_sdhci_devices));
		}
		else {
			parrot_init_pin(pins_init_fc6050b_spi);
			p6i_spi_uart_init();
		}
	}

	platform_add_devices(p6i_devices, ARRAY_SIZE(p6i_devices));
}

static int __init fc6050_aai_clocks(char *str)
{
	aai_clock_mode = str;
	return 1;
}
__setup("fc6050_aai_clocks=", fc6050_aai_clocks);

static int __init fc6050_aai_sync(char *str)
{
	aai_sync_freq = str;
	return 1;
}
__setup("fc6050_aai_sync=", fc6050_aai_sync);

static int __init fc6050_sd_mode(char *str)
{
	sd_mode = str;
	return 1;
}
__setup("fc6050_sd_mode=", fc6050_sd_mode);

static int __init fc6050_usb_speed(char *str)
{
	usb_speed = str;
	return 1;
}
__setup("fc6050_usb_speed=", fc6050_usb_speed);

MACHINE_START(PARROT_FC6050, "FC6050W Parrot platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6i_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= fc6050w_init,
MACHINE_END

MACHINE_START(PARROT_FC6050B, "FC6050B Parrot platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6i_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= fc6050b_init,
MACHINE_END
