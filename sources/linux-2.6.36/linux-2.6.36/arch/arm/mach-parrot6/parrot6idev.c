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

#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"

static unsigned int pins_init_p6i[] = {
    /* uart */
    P6I_UART0_DEFAULT,      /* bt */
    P6I_UART1_RXTX_DEFAULT, /* no rts/cts */
    P6I_UART2_DEFAULT,
    P6I_NAND8_DEFAULT,
    P6I_I2CM0_DEFAULT,
    P6I_SD0_DEFAULT,
    P6I_MMC_GPIO_DEFAULT,
    P6I_AAI_I2S_SYNC_DEFAULT,
    P6I_AAI_PCM0_DEFAULT,
	P6I_I2S_IN1,
	P6I_I2S_OUT0,
    /* activate bluetooth chip */
    P6I_GPIO_005,
	/* RTC 32K out for TI BT */
	P6I_RTC_32K_OUT,
    /* usb pwr on */
	P6I_USB_PWR_ON,
    0,
};

static struct platform_device *p6i_devices[] __initdata = {
    &p6_uart0_device,
    &p6_uart1_device,
    &p6_uart2_device,
    &p6_nand_device,
    &p6_aai_device,
    &p6_i2cm0_device,
    //&p6_i2cm1_device,
    //&p6_usb0_device,
    //&p6_usb1_device,
    &p6_spi2_device,
    &p6_sdhci0_device,
    //&dmamem_device,
    //&p6mu_rtc,
    //&p6_acpower_device,
    &p6i_usb0_device,
	/* virtual device */
    &p6_dmac_device,
    &p6_gpio,
};

static struct parrot_mmc_platform_data p6idev_mmc_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.wp_pin = 1,
	.cd_pin = 2,
};

static void __init p6idev_init(void)
{
    p6i_init();
    parrot_init_pin(pins_init_p6i);

    /* disable NAND flash write protection on P6 dev */
    gpio_direction_output(29, 1);

    /* pads init */
    p6i_set_pads_i2c(0);
    p6i_set_pads_sdcard(25000000);

    p6i_eth_on_jtag_init();

	p6_sdhci0_device.dev.platform_data = &p6idev_mmc_platform_data;
	if (machine_is_sip6dev())
		p6idev_mmc_platform_data.wp_pin = p6idev_mmc_platform_data.cd_pin = -1;

    platform_add_devices(p6i_devices, ARRAY_SIZE(p6i_devices));
}

#ifdef CONFIG_MACH_PARROT_P6IDEV
MACHINE_START(PARROT_P6IDEV, "P6iDev Parrot platform")
    /* Maintainer: Parrot S.A. */
    .phys_io    = PARROT6_UART0,
    .io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
    .boot_params    = PARROT6_DDR_BASE+0x100,
    .map_io     = p6i_map_io,
    .init_irq   = p6_init_irq,
    .timer      = &p6_timer,
    .init_machine   = p6idev_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_PARROT_SIP6DEV
MACHINE_START(PARROT_SIP6DEV, "P6iDev Parrot platform")
    /* Maintainer: Parrot S.A. */
    .phys_io    = PARROT6_UART0,
    .io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
    .boot_params    = PARROT6_DDR_BASE+0x100,
    .map_io     = p6i_map_io,
    .init_irq   = p6_init_irq,
    .timer      = &p6_timer,
    .init_machine   = p6idev_init,
MACHINE_END
#endif
