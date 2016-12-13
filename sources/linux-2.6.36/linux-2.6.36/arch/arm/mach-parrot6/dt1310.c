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
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
#include <linux/mtd/physmap.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/usb.h>
#include <mach/spi.h>

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
    P6I_SPI0_DEFAULT,
    P6I_SPI1_DEFAULT,
    P6I_SPI2a,
    /* usb pwr on */
	P6I_USB_PWR_ON,
    /* GPIOs to control octopus reset */
    GPIO_052,
    GPIO_053,
    P6I_GPIO_002, /* marvell/BT reset */
    P6I_SD0_DEFAULT,
    0,
};

static struct platform_device *p6i_devices[] __initdata = {
    &p6_uart0_device,
    &p6_uart1_device,
    &p6_spi0_device,
    &p6_spi1_device,
    &p6_spi2_device,
    //&dmamem_device,
    &p6i_usb0_device,
	&p6_sdhci0_device,
	/* virtual device */
    &p6_dmac_device,
    &p6_gpio,
};


static struct p6_spi_config p6i_spi_controller_data = {
	.tsetupcs_ns = 0,
	.tholdcs_ns  = 0,
};

#if defined(CONFIG_MTD_M25P80) \
	|| defined(CONFIG_MTD_M25P80_MODULE)
/* SPI flash chip */
static struct mtd_partition p6idev_spi_flash_partitions[] = {
	{
		.name = "nor_spi_part_1",
        .size = MTDPART_SIZ_FULL,
		.offset = 0,
	},
#if 0
    {
		.name = "nor_spi_part_2",
        .size = 0x00800000,
		.offset = MTDPART_OFS_APPEND,
    }
#endif
};

static struct flash_platform_data p6idev_spi_flash_data = {
	.name = "m25p80",
	.parts = p6idev_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(p6idev_spi_flash_partitions),
//    .type = "w25x128",
    .type = "w25q256",

};

static struct p6_spi_config p6i_flash_spi_controller_data = {
	.tsetupcs_ns = 0,
	.tholdcs_ns  = 0,
	.disable_dma = 1,
};
#endif


static struct spi_board_info p6i_spi_board_info[] =  {
/* SPI FLASH */
#if defined(CONFIG_MTD_M25P80) \
	|| defined(CONFIG_MTD_M25P80_MODULE)
	{
		.modalias = "m25p80",
		.max_speed_hz = 50000000,
        .bus_num = 0,
		.chip_select = 0,
		.platform_data = &p6idev_spi_flash_data,
		.controller_data = &p6i_flash_spi_controller_data,
		.mode = SPI_MODE_0,
	},
#endif
/* SPIDEV1.0 */
    {
        .modalias = "spidev",
        .max_speed_hz = 20000000,
        .bus_num = 1,
        .chip_select = 0,
        .platform_data = NULL,
		.controller_data = &p6i_spi_controller_data,
        .mode = SPI_MODE_0,
    },

/* SPIDEV2.0 */
    {
        .modalias = "spidev",
        .max_speed_hz = 20000000,
        .bus_num = 2,
        .chip_select = 0,
        .platform_data = NULL,
		.controller_data = &p6i_spi_controller_data,
        .mode = SPI_MODE_0,
    }
};

static struct parrot_mmc_platform_data p6i_mmc_wifi_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	/* wifi : not cd and wp pins */
};



static void __init p6idev_init(void)
{
    printk("++++p6idev_init\n");
    p6i_init();
    printk("calling parrot_init_pin\n");
    parrot_init_pin(pins_init_p6i);

    /* Octopus POWER_DWN */
    gpio_direction_output(53, 0);
    /* Octopus RESETN */
    gpio_direction_output(52, 0);

    /* pads init */
    p6i_set_pads_i2c(0);
    p6i_set_pads_sdcard(25000000);

    printk("Now registering p6i_spi_board_info, nb spi = %d\n", ARRAY_SIZE(p6i_spi_board_info));
    spi_register_board_info(p6i_spi_board_info, ARRAY_SIZE(p6i_spi_board_info));


    p6_sdhci0_device.dev.platform_data = &p6i_mmc_wifi_platform_data;

    platform_add_devices(p6i_devices, ARRAY_SIZE(p6i_devices));
}

MACHINE_START(PARROT_DT1310, "dt1310 Parrot platform")
    /* Maintainer: Parrot S.A. */
    .phys_io    = PARROT6_UART0,
    .io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
    .boot_params    = PARROT6_DDR_BASE+0x100,
    .map_io     = p6i_map_io,
    .init_irq   = p6_init_irq,
    .timer      = &p6_timer,
    .init_machine   = p6idev_init,
MACHINE_END
