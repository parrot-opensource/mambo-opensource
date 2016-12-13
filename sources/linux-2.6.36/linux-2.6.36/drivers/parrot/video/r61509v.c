/*
 * R61509V display driver
 *
 * Copyright (C) 2010 Parrot SA
 * Author: François MULLER <francois.muller@parrot.com>
 * Inspired by ksz8851snl.c from Florent Bayendrian
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * $Id: ksz8851snl.c,v 1.12 2009-08-26 09:11:44 fbayendrian Exp $
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/time.h>

#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/gpio_parrot.h>

#define DRV_NAME	"r61509v"
#define DRV_VERSION	"$Revision: 1.0 $"

// registre
#define R61509_DEVICE_CODE_READ				0x0
#define R61509_DRIVER_OUTPUT_CONTROL		0x1
#define R61509_LCD_DRIVE_WAVE_CONTROL		0x2
#define R61509_ENTRY_MODE					0x3
#define R61509_DISPLAY_CONTROL_1			0x7
#define R61509_DISPLAY_CONTROL_2			0x8
#define R61509_DISPLAY_CONTROL_3			0x9
#define R61509_8_COLOR_CONTROL				0xB
#define R61509_EXT_DISPLAY_IF_CONTROL_1		0xC
#define R61509_EXT_DISPLAY_IF_CONTROL_2		0xF
#define R61509_PANEL_IF_CONTROL_1			0x10
#define R61509_PANEL_IF_CONTROL_2			0x11
#define R61509_PANEL_IF_CONTROL_3			0x12
#define R61509_PANEL_IF_CONTROL_4			0x13
#define R61509_PANEL_IF_CONTROL_5			0x14
#define R61509_PANEL_IF_CONTROL_6			0x20
#define R61509_PANEL_IF_CONTROL_7			0x21
#define R61509_PANEL_IF_CONTROL_8			0x22
#define R61509_PANEL_IF_CONTROL_9			0x23
#define R61509_FRAME_MARKER_CONTROL			0x90
#define R61509_POWER_CONTROL_1				0x100
#define R61509_POWER_CONTROL_2				0x101
#define R61509_POWER_CONTROL_3				0x102
#define R61509_POWER_CONTROL_4				0x103
#define R61509_RAM_ADRR_SET_H				0x200
#define R61509_RAM_ADRR_SET_V				0x201
#define R61509_RAM_RW						0x202
#define R61509_WIN_H_RAM_ADRR_START			0x210
#define R61509_WIN_H_RAM_ADRR_END			0x211
#define R61509_WIN_V_RAM_ADRR_START			0x212
#define R61509_WIN_V_RAM_ADRR_END			0x213
#define R61509_NVM_DATA_RW					0x280
#define R61509_GAMMA_CONTROL_1				0x300
#define R61509_GAMMA_CONTROL_2				0x301
#define R61509_GAMMA_CONTROL_3				0x302
#define R61509_GAMMA_CONTROL_4				0x303
#define R61509_GAMMA_CONTROL_5				0x304
#define R61509_GAMMA_CONTROL_6				0x305
#define R61509_GAMMA_CONTROL_7				0x306
#define R61509_GAMMA_CONTROL_8				0x307
#define R61509_GAMMA_CONTROL_9				0x308
#define R61509_GAMMA_CONTROL_10				0x309
#define R61509_BASE_IMAGE_NB_LINE			0x400
#define R61509_BASE_IMAGE_DISPLAY_CTRL		0x401
#define R61509_BASE_IMAGE_V_SCROLL_CTRL		0x404
#define R61509_SOFTWARE_RESET				0x600

#define R61509_SET_REGISTER		0
#define R61509_WRITE_DATA		1


static int r61509v_write(struct spi_device *spi, char reg_or_data, int value) // reg_or_data : 0 = set_register, 1 = data
{
	struct spi_transfer	t;
	struct spi_message	msg;
	int ret = 0;
	unsigned char first_frame;
	u8 buf[sizeof(u8) + sizeof(u16)];

	if (reg_or_data == R61509_WRITE_DATA)
		first_frame = 0x72; // data (rs=1 rw=0)
	else if (reg_or_data == R61509_SET_REGISTER)
		first_frame = 0x70; // set index register
	else
		ret = -EINVAL;

	if (ret == 0)
	{
		buf[0] = first_frame;
		buf[1] = value >> 8;
		buf[2] = value & 0xFF;

		memset(&t, 0, sizeof(t));
		t.tx_buf = buf;
		t.len = sizeof(u8) + sizeof(u16);
		t.speed_hz = 500000;//spi->max_speed_hz;

		spi_message_init(&msg);
		spi_message_add_tail(&t, &msg);
		ret = spi_sync(spi, &msg);
		if (ret == 0)
			ret = msg.status;
	}

	return ret;
}

static int r61509v_write_reg(struct spi_device *spi, int reg, int data)
{
	int ret = 0;

	r61509v_write(spi, R61509_SET_REGISTER, reg);
	r61509v_write(spi, R61509_WRITE_DATA, data);

	return ret;
}

void lcd_rnb4_init(struct spi_device *spi)
{
	// LCD hard reset
	gpio_direction_output(93, 0);
	msleep(10);
	gpio_direction_output(93, 1);
	msleep(10);

	// LCD soft reset
	r61509v_write_reg(spi, R61509_SOFTWARE_RESET, 	0x1);
	msleep(1);
	r61509v_write_reg(spi, R61509_SOFTWARE_RESET,	0x0);

	//Gamma correction (Manufacturer)
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_1, 	0x0700);
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_2, 	0x7811);
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_3, 	0x0f05);
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_4, 	0x0308);
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_5, 	0x0111);
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_6, 	0x0803);
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_7, 	0x750f);
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_8, 	0x1108);
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_9, 	0x0007);
	r61509v_write_reg(spi, R61509_GAMMA_CONTROL_10,	0x1110);

	// Settings
	r61509v_write_reg(spi, R61509_DISPLAY_CONTROL_2,	0x8080);
	r61509v_write_reg(spi, R61509_NVM_DATA_RW,		0xa3ff); // VCOMH voltage
	r61509v_write_reg(spi, R61509_WIN_V_RAM_ADRR_END,	0x018f);
	r61509v_write_reg(spi, R61509_BASE_IMAGE_DISPLAY_CTRL,	0x0001);
	r61509v_write_reg(spi, R61509_DRIVER_OUTPUT_CONTROL,	0x0100);
	r61509v_write_reg(spi, R61509_LCD_DRIVE_WAVE_CONTROL,	0x0100);
	r61509v_write_reg(spi, R61509_ENTRY_MODE,		0x1098);

	// RGB operating mode
	r61509v_write_reg(spi, R61509_EXT_DISPLAY_IF_CONTROL_1, 0x0110);
	if (parrot_board_rev() == 1)
		r61509v_write_reg(spi, R61509_EXT_DISPLAY_IF_CONTROL_2, 0x0010); // New hardware (ie 2Gbit DDR)
	else
		r61509v_write_reg(spi, R61509_EXT_DISPLAY_IF_CONTROL_2, 0x0011);

	// Activate display
	r61509v_write_reg(spi, R61509_POWER_CONTROL_4,		0x1200);
	r61509v_write_reg(spi, R61509_POWER_CONTROL_3,		0xc1b0);
	r61509v_write_reg(spi, R61509_DISPLAY_CONTROL_1,	0x0100);

	// Prepare to write to GRAM
	r61509v_write(spi, R61509_SET_REGISTER, R61509_RAM_RW);
}

static int __devinit r61509v_probe(struct spi_device *spi)
{
	int ret = 0;

	dev_info(&spi->dev, "display driver loaded\n");

	lcd_rnb4_init(spi);

	return ret;
}

static int __devexit r61509v_remove(struct spi_device *spi)
{
	// remove ?
	return 0;
}

static struct spi_driver r61509v_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = r61509v_probe,
	.remove = __devexit_p(r61509v_remove),
};

static int __init r61509v_init(void)
{
	printk(KERN_INFO DRV_NAME " driver " DRV_VERSION "\n");
	return spi_register_driver(&r61509v_driver);
}

static void __exit r61509v_exit(void)
{
	spi_unregister_driver(&r61509v_driver);
}

module_init(r61509v_init);
module_exit(r61509v_exit);

MODULE_DESCRIPTION(DRV_NAME " display driver");
MODULE_AUTHOR("François MULLER <francois.muller@parrot.com>");
MODULE_LICENSE("GPL");
