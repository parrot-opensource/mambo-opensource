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
#include <linux/sysrq.h>
#include <linux/mmc/host.h>
#include <linux/i2c.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/i2c.h>
#include <linux/delay.h>
#include <mach/dma-pl08x.h>
#include <mach/fb.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/spi.h>
#include <mach/gpio_parrot.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"

int parrot_force_usb_device;
EXPORT_SYMBOL(parrot_force_usb_device);

static struct map_desc p6_io_desc[] __initdata = {
	{
		.virtual	= PARROT6_VA_VIC,
		.pfn		= __phys_to_pfn(PARROT6_VIC),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_MPMC,
		.pfn		= __phys_to_pfn(PARROT6_MPMC),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_SYS,
		.pfn		= __phys_to_pfn(PARROT6_SYS),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_GPIO,
		.pfn		= __phys_to_pfn(PARROT6_GPIO),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_UART0,
		.pfn		= __phys_to_pfn(PARROT6_UART0),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_UART1,
		.pfn		= __phys_to_pfn(PARROT6_UART1),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_UART2,
		.pfn		= __phys_to_pfn(PARROT6_UART2),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_UART3,
		.pfn		= __phys_to_pfn(PARROT6_UART3),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_USB0,
		.pfn		= __phys_to_pfn(PARROT6_USB0),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_USB1,
		.pfn		= __phys_to_pfn(PARROT6_USB1),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
                .virtual        = PARROT6_VA_H264,
                .pfn            = __phys_to_pfn(PARROT6_H264),
                .length         = SZ_1M,
                .type           = MT_DEVICE
        },
};


static struct parrot5_i2cm_platform i2cm_platform_p6 = {
	.bus_freq	= 10*1000,
	.retries        = 1,
};

#define _F(_type)   _PL080_CXCONFIG_FLOWCNTRL_ ## _type
#define _P(_periph) _PL080_PERIPH_ ## _periph

/* PL08x DMA controller flow control table */
static const int p6_dma_flowctrl[_PL080_PERIPH_MAX][_PL080_PERIPH_MAX] = {
	/* default is invalid */
	[0 ... _PL080_PERIPH_MAX-1][0 ... _PL080_PERIPH_MAX-1] = -1,
	/* source is a peripheral, flow controller = DMAC or peripheral */
	[_P(PARINT)][_P(MEM)] = _F(P2M_DMAC),
	[_P(NAND)][_P(MEM)]   = _F(P2M_DMAC),
	[_P(PSD0)][_P(MEM)]   = _F(P2M_PERIPH),
	[_P(PSD1)][_P(MEM)]   = _F(P2M_PERIPH),
	[_P(SPI0)][_P(MEM)]   = _F(P2M_DMAC),
	[_P(SPI1)][_P(MEM)]   = _F(P2M_DMAC),
	[_P(SPI2)][_P(MEM)]   = _F(P2M_DMAC),
	/* source is memory, flow controller = DMAC or peripheral */
	[_P(MEM)][_P(PARINT)] = _F(M2P_DMAC),
	[_P(MEM)][_P(NAND)]   = _F(M2P_DMAC),
	[_P(MEM)][_P(PSD0)]   = _F(M2P_PERIPH),
	[_P(MEM)][_P(PSD1)]   = _F(M2P_PERIPH),
	[_P(MEM)][_P(SPI0)]   = _F(M2P_DMAC),
	[_P(MEM)][_P(SPI1)]   = _F(M2P_DMAC),
	[_P(MEM)][_P(SPI2)]   = _F(M2P_DMAC),
	/* source and dest are memory */
	[_P(MEM)][_P(MEM)]    = _F(M2M_DMAC),
};

static struct p6_spi_config p6_eth_spi_controller_data = {
	.tsetupcs_ns = 10,
	.tholdcs_ns  = 10,
};

static void p6_jtag_spi_mode(int enable)
{
	if (enable) {
		u32 reg = __raw_readl(PARROT6_VA_SYS + _P6_SYS_IOCR11);
		__raw_writel(reg | P6_SYS_NJTAG_SPI, PARROT6_VA_SYS + _P6_SYS_IOCR11);
		printk("JTAG has been disabled\n");
	}
	else {
		u32 reg = __raw_readl(PARROT6_VA_SYS + _P6_SYS_IOCR11);
		if (reg & P6_SYS_NJTAG_SPI) {
			reg &= ~P6_SYS_NJTAG_SPI;
			__raw_writel(reg, PARROT6_VA_SYS + _P6_SYS_IOCR11);
		}
		printk("JTAG is now enable\n");
	}
}

static struct spi_board_info p6_eth_spi_board_info[] = {
	{
		.modalias = "ksz8851snl",
		.max_speed_hz = 40000000,
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = NULL,
		.controller_data = &p6_eth_spi_controller_data,
		.mode = SPI_MODE_0,
	},
};

static struct parrot_mmc_platform_data p6_mmc_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.wp_pin = 37,
	.cd_pin = 36,
};

static void sysrq_handle_jtag(int key)
{
	p6_jtag_spi_mode(0);
}

static struct sysrq_key_op sysrq_showlocks_op = {
	.handler	= sysrq_handle_jtag,
	.help_msg	= "Jtag",
	.action_msg	= "Switch from spi to jtag mode",
};

/* XXX this should be encoded in pins-p6.h */
static void disable_pullupdown(unsigned int bit)
{
	u32 addr = PARROT6_VA_SYS + _P6_SYS_PUPD0 + (bit/32)*4;
	u32 reg = __raw_readl(addr);
	__raw_writel(reg & ~(1 << (bit&0x1f)), addr);
}

void eth_on_jtag_init(void)
{
	printk("ethernet is on jtag\n");
	p6_eth_spi_board_info[0].platform_data = p6_jtag_spi_mode;
	// hw problem: limit the frequency on the SPI
	p6_eth_spi_board_info[0].max_speed_hz = 30000000;
	// RST pin
	gpio_direction_output(99, 0);
	mdelay(10);
	gpio_direction_output(99, 1);
	if (machine_is_fc6xxx() || machine_is_rnb4()) {
		disable_pullupdown(66); /* gpio 41 */
		gpio_direction_output(41, 0); /* RST */
		mdelay(10);
		gpio_direction_input(41); /* IRQ */
		p6_eth_spi_board_info[0].irq = gpio_interrupt_register(41, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
	} else {
		p6_eth_spi_board_info[0].irq = gpio_interrupt_register(26, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
	}
	spi_register_board_info(p6_eth_spi_board_info,
				ARRAY_SIZE(p6_eth_spi_board_info));
}

void eth_on_spi0_init(void)
{
	printk("ethernet is on spi0\n");
	disable_pullupdown(66); /* gpio 41 */
	gpio_direction_input(41); /* IRQ PIN */
	p6_eth_spi_board_info[0].irq = gpio_interrupt_register(41, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
	spi_register_board_info(p6_eth_spi_board_info,
				ARRAY_SIZE(p6_eth_spi_board_info));
}

static char *board_name(int board)
{
	char *name;
	switch(board) {
		case 0x00:
			name = "Parrot6dev";
			WARN_ON(!machine_is_parrot_p6());
			break;
		case 0x01:
			name = "Okia";
			WARN_ON(!machine_is_okia());
			break;
		case 0x02:
			name = "FC6050";
			break;
		case 0x04:
			name = "FC6100";
			WARN_ON(!machine_is_fc6xxx());
			break;
		case 0x05:
			name = "Parelia";
			WARN_ON(!machine_is_parelia());
			break;
		case 0x08:
			name = "RnB4";
			WARN_ON(!machine_is_rnb4());
			break;
		case 0x09:
			name = "Mykonos";
			WARN_ON(!machine_is_mykonos());
			break;
		default:
			name = "Unknow";
	}
	return name;
}

static void print_board_version(void)
{
	u32 reset = __raw_readl(PARROT6_VA_SYS+_P6_SYS_RESET);
	int board = (reset >> 11) & 0xff;
	int rev = (reset >> 19) & 0xff;

	/* hack for p6dev rev 2 that use boot28 */
	if (machine_is_parrot_p6())
		rev = !(reset & (1<<28));
	/* comment this for first rev of fc6xxx or use a
	   bootloader that give rev tag.
	 */
#if 0
	/* we can check if we are on rev1 or rev2, assume rev2 */
	if (machine_is_fc6xxx() && rev == 0)
		rev++;
#endif

	printk(KERN_INFO "board %s (0x%02x) rev 0x%02x (rev tag 0x%04x)\n",
			board_name(board), board, rev, system_rev);
	if (system_rev == 0)
		system_rev = (board<<8)|rev;
}

void p6_device_release(struct device *dev)
{
	return;
}

void p6_device_init(struct device *dev, void *platform_data)
{
	memset(dev, 0, sizeof(struct device));
	dev->platform_data = platform_data;
	dev->release = p6_device_release;
}


void __init p6_init(void)
{
	BUG_ON(!parrot_chip_is_p6());
	clocks_init_p6();
	print_board_version();
	register_sysrq_key('j', &sysrq_showlocks_op);

	p6_usb0_device.dev.release = &p6_device_release;
	p6_usb1_device.dev.release = &p6_device_release;

	p6_i2cm0_device.dev.platform_data = &i2cm_platform_p6;
	p6_i2cm1_device.dev.platform_data = &i2cm_platform_p6;
	p6_dmac_device.dev.platform_data = &p6_dma_flowctrl;

	p6_psd0_device.dev.platform_data = &p6_mmc_platform_data;
	p6_sdhci0_device.dev.platform_data = &p6_mmc_platform_data;
	p6_sdhci1_device.dev.platform_data = &p6_mmc_platform_data;
	p6_sdhci2_device.dev.platform_data = &p6_mmc_platform_data;

	/* configure mpmc prio for lcd */
	__raw_writel(0x03030303, PARROT6_VA_MPMC+0x2c);
	__raw_writel(0x03030303, PARROT6_VA_MPMC+0x30);
	__raw_writel(0x03030303, PARROT6_VA_MPMC+0x34);

	/* power management default operator */
	suspend_set_ops(&parrot6_pm_ops_never);
}

void __init p6_map_io(void)
{
	iotable_init(p6_io_desc, ARRAY_SIZE(p6_io_desc));
	/* this should be done after static mapping */
}

void __init p6_init_irq(void)
{
	vic_init(__io(PARROT6_VA_VIC), 0, ~0, 0);
}


#ifdef CONFIG_I2C
/* i2c0 helper function */
int i2c0_write(int addr, int reg, int val)
{
	struct i2c_adapter *a = i2c_get_adapter(0);
	struct i2c_msg msg;
	int ret = 0;
	u_int8_t buf[2];

	if (a == NULL)
		return -1;

	msg.addr = addr;
	msg.buf = buf;
	msg.len = 2;
	msg.flags = 0;

	buf[0] = reg & 0xff;
	buf[1] = val & 0xff;

	ret = i2c_transfer(a, &msg, 1);
	if (ret <= 0)
		printk("error : %d\n", ret);

	return ret;
}

int i2c0_read(int addr, int reg)
{
	struct i2c_adapter *a = i2c_get_adapter(0);
	struct i2c_msg msg[2];
	int ret = 0;

	u_int8_t buf[2];

	if (a == NULL)
		return -1;

	msg[0].addr = addr;
	msg[0].buf = buf;
	msg[0].len = 1;
	msg[0].flags = 0;

	buf[0] = reg & 0xff;

	msg[1].addr = addr;
	msg[1].buf = buf+1;
	msg[1].len = 1;
	msg[1].flags = I2C_M_RD;

	ret = i2c_transfer(a, msg, 2);
	if (ret <= 0)
		printk("error : %d\n", ret);

	return buf[1];
}
#endif

struct i2c_board_info /*__initdata*/ p6mu_rtc_i2c_board_info [1] = {
	{
		I2C_BOARD_INFO("p6mu_rtc", 0x49),
	},
};

static int __init parrot_force_usbd(char *str)
{
	parrot_force_usb_device = 1;
	return 1;
}
__setup("parrot_force_usbd", parrot_force_usbd);
