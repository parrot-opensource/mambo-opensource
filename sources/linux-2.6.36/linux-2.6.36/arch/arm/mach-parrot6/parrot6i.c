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
#include <linux/delay.h>
#include <linux/clk.h>

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
#include <mach/dma-pl08x.h>
#include <mach/fb.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/spi.h>
#include <mach/gpio_parrot.h>
#include <mach/regs-rtc-p6i.h>
#include <mach/usb-p6i.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"

static struct map_desc p6i_io_desc[] __initdata = {
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
		.virtual	= PARROT6I_VA_RTC,
		.pfn		= __phys_to_pfn(PARROT6I_RTC),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
};


static struct parrot5_i2cm_platform i2cm_platform_p6 = {
	.bus_freq	= 400*1000,
	.retries        = 1,
};

struct p6i_usb2_platform_data_s p6i_usb2_platform_data = {
	/* board specific information */
	.operating_mode = P6P_USB2_DR_HOST,
	.phy_mode       = P6P_USB2_PHY_UTMI,
	.port_enables   = 1,
	.force_full_speed = 0
};

#define _F(_type)   _PL080_CXCONFIG_FLOWCNTRL_ ## _type
#define _P(_periph) _PL080_PERIPH_ ## _periph

/* PL08x DMA controller flow control table */
static const int p6i_dma_flowctrl[_PL080_PERIPH_MAX][_PL080_PERIPH_MAX] = {
	/* default is invalid */
	[0 ... _PL080_PERIPH_MAX-1][0 ... _PL080_PERIPH_MAX-1] = -1,
	/* source is a peripheral, flow controller = DMAC or peripheral */
	[_P(PARINT)][_P(MEM)] = _F(P2M_DMAC),
	[_P(NAND)][_P(MEM)]   = _F(P2M_DMAC),
	[_P(SPI0)][_P(MEM)]   = _F(P2M_DMAC),
	[_P(SPI1)][_P(MEM)]   = _F(P2M_DMAC),
	[_P(SPI2)][_P(MEM)]   = _F(P2M_DMAC),
	/* source is memory, flow controller = DMAC or peripheral */
	[_P(MEM)][_P(PARINT)] = _F(M2P_DMAC),
	[_P(MEM)][_P(NAND)]   = _F(M2P_DMAC),
	[_P(MEM)][_P(SPI0)]   = _F(M2P_DMAC),
	[_P(MEM)][_P(SPI1)]   = _F(M2P_DMAC),
	[_P(MEM)][_P(SPI2)]   = _F(M2P_DMAC),
	/* source and dest are memory */
	[_P(MEM)][_P(MEM)]    = _F(M2M_DMAC),
};

static struct p6_spi_config p6i_eth_spi_controller_data = {
	.tsetupcs_ns = 10,
	.tholdcs_ns  = 10,
};

static void p6i_jtag_spi_mode(int enable)
{
	if (enable) {
		parrot_select_pin(P6I_SPI2a);
		parrot_select_pin(GPIO_000);
		/* XXX gpio 0 seems input only */
#ifdef CONFIG_GPIOLIB
		gpio_request(0, NULL);
#endif
		gpio_direction_output(0, 0);
		msleep(10);
		gpio_direction_input(0);
		printk("JTAG has been disabled\n");
	}
	else {
		parrot_select_pin(P6I_nTRST);
		parrot_select_pin(P6I_JTAG);
		printk("JTAG is now enabled\n");
	}
}

static struct spi_board_info p6i_eth_spi_board_info[] = {
	{
		.modalias = "ksz8851snl",
		.max_speed_hz = 40000000,
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = p6i_jtag_spi_mode,
		.controller_data = &p6i_eth_spi_controller_data,
		.mode = SPI_MODE_0,
	},
};

static struct parrot_mmc_platform_data p6i_mmc_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.wp_pin = 1,
	.cd_pin = 2,
};

static void sysrq_handle_jtag(int key /*, struct tty_struct *tty*/)
{
	p6i_jtag_spi_mode(0);
}

static struct sysrq_key_op sysrq_showlocks_op = {
	.handler	= sysrq_handle_jtag,
	.help_msg	= "Jtag",
	.action_msg	= "Switch from spi to jtag mode",
};

void __init p6i_eth_on_jtag_init(void)
{
	printk("ethernet is on jtag\n");

	p6i_eth_spi_board_info[0].irq = gpio_interrupt_register(0, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
	spi_register_board_info(p6i_eth_spi_board_info,
				ARRAY_SIZE(p6i_eth_spi_board_info));
}


void sys_set_bit(u32 reg, u32 bit, u32 val)
{
	u32 value = __raw_readl(PARROT6_VA_SYS + reg);
	value = (value & ~(1<<bit)) | ((val & 0x1) << bit);
	__raw_writel(value, PARROT6_VA_SYS + reg);
}

static void p6i_set_sdhci_clock(unsigned long max_clk)
{
	int i;
	unsigned long ahb_clk = clk_get_rate(NULL);

	/* i = 0 violate the sdio spec (wrong ratio) */
	for (i = 1; i <= 7; i++) {
		if ((ahb_clk/(i+3)) < max_clk)
			break;
	}

	if (i <= 7) {
		__raw_writel(i, PARROT6_VA_SYS + _P6_SYS_SDCARD);
		printk("p6i_set_sdhci_clock: %luHz\n", ahb_clk/(i+3));
	} else {
		printk("p6i_set_sdhci_clock: unreachable frequency!\n");
	}
}

/**
 * high speed should only be enabled if you are using SDcards
 */
void p6i_set_pads_sdcard(uint32_t freq)
{
	// 3V3
	sys_set_bit(_P6I_SYS_HVPS, 1, 0);

	// driver one eighth for every one except SD_CLK
	sys_set_bit(_P6I_SYS_E1_2, 19, 0);
	sys_set_bit(_P6I_SYS_E1_2, 20, 0);
	sys_set_bit(_P6I_SYS_E1_2, 21, 0);
	sys_set_bit(_P6I_SYS_E1_2, 22, 0);
	sys_set_bit(_P6I_SYS_E1_2, 23, 0);

	sys_set_bit(_P6I_SYS_E2_2, 19, 0);
	sys_set_bit(_P6I_SYS_E2_2, 20, 0);
	sys_set_bit(_P6I_SYS_E2_2, 21, 0);
	sys_set_bit(_P6I_SYS_E2_2, 22, 0);
	sys_set_bit(_P6I_SYS_E2_2, 23, 0);

	if (freq > 25000000) {
		// half drive
		sys_set_bit(_P6I_SYS_E1_2, 18, 0);
		sys_set_bit(_P6I_SYS_E2_2, 18, 1);
	} else {
		// quarter drive
		sys_set_bit(_P6I_SYS_E1_2, 18, 1);
		sys_set_bit(_P6I_SYS_E2_2, 18, 0);
	}
	p6i_set_sdhci_clock(freq);
}

void p6i_set_pads_i2c(int num)
{
	if (num == 1) {
		//pull-up i2c-1
		sys_set_bit(_P6I_SYS_P1_0, 16, 1);
		sys_set_bit(_P6I_SYS_P2_1, 27, 0);

		sys_set_bit(_P6I_SYS_P1_0, 17, 1);
		sys_set_bit(_P6I_SYS_P2_1, 28, 0);
	}
	else {
		//pull-up i2c-0
		sys_set_bit(_P6I_SYS_P1_0, 0, 1);
		sys_set_bit(_P6I_SYS_P2_1, 11, 0);

		sys_set_bit(_P6I_SYS_P1_0, 23, 1);
		sys_set_bit(_P6I_SYS_P2_2, 2, 0);
	}
}

/* p6i_set_i2c_drive_strength
 * num is for I2C0 or I2C1
 * val:	bit 0 is E1
 * 		bit 1 is E2
 * 	=>	0x0 is 1/8th drive
 * 		0x1 is 1/4th drive
 * 		0x2 is half drive
 * 		0x3 is full drive
 * */
void p6i_set_i2c_drive_strength(int num, int val)
{
	if (num == 1)
	{
		// I2C1 SCL drive strenght :
		sys_set_bit(_P6I_SYS_E1_1, 28, val&0x01);
		sys_set_bit(_P6I_SYS_E2_1, 28, val&0x02);
		// I2C1 SDA drive strenght :
		sys_set_bit(_P6I_SYS_E1_1,  27, val&0x01);
		sys_set_bit(_P6I_SYS_E2_1,  27, val&0x02);
	}
	else
	{
		// I2C0 SCL drive strenght :
		sys_set_bit(_P6I_SYS_E1_1, 11, val&0x01);
		sys_set_bit(_P6I_SYS_E2_1, 11, val&0x02);
		// I2C0 SDA drive strenght :
		sys_set_bit(_P6I_SYS_E1_2,  2, val&0x01);
		sys_set_bit(_P6I_SYS_E2_2,  2, val&0x02);
	}
}

void p6i_set_pads_uart1_pullup(void)
{
    // Pull Up on UART1 RX:
    sys_set_bit(_P6I_SYS_P1_0, 9, 1);
    sys_set_bit(_P6I_SYS_P2_2, 20, 0);
}

void p6i_set_usb_drive_strength(int param, int val)
{
	switch (param)
	{
		case P6I_SYS_USB_CTL0_PHY_PREEMDEPTH:
			sys_set_bit(_P6I_SYS_USB_CTL0, 30, val&0x01);
			break;

		case P6I_SYS_USB_CTL0_PHY_ENPRE:
			sys_set_bit(_P6I_SYS_USB_CTL0, 31, val&0x01);
			break;

		case P6I_SYS_USB_CTL0_PHY_HSDRVSLOPE:
			sys_set_bit(_P6I_SYS_USB_CTL0, 13, val&0x01);
			sys_set_bit(_P6I_SYS_USB_CTL0, 14, (val&0x02) >> 1);
			sys_set_bit(_P6I_SYS_USB_CTL0, 15, (val&0x04) >> 2);
			sys_set_bit(_P6I_SYS_USB_CTL0, 16, (val&0x08) >> 3);
			break;

		case P6I_SYS_USB_CTL0_PHY_HSDRVAMPLITUDE:
			sys_set_bit(_P6I_SYS_USB_CTL0, 11, val&0x01);
			sys_set_bit(_P6I_SYS_USB_CTL0, 12, (val&0x02) >> 1);
			break;

		default:
			break;
	}
}

static void p6i_pads_init(void)
{
	u32 tmp;
	/* set all (safe) pads to Z */

	__raw_writel(0, PARROT6_VA_SYS+_P6I_SYS_P1_0);
	/* bit 0-12 */
	__raw_writel(0, PARROT6_VA_SYS+_P6I_SYS_P1_1);

	/* P2 _P2_1[11] ... SYSC_P2_2[23] */
	tmp = __raw_readl(PARROT6_VA_SYS+_P6I_SYS_P2_1) & 0x7ff;
	__raw_writel(tmp, PARROT6_VA_SYS+_P6I_SYS_P2_1);
	__raw_writel(0, PARROT6_VA_SYS+_P6I_SYS_P2_2);

	/* UART0 */
	tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0xff00);
	__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

	/* set nand nCE & RnB to pull up */
	sys_set_bit(_P6I_SYS_P1_0, 25, 1);
	sys_set_bit(_P6I_SYS_P2_2, 4, 0);

	sys_set_bit(_P6I_SYS_P1_0, 26, 1);
	sys_set_bit(_P6I_SYS_P2_2, 5, 0);
}

static char *board_name(int board)
{
	char *name;
	switch(board) {
		case 0x04:
			name = "FC6050";
			WARN_ON(!machine_is_fc6050());
			break;
		case 0x0a:
			name = "Jump Sumo Classic";
			WARN_ON(!machine_is_jpsumo());
			break;
		case 0x0c:
			name = "Jump Sumo EVO";
			WARN_ON(!machine_is_jpsumo_evo());
			break;
		case 0x21:
			name = "Delos Classic";
			WARN_ON(!machine_is_delos());
			break;
		case 0x24:
			name = "Delos EVO";
			WARN_ON(!machine_is_delos_evo());
			break;
		default:
			name = "Unknow";
	}
	return name;
}

static void print_board_version(void)
{
	/* A[0:15] = RAM_DATA[0:15], A[16:31] = RAM_ADDR[0:15] */
	u32 reset = __raw_readl(PARROT6_VA_SYS+_P6_SYS_RESET);
	int board = (reset >> 16) & 0x7f;
	int rev = (reset >> 23) & 0x7;
	u32 hvps = __raw_readl(PARROT6_VA_SYS+_P6I_SYS_HVPS);
	/* A[32:63] */
	u32 efuse = __raw_readl(PARROT6I_VA_RTC+_RTC_FUSE_A1);

	/* product part A[33:55] */
	efuse = (efuse >> 1) & 0x7fffff;

	printk(KERN_INFO "board %s (0x%02x) rev 0x%02x (rev tag 0x%04x)\n",
			board_name(board), board, rev, system_rev);
	printk(KERN_INFO "nand cfg %sV\n", (hvps&1)?"1.8":"3.3");
	printk(KERN_INFO "efuse %x\n", efuse);

	if (system_rev == 0)
		system_rev = (board<<8)|rev;

	/* use efuse if no config resitor */
	if ((machine_is_fc6050() || machine_is_fc6050b()) && !system_rev)
		system_rev = efuse;
}


void __init p6i_init(void)
{
	BUG_ON(!parrot_chip_is_p6i());
	p6i_pads_init();
	clocks_init_p6();
	print_board_version();
	register_sysrq_key('j', &sysrq_showlocks_op);

	p6_i2cm0_device.dev.platform_data = &i2cm_platform_p6;
	p6_i2cm1_device.dev.platform_data = &i2cm_platform_p6;
	p6_dmac_device.dev.platform_data = &p6i_dma_flowctrl;
	//p6i_usb2_platform_data.operating_mode = P6P_USB2_DR_DEVICE;
	if (parrot_force_usb_device)
		p6i_usb0_device.name = "p6i-ehci-dev";
	p6i_usb0_device.dev.platform_data = &p6i_usb2_platform_data;

	p6_sdhci0_device.dev.platform_data = &p6i_mmc_platform_data;

	/* power management default operator */
	suspend_set_ops(&parrot6_pm_ops_never);

	/* ldo enable */
	__raw_writel(__raw_readl(PARROT6I_VA_RTC+_RTC_PWR_CTRL) | RTC_PWR_CTRL_CODEC_2v8_n3v0, PARROT6I_VA_RTC+_RTC_PWR_CTRL);
	__raw_writel(__raw_readl(PARROT6I_VA_RTC+_RTC_PWR_CTRL) | RTC_PWR_CTRL_CODEC_LDO_EN, PARROT6I_VA_RTC+_RTC_PWR_CTRL);
	msleep(1);
	__raw_writel(__raw_readl(PARROT6I_VA_RTC+_RTC_PWR_CTRL) | RTC_PWR_CTRL_USB_LDO_EN, PARROT6I_VA_RTC+_RTC_PWR_CTRL);
}

static unsigned int pins_init_sip6 [] = {
	P6I_UART0_DEFAULT,
	P6I_AAI_PCM0_DEFAULT,
	0,
};

static unsigned int pins_init_sip6_nand [] = {
	P6I_NAND8_DEFAULT,
	0,
};

void __init sip6_init(int bt_on)
{
	p6i_init();
	parrot_init_pin(pins_init_sip6_nand);
	/* disable NAND flash write protection on P6 dev */
#ifdef CONFIG_GPIOLIB
	gpio_request(29, "NAND_WP");
#endif
	gpio_direction_output(29, 1);
	if (bt_on) {
		parrot_init_pin(pins_init_sip6);
	}
}

void __init p6i_map_io(void)
{
	iotable_init(p6i_io_desc, ARRAY_SIZE(p6i_io_desc));
	/* this should be done after static mapping */
}

#ifdef CONFIG_GPIOLIB
/**
 * p6i_export_gpio() - Expose a gpio line to userspace through sysfs
 *
 * @gpio:   gpio line
 * @flags:  GPIOF_* flags
 * @label:  arbitrary name given to gpio line
 * @direction_may_change: true if userspace may change gpio direction
 *
 * An invalid @gpio will be silently ignored purposedly.
 */
int p6i_export_gpio(int gpio, unsigned int flags, char const* label, bool direction_may_change)
{
  int err;

  if (! gpio_is_valid(gpio)) {
    pr_err("%s: invalid GPIO %d for \"%s\"\n", __func__,
	   gpio,
	   label);
    return -EINVAL;
  }

  err = gpio_request_one((unsigned) gpio, flags, label);
  if (err) {
    pr_err("%s: failed to request GPIO %d for \"%s\" (%d)\n", __func__,
	   gpio,
	   label,
	   err);
    return err;
  }

  /* gpio line MUST refer to a valid & instantiated gpio chip ! */
  BUG_ON(gpio_export((unsigned) gpio, direction_may_change));

  if (label) {
    /* Export GPIO using peudo-driver "user_gpio" */
    err = gpio_export_link(&user_gpio.dev, label, gpio);
    pr_info("%s: Export GPIO-%d as %s %s (%d)\n", __func__,
	    gpio, label,
	    (flags & GPIOF_DIR_IN)?"in":"out",
	    err);
  }

  return 0;
}

void __init p6i_unexport_gpio(int gpio)
{
  if (gpio_is_valid(gpio)) {
    gpio_unexport(gpio);
    gpio_free(gpio);
  }
}
#endif /* CONFIG_GPIOLIB */
