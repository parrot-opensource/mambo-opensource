/*
 *  linux/arch/arm/mach-parrot6/clock.c
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
#include <linux/err.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>

/*
 * Note on Parrot6 clocks:
 *
 * The Parrot6 can run in two modes:
 *
 * PCLK = 156 MHz (bit 0 in register _SYS_SPEED set to 1)
 * PCLK = 130 MHz (bit 0 in register _SYS_SPEED set to 0)
 *
 * Almost all peripherals derive clocks from PCLK. There are exceptions, such
 * as CAN. The CPU frequency is a multiple of PCLK, the multiplier being 2 or 3
 * depending on reset configuration:
 *
 * CPU CLK = 2*PCLK (bit 2 in register _SYS_RESET set to 1)
 * CPU CLK = 3*PCLK (bit 2 in register _SYS_RESET set to 0)
 *
 * PCLK can be dynamically switched to 156 MHz or 130 MHz; in order to change
 * PLL frequency, the ARM core must first set the _SYS_SPEED register, then go
 * to WFI mode and wait for a spurious interrupt.
 *
 * The CPU frequency multiplier _cannot_ be changed dynamically, it is sampled
 * once and for all at reset.
 *
 * Summary:
 *
 * CONFIGURATION                                      CPU CLK   PCLK
 * --------------------------------------------------------------------
 * (SYS_SPEED_156MHZ == 0) && (SYS_RESET_A2 == 1)     260 MHz   130 MHz
 * (SYS_SPEED_156MHZ == 1) && (SYS_RESET_A2 == 1)     312 MHz   156 MHz
 * (SYS_SPEED_156MHZ == 0) && (SYS_RESET_A2 == 0)     390 MHz   130 MHz
 * (SYS_SPEED_156MHZ == 1) && (SYS_RESET_A2 == 0)     468 MHz   156 MHz
 */

/*
 * Note on Parrot6i clocks:
 *
 * The Parrot6i can run in the following modes (clock values in MHz):
 *
 * --------------------------------------------------------------------
 * CLK CLK  PLL   ARM  ARM/ HCLK  DIV  SDRAM  SPEED SDR_nDDR Async
 * IN   F   CLK   CLK  AHB       SDRAM  CLK   [4:0]          clocks
 * --------------------------------------------------------------------
 * 26  16   416   208    2  104     4  104      0      x
 * 26  17   442   221    2  110.5   4  110.5    1      x
 * 26  18   468   234    2  117     4  117      2      x
 * 26  19   494   247    2  123.5   4  123.5    3      x
 * 26  20   520   260    2  130     4  130      4      x
 * 26  21   546   273    2  136.5   4  136.5    5      x
 * 26  22   572   286    2  143     4  143      6      x
 * 26  23   598   299    2  149.5   4  149.5    7      x
 * 26  24   624   312    2  156     4  156      8      0
 * 26  24   624   312    2  156     6  104      8      1       *
 * 26  25   650   325    3  108.33  6  108.33   9      x
 * 26  26   676   338    3  112.67  6  112.67  10      x
 * 26  27   702   351    3  117     6  117     11      x
 * 26  28   728   364    3  121.33  6  121.33  12      x
 * 26  29   754   377    3  125.67  6  125.67  13      x
 * 26  30   780   390    3  130     6  130     14      x
 * 26  31   806   403    3  134.33  6  134.33  15      x
 * 26  32   832   416    3  138.67  6  138.67  16      0
 * 26  32   832   416    3  138.67  8  104     16      1       *
 * --------------------------------------------------------------------
 * 26  33   858   429    3  143     6  143     17      0
 * 26  33   858   429    3  143     8  107.25  17      1       *
 * 26  34   884   442    3  147.33  6  147.33  18      0
 * 26  34   884   442    3  147.33  8  110.5   18      1       *
 * 26  35   910   455    3  151.67  6  151.67  19      0
 * 26  35   910   455    3  151.67  8  113.75  19      1       *
 * 26  36   936   468    3  156     6  156     20      0
 * 26  36   936   468    3  156     8  117     20      1       *
 *
 * ARM frequencies > 416 are not guaranteed to work (overclocking)
 *
 * PLL_MAIN  = CLK_IN * clf_f (16 <= clk_f <= 36) = CLK_IN * (speed+16)
 * PLL_USB   = CLK_IN * 12
 *
 * ARM_CLK   = PLL_MAIN/2
 * HCLK      = PLL_MAIN/div_hclk
 * SDRAM_CLK = PLL_MAIN/div_sdram
 * div_hclk  = (speed > 8)? 6 : 4
 * div_sdram = div_hclk + 2*SDR_nDDR
 *
 * At boot time: AHB bus matrix, NAND, GPIO and INTROM are enabled; all other
 * clock are disabled
 * clk_f = 24 (speed = 8), div_hclk = 4, div_sdram = 4
 * => PLL_MAIN = 624 MHz, ARM_CLK = 312 MHz, HCLK = SDRAM_CLK = 156 MHz
 */

struct clk {
	const char*  id;            /* peripheral name */
	unsigned int bit;           /* clock registers enable/disable bit */
	unsigned int offset;        /* clock registers offset */
	int users;                  /* num users that need the clock enabled */
};

/* psd clocks use separate clock registers */
#define PSD_CLK_OFFSET (_P6_SYS_PSD_CEN-_P6_SYS_CEN)

static struct clk clocks[] = {
	{.id = "dmac",  .bit = P6_SYS_CLK_DMA,   .offset = 0},
	{.id = "camif0",.bit = P6_SYS_CLK_CAMIF0,.offset = 0},
	{.id = "camif1",.bit = P6_SYS_CLK_CAMIF1,.offset = 0},
	{.id = "aai",   .bit = P6_SYS_CLK_AAI,   .offset = 0},
	{.id = "h264",  .bit = P6_SYS_CLK_H264,  .offset = 0},
	{.id = "lcdc",  .bit = P6_SYS_CLK_LCDC,  .offset = 0},
	//{.id = "cv",    .bit = P6_SYS_CLK_CV,    .offset = 0},
	{.id = "usb0",  .bit = P6_SYS_CLK_USB0,  .offset = 0},
	{.id = "usb1",  .bit = P6_SYS_CLK_USB1,  .offset = 0},
	{.id = "sdio",  .bit = P6_SYS_CLK_SDIO,  .offset = 0},
	{.id = "parint",.bit = P6_SYS_CLK_PARINT,.offset = 0},
	{.id = "nand",  .bit = P6_SYS_CLK_NAND,  .offset = 0},
	{.id = "pwm",   .bit = P6_SYS_CLK_PWM,   .offset = 0},
	//{.id = "can",   .bit = P6_SYS_CLK_CAN,   .offset = 0},
	//{.id = "uartsim", .bit = P6_SYS_CLK_UARTSIM, .offset = 0},
	/* gpio */
	/* most */
	{.id = "spi0",  .bit = P6_SYS_CLK_SPI0,  .offset = 0},
	{.id = "spi1",  .bit = P6_SYS_CLK_SPI1,  .offset = 0},
	{.id = "spi2",  .bit = P6_SYS_CLK_SPI2,  .offset = 0},
	{.id = "uart0", .bit = P6_SYS_CLK_UART0, .offset = 0},
	{.id = "uart1", .bit = P6_SYS_CLK_UART1, .offset = 0},
	{.id = "uart2", .bit = P6_SYS_CLK_UART2, .offset = 0},
	{.id = "uart3", .bit = P6_SYS_CLK_UART3, .offset = 0},
	/* ahbmon */
	{.id = "i2cs",  .bit = P6_SYS_CLK_I2CS,  .offset = 0},
	{.id = "i2cm",  .bit = P6_SYS_CLK_I2CM0, .offset = 0},
	{.id = "i2cm1", .bit = P6_SYS_CLK_I2CM1, .offset = 0},

	{.id = "arm",   .bit = P6_SYS_CLK_ARM,   .offset = 0},
	{.id = "psd0",  .bit = P6_SYS_CLK_PSD0,  .offset = PSD_CLK_OFFSET},
	{.id = "psd1",  .bit = P6_SYS_CLK_PSD1,  .offset = PSD_CLK_OFFSET},
};
static DEFINE_SPINLOCK(clk_lock);

static unsigned long get_ahb_clk_p6(void)
{
	u32 fast;

	/* AHB clock can dynamically change, see above note about clocks */
	fast = __raw_readl(PARROT6_VA_SYS+_P6_SYS_SPEED) & P6_SYS_SPEED_156MHZ;

	return (fast)? PARROT6_PCLK_HS : PARROT6_PCLK_LS;
}

static unsigned long get_arm_clk_p6(void)
{
	u32 rst;
	unsigned long rate;

	rate = get_ahb_clk_p6();
	/* multiply ahb rate for cpu clock */
	rst = __raw_readl(PARROT6_VA_SYS + _P6_SYS_RESET);
	rate *= (rst & P6_SYS_RESET_A2)? 2 : 3;
	return rate;
}

static unsigned long get_arm_clk_p6i(void)
{
	unsigned long rate = PARROT6I_CLKIN;
	u32 speed;

	/* ARM / AHB clock can dynamically change, see above note about clocks */
	speed = __raw_readl(PARROT6_VA_SYS+_P6_SYS_SPEED) & 0x1f;
	BUG_ON(speed > 20);
	if (speed > 20)
		speed = 20;

	rate *= speed+16;
	return rate/2;
}

static unsigned long get_ahb_clk_p6i(void)
{
	unsigned long rate = get_arm_clk_p6i();

	if (rate > 312000000) {
		rate = (rate + 2) / 3;
	}
	else {
		rate /= 2;
	}
	return rate;
}

static unsigned long get_ahb_clk(void)
{
	if (parrot_chip_is_p6i())
		return get_ahb_clk_p6i();
	return get_ahb_clk_p6();
}
static unsigned long get_arm_clk(void)
{
	if (parrot_chip_is_p6i())
		return get_arm_clk_p6i();
	return get_arm_clk_p6();
}

int __init clocks_init_p6(void)
{
	printk("arm clk %lu Mhz, AHB bus %lu Mhz\n", get_arm_clk()/1000000,
			get_ahb_clk()/1000000);
	/* disable all clocks to save power and detect drivers
	 * not enabling clock, but keep uart0 for console
	 */
	__raw_writel(~(P6_SYS_CLK_ARM|
				P6_SYS_CLK_MPMC|
				P6_SYS_CLK_BUSMX|
				P6_SYS_CLK_INTROM|
#ifdef CONFIG_DEBUG_LL
				(P6_SYS_CLK_UART0<<CONFIG_DEBUG_PARROT_UART)|
#endif
				P6_SYS_CLK_GPIO),
			PARROT6_VA_SYS+_P6_SYS_CDIS);

	if (parrot_chip_is_p6()) {
		__raw_writel(P6_SYS_CLK_PSD0|P6_SYS_CLK_PSD1,
			     PARROT6_VA_SYS+_P6_SYS_PSD_CDIS);

		// could be override in the board specific code
		__raw_writel(P6_SYS_156MHZ_SDIO_22_3MHZ |
					 P6_SYS_156MHZ_PSD_22_3MHZ << 8 |
					 P6_SYS_156MHZ_PSD_22_3MHZ << 16,
					 PARROT6_VA_SYS+_P6_SYS_SDCARD);
	} else {
		/*
		 * Could be override in the board specific code and should be set to 39MHz for SDCard
		 * (this default value, 22.3MHz, is better for SDIO card)
		 */
		__raw_writel(P6_SYS_156MHZ_SDIO_22_3MHZ,
				 PARROT6_VA_SYS+_P6_SYS_SDCARD);
	}

	return 0;
}

/* Clock API calls */

struct clk *clk_get(struct device *dev, const char *id)
{
	int i;
	struct clk * clock = ERR_PTR(-ENOENT);

	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		if (strcmp(clocks[i].id, id) == 0) {
			clock = &clocks[i];
			break;
		}
	}

	return clock;
}

void clk_put(struct clk *clk)
{
}

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	if (IS_ERR(clk) || (clk == NULL)) {
		return -EINVAL;
	}

	/* Enable the clock in HW on first enable. */
	spin_lock_irqsave(&clk_lock, flags);
	if (clk->users == 0) {
		__raw_writel(clk->bit,
		             PARROT6_VA_SYS + _P6_SYS_CEN + clk->offset);
	}
	clk->users++;
	spin_unlock_irqrestore(&clk_lock, flags);
	return 0;
}

void clk_disable(struct clk *clk)
{
	unsigned long flags;
	if (IS_ERR(clk) || (clk == NULL)) {
		return;
	}
	/* Disable clock in HW when count of users reaches 0. */
	spin_lock_irqsave(&clk_lock, flags);
	if (clk->users > 0) {
		clk->users--;
	} else {
		WARN_ON(clk->users <= 0);
	}
	if (clk->users == 0) {
		__raw_writel(clk->bit,
		             PARROT6_VA_SYS + _P6_SYS_CDIS + clk->offset);
	}
	spin_unlock_irqrestore(&clk_lock, flags);
}

unsigned long clk_get_rate(struct clk *clk)
{
	u32 cstat;
	unsigned long rate;

	rate = get_ahb_clk();

	/* if NULL pointer assume ahb clock */
	if (IS_ERR(clk) || (clk == NULL)) {
		return rate;
	}

	cstat = __raw_readl(PARROT6_VA_SYS + _P6_SYS_CSTAT + clk->offset);

	if (cstat & clk->bit) {
		/* clock is enabled */

		/* arm */
		if ((clk->bit == P6_SYS_CLK_ARM) && (clk->offset == 0)) {
			rate = get_arm_clk();
		}

		/* sdcard */
		if (((clk-> bit == P6_SYS_CLK_SDIO) && (clk->offset == 0)) ||
			(clk->offset == PSD_CLK_OFFSET)) {

			u32 sd = __raw_readl(PARROT6_VA_SYS + _P6_SYS_SDCARD);
			int div = 3;
			if (clk->offset == PSD_CLK_OFFSET) {
				sd >>= (clk->bit == P6_SYS_CLK_PSD0)? 8 : 16;
				div = 6;
			}
			sd &= 0x7;

			rate = rate / (div + sd);
		}
	}
	else {
		/* clock is disabled */
		rate = 0;
	}

	return rate;
}

EXPORT_SYMBOL(clk_get);
EXPORT_SYMBOL(clk_put);
EXPORT_SYMBOL(clk_enable);
EXPORT_SYMBOL(clk_disable);
EXPORT_SYMBOL(clk_get_rate);
