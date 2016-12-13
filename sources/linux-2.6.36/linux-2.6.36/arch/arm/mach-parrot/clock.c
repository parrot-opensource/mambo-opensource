/*
 *  linux/arch/arm/mach-parrot/clock.c
 *
 *  Copyright (C) 2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2007-06-11
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

struct clk {
	const char* id;
	unsigned int enable_bit;
};

static unsigned long ahb_rate;

static const struct clk clk_uart0 = {
	.enable_bit           = P5_SYS_CEN_UART0CLK,
	.id = "uart0",
};

static const struct clk clk_uart1 = {
	.enable_bit           = P5_SYS_CEN_UART1CLK,
	.id = "uart1",
};

static const struct clk clk_uart2 = {
	.enable_bit           = P5_SYS_CEN_UART2CLK,
	.id = "uart2",
};

static const struct clk clk_i2cm = {
	.enable_bit           = P5_SYS_CEN_I2CMCLK,
	.id = "i2cm",
};

static const struct clk clk_pwm = {
	.enable_bit           = P5_SYS_CEN_PWMCLK,
	.id = "pwm",
};

static const struct clk clk_dmac = {
	.enable_bit           = P5P_SYS_CEN_DMACLK,
	.id = "dmac",
};

static const struct clk clk_psd0 = {
	.enable_bit           = P5_SYS_CEN_PSDCLK,
	.id = "psd0",
};

static const struct clk clk_psd1 = {
	.enable_bit           = P5P_SYS_CEN_PSD1CLK,
	.id = "psd1",
};

static const struct clk * const clocks[] = {
	&clk_uart0,
	&clk_uart1,
	&clk_uart2,
	&clk_i2cm,
	&clk_pwm,
	&clk_dmac,
	&clk_psd0,
	&clk_psd1,
};

static int clocks_init_p5(void)
{
	u32 rst;

	/* get reset configuration and chip ID */
	rst = __raw_readl(PARROT5_VA_SYS+_P5_SYS_RESET);

	if (parrot_chip_is_p5p() && (rst & P5_SYS_RESET_A16)) {
		/* overclock mode (130 MHz) */
		ahb_rate = PARROT5_PCLK_HS_OVERCLOCK;
	}
	else if (rst & P5_SYS_RESET_A2) {
		/* slow mode (78 MHz) */
		ahb_rate = PARROT5_PCLK_HS_SLOW;
	}
	else {
		/* normal mode (104 MHz) */
		ahb_rate = PARROT5_PCLK_HS;
	}

	/* disable all clocks to save power and detect driver
	 * not enabling clock, but keep uart0 for console
	 */
	__raw_writel(0x003ffff4,
		     PARROT5_VA_SYS+_P5_SYS_CDIS);

	return 0;
}
/* Clock API calls */

struct clk *clk_get(struct device *dev, const char *id)
{
	int i;
	const struct clk * clock = ERR_PTR(-ENOENT);
	for (i = 0; i < ARRAY_SIZE(clocks); i++) {
		if (strcmp(clocks[i]->id, id) == 0) {
			clock = clocks[i];
			break;
		}
	}

	/* the cast is need to remove the const qualifier. It
	 * is safe to do, as nobody touch it
	 */
	return (struct clk *)clock;
}

void clk_put(struct clk *clk)
{
}

int clk_enable(struct clk *clk)
{
	if (IS_ERR(clk) || clk == NULL)
		return -EINVAL;

	__raw_writel(clk->enable_bit,
		     PARROT5_VA_SYS+_P5_SYS_CEN);
	return 0;
}

void clk_disable(struct clk *clk)
{
	if (IS_ERR(clk) || clk == NULL)
		return;

	__raw_writel(clk->enable_bit,
		     PARROT5_VA_SYS+_P5_SYS_CDIS);
}

unsigned long clk_get_rate(struct clk *clk)
{
	int enabled;
	/* TODO move this to arch init code ??? */
	static int init;
	if (init == 0) {
		clocks_init_p5();
		init = 1;
	}

	/* if NULL pointer assume ahb clock */
	if (IS_ERR(clk) || clk == NULL)
		enabled = 1;
	else
		enabled = __raw_readl(PARROT5_VA_SYS+_P5_SYS_CSTAT) & (clk->enable_bit);
	return enabled?ahb_rate:0;
}

EXPORT_SYMBOL(clk_get);
EXPORT_SYMBOL(clk_put);
EXPORT_SYMBOL(clk_enable);
EXPORT_SYMBOL(clk_disable);
EXPORT_SYMBOL(clk_get_rate);
