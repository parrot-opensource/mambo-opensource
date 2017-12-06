/*
 * p6_sdhci.c
 *
 * Parrot6 SDHCI driver
 *
 * Copyright (C) 2009 Parrot S.A.
 *
 * @author     florent.bayendrian@parrot.com
 * @date       2009-04-09
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

#include <linux/mmc/host.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <mach/gpio.h>
#include <mach/mmc.h>

#include "../../mmc/host/sdhci.h"

#define DRV_NAME		"p6-sdhci"

#define SLOT_WIDTH		0x100

struct driver_data {
	struct clk			*clk;
	resource_size_t			pbase;
	resource_size_t			base_size;
	struct parrot_mmc_platform_data	*pdata;
	struct mmc_host_ops		ops;
};

static int p6_sdhci_get_ro(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct driver_data *drv_data = sdhci_priv(host);
	int ret = -ENOSYS;

	if (drv_data->pdata && gpio_is_valid(drv_data->pdata->wp_pin)) {
		ret =  gpio_get_value(drv_data->pdata->wp_pin);
	}

	return ret;
}

static int p6_sdhci_get_cd(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct driver_data *drv_data = sdhci_priv(host);
	int ret;

	ret =  gpio_get_value(drv_data->pdata->cd_pin);

	return !ret;
}

static unsigned int p6_sdhci_get_max_clk(struct sdhci_host *host)
{
	struct driver_data *drv_data = sdhci_priv(host);
	unsigned int max_clk;

#if !defined(CONFIG_VERSATILE_PARROT6)
	max_clk = clk_get_rate(drv_data->clk);
#else
	max_clk = 35000000; // check your FPGA configuration for that
#endif

	return max_clk;
}

static struct sdhci_ops p6_sdhci_ops = {
	.enable_dma = NULL,
	.get_max_clock		= p6_sdhci_get_max_clk,
};

static int __devinit p6_sdhci_probe(struct platform_device *pdev)
{
	struct parrot_mmc_platform_data *pdata = pdev->dev.platform_data;
	struct sdhci_host *host;
	struct driver_data *drv_data;
	struct resource *res_mem, *res_irq;
	int ret;
	int do_cd = 0;

	if (pdev->id > 2) {
		ret = -ENODEV;
		goto out;
	}
	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem) {
		ret = -ENXIO;
		goto out;
	}
	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res_irq) {
		ret = -ENXIO;
		goto out;
	}

	host = sdhci_alloc_host(&pdev->dev, sizeof(struct driver_data));
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		goto out;
	}

	drv_data = sdhci_priv(host);
	drv_data->pdata = pdata;

#if !defined(CONFIG_VERSATILE_PARROT6)
	drv_data->clk = clk_get(&pdev->dev, "sdio");
	if (IS_ERR(drv_data->clk)) {
		ret = PTR_ERR(drv_data->clk);
		goto no_clk;
	}
	clk_enable(drv_data->clk);
#endif

	host->hw_name = DRV_NAME;
	host->ops = &p6_sdhci_ops;
	drv_data->pbase = res_mem->start;
	drv_data->base_size = res_mem->end - res_mem->start + 1;
	if (!request_mem_region(drv_data->pbase, drv_data->base_size, DRV_NAME)) {
		ret = -EBUSY;
		goto no_req_mem;
	}

	host->ioaddr = ioremap(drv_data->pbase, drv_data->base_size);
	if (!host->ioaddr) {
		ret = -ENOMEM;
		goto no_ioremap;
	}

	if (drv_data->pdata) {
		if (gpio_is_valid(drv_data->pdata->wp_pin))
			gpio_direction_input(drv_data->pdata->wp_pin);
		if (gpio_is_valid(drv_data->pdata->cd_pin)) {
			gpio_direction_input(drv_data->pdata->cd_pin);
			do_cd = 1;
		}
	}

	host->irq = res_irq->start;

	/*
	 * Controller does not support unaligned buffer start address and size for
	 * ADMA mode (see section 2.9 of Arasan SD3.0 / SDIO3.0 / eMMC4.41 AHB Host
	 * Controller user guide).
	 */
	host->quirks = SDHCI_QUIRK_32BIT_DMA_ADDR | SDHCI_QUIRK_32BIT_ADMA_SIZE;

	/* Timeout clock (TMCLK) is the same as sd clock.
	 * Note: sometimes a device may specify timeouts that are larger than the
	 * highest possible timeout in the host controller. In this case Linux
	 * generates a warning you may inhibit using the
	 * SDHCI_QUIRK_BROKEN_TIMEOUT_VAL (see sdhci_calc_timeout).
	 */
	host->quirks |= SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK;

	host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
	/* force using our freq */
	host->quirks |= SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN | SDHCI_QUIRK_FORCE_BLK_SZ_2048;

	/* Used to fix the hold time of the SD card */
	host->quirks |= SDHCI_QUIRK_NO_HISPD_BIT;

	/* Use cache if available */
	host->mmc->caps = MMC_CAP_CACHE_CTRL;

	// hack: lock the mmc to change some values just after sdhci_add_host
	mmc_claim_host(host->mmc);
	ret = sdhci_add_host(host);
	if (ret) {
		mmc_release_host(host->mmc);
		goto no_add_host;
	}
	// redefine the get_ro callback by our new one
	drv_data->ops = *host->mmc->ops;
	drv_data->ops.get_ro = p6_sdhci_get_ro;
	if (do_cd) {
		drv_data->ops.get_cd = p6_sdhci_get_cd;
	}
	host->mmc->ops = &drv_data->ops;
	// now we could allow access to the mmc struct
	mmc_release_host(host->mmc);

	platform_set_drvdata(pdev, host);

	return 0;

no_add_host:
	iounmap(host->ioaddr);
no_ioremap:
	release_mem_region(drv_data->pbase, drv_data->base_size);
no_req_mem:
	clk_disable(drv_data->clk);
	clk_put(drv_data->clk);
no_clk:
	sdhci_free_host(host);
out:
	return ret;
}

static int __devexit p6_sdhci_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);

	if (host) {
		struct driver_data *drv_data = sdhci_priv(host);

		cancel_delayed_work_sync(&host->mmc->detect);
		sdhci_remove_host(host, 0);
		iounmap(host->ioaddr);
		release_mem_region(drv_data->pbase, drv_data->base_size);
		clk_disable(drv_data->clk);
		clk_put(drv_data->clk);
		sdhci_free_host(host);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

#ifdef CONFIG_PM
static int p6_sdhci_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct driver_data *drv_data = sdhci_priv(host);

	clk_disable(drv_data->clk);
	return sdhci_suspend_host(host, state);
}

static int p6_sdhci_resume(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct driver_data *drv_data = sdhci_priv(host);

	clk_enable(drv_data->clk);
	return sdhci_resume_host(host);
}

#else
#define p6_sdhci_suspend NULL
#define p6_sdhci_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver p6_sdhci_driver = {
	.probe          = p6_sdhci_probe,
	.remove         = __devexit_p(p6_sdhci_remove),
	.suspend        = p6_sdhci_suspend,
	.resume         = p6_sdhci_resume,
	.driver         = {
		.name   = DRV_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init p6_sdhci_drv_init(void)
{
	printk(KERN_INFO "Parrot6 SDHCI driver $Revision: 1.7 $\n");
	return platform_driver_register(&p6_sdhci_driver);
}

static void __exit p6_sdhci_drv_exit(void)
{
	platform_driver_unregister(&p6_sdhci_driver);
}

module_init(p6_sdhci_drv_init);
module_exit(p6_sdhci_drv_exit);

MODULE_AUTHOR("Florent Bayendrian <florent.bayendrian@parrot.com>");
MODULE_DESCRIPTION("Parrot6 Secure Digital Host Controller Interface driver");
MODULE_LICENSE("GPL");
