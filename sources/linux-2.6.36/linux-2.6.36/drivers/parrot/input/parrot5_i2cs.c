/**
 * @file parrot5_i2cs.c
 *
 * @brief Parrot I2C Slave driver
 *
 * Copyright (C) 2010 Parrot S.A.
 *
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

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/input.h>

#include <asm/io.h>
#include <mach/parrot.h>

#define PARROT5_I2CS_NAME "parrot5-i2cs"
#define DRIVER_NAME PARROT5_I2CS_NAME
#define DRIVER_VERSION 0x001

#define I2CS_ADDRESS 0
#define I2CS_ITEN 4
#define I2CS_ITEN_ITEN 3
#define I2CS_ITEN_ITDIS 0
#define I2CS_ITACK_ITACK 8
#define I2CS_ITACK 1
#define I2CS_TRANSMIT 0xc
#define I2CS_RECEIVE 0x20

struct parrot5_i2cs_data {
	struct input_dev				*input;
	struct parrot5_i2cs_platform	*pdata;

	struct clk					*clk;

	struct resource				*ioarea;
	void __iomem				*base;
};


/**
 * Interrupt functions
 */
static void parrot5_i2cs_enable_irq(struct parrot5_i2cs_data *drv_data)
{
	writel(I2CS_ITEN_ITEN, drv_data->base + I2CS_ITEN);
}


static void parrot5_i2cs_disable_irq(struct parrot5_i2cs_data *drv_data)
{
	writel(I2CS_ITEN_ITDIS, drv_data->base + I2CS_ITEN);
}

static irqreturn_t parrot5_i2cs_irq(int irq, void *dev_id)
{
	struct parrot5_i2cs_data *drv_data = dev_id;

	unsigned int data = readl(drv_data->base + I2CS_RECEIVE);

	if ((data & 0x100) == 0) {
		while ((data & 0x100) == 0) {
			data &= 0xff;
			printk("receive %x\n", data);
			data = readl(drv_data->base + I2CS_RECEIVE);
			/* XXX hack to set supported key */
			set_bit(data & 0x7f, drv_data->input->keybit);
			/* send key to input layer */
			input_report_key(drv_data->input, data & 0x7f, data & 0x80);
			input_sync(drv_data->input);
		}
	}
	else {
		printk("tx interrupt\n");
		/* XXX always read 0xde */
		writel(0xde, drv_data->base + I2CS_TRANSMIT);
		writel(I2CS_ITACK, drv_data->base + I2CS_ITACK_ITACK);
	}

	return IRQ_HANDLED;
}


/**
 * Hardware init
 */
static int parrot5_i2cs_hw_init(struct parrot5_i2cs_data *drv_data)
{
	int ret = 0;
	const char *const clk_name = "i2cs";

	/* enable I2C master clock */
	drv_data->clk = clk_get(NULL, clk_name);
	if (IS_ERR(drv_data->clk)) {
		ret = -EINVAL;
		goto exit;
	}
	clk_enable(drv_data->clk);

	/* XXX hardcoded address pass it, via platform data */
	writel(0x45, drv_data->base + I2CS_ADDRESS);

	/* interrupt mode  */
	parrot5_i2cs_enable_irq(drv_data);

exit:
	return ret;
}

/**
 * probe function
 */
static int parrot5_i2cs_probe(struct platform_device *pdev)
{
	struct parrot5_i2cs_data *drv_data;
	struct input_dev *input_dev;
	struct resource *res;
	int ret;

	drv_data = kzalloc(sizeof(struct parrot5_i2cs_data), GFP_KERNEL);
	if (!drv_data) {
		ret = -ENOMEM;
		goto no_mem;
	}

	drv_data->pdata = pdev->dev.platform_data;
	if (!drv_data->pdata) {
		dev_err(&pdev->dev, "no driver data\n");
		ret = -ENODEV;
		goto no_pdata;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "get memory region resource failed\n");
		ret = -ENOENT;
		goto no_res;
	}

	drv_data->ioarea = request_mem_region(res->start,
						  res->end - res->start + 1,
						  pdev->name);
	if (!drv_data->ioarea) {
		dev_err(&pdev->dev, "request memory region failed\n");
		ret = -ENOENT;
		goto no_res;
	}

	drv_data->base = ioremap(res->start, res->end - res->start + 1);
	if (!drv_data->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto no_map;
	}

	/* input init */
	input_dev = input_allocate_device();

	if (!input_dev) {
		ret = -ENOMEM;
		goto input_failure;
	}
	drv_data->input = input_dev;
	input_dev->name = DRIVER_NAME;
	input_dev->phys = DRIVER_NAME"/input0";

	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = DRIVER_VERSION;
	input_dev->evbit[0] = BIT(EV_KEY);
	input_dev->evbit[0] |= BIT(EV_REP);
	while (0) {
		set_bit(/*keycode*/0, input_dev->keybit);
	}
	ret = input_register_device(drv_data->input);
	if (ret)
		goto input_register_failure;

	ret = request_irq(platform_get_irq(pdev, 0),
			  parrot5_i2cs_irq, IRQF_SHARED,
			  pdev->name, drv_data);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed (%d)\n", ret);
		goto no_irq;
	}

	ret = parrot5_i2cs_hw_init(drv_data);
	if (ret) {
		dev_err(&pdev->dev, "Hardware init failed (%d)\n", ret);
		goto err_hw;
	}

	platform_set_drvdata(pdev, drv_data);

	dev_info(&pdev->dev, "controller probe successfully\n");

	return 0;

err_hw:
	free_irq(platform_get_irq(pdev, 0), drv_data);
no_irq:
	input_unregister_device(drv_data->input);
input_register_failure:
	input_free_device(drv_data->input);
input_failure:
no_map:
	iounmap(drv_data->base);
	release_resource(drv_data->ioarea);
no_res:
	kfree(drv_data);

no_pdata:
no_mem:
	return ret;

}


/**
 * remove function
 */
static int parrot5_i2cs_remove(struct platform_device *pdev)
{
	struct parrot5_i2cs_data *drv_data = platform_get_drvdata(pdev);

	input_unregister_device(drv_data->input);
	input_free_device(drv_data->input);

	parrot5_i2cs_disable_irq(drv_data);
	clk_disable(drv_data->clk);
	clk_put(drv_data->clk);
	free_irq(platform_get_irq(pdev, 0), drv_data);

	iounmap(drv_data->base);

	release_resource(drv_data->ioarea);

	kfree(drv_data);

	platform_set_drvdata(pdev, NULL);

	return 0;
}


/**
 * platform driver structure
 */
static struct platform_driver parrot5_i2cs_driver = {
	.probe		= parrot5_i2cs_probe,
	.remove		= parrot5_i2cs_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PARROT5_I2CS_NAME,
	},
};


/**
 * init function
 */
static int __init parrot5_i2cs_init(void)
{
	return platform_driver_register(&parrot5_i2cs_driver);
}
module_init(parrot5_i2cs_init);


/**
 * exit function
 */
static void __exit parrot5_i2cs_exit(void)
{
	platform_driver_unregister(&parrot5_i2cs_driver);
}
module_exit(parrot5_i2cs_exit);



MODULE_AUTHOR("Parrot SA");
MODULE_DESCRIPTION("Parrot I2C Slave driver");
MODULE_LICENSE("GPL");
