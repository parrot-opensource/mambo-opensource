/*
 *  Copyright (C) 2015 Parrot S.A.
 *
 * @author     aurelien.lefebvre@parrot.com
 * @date       2015-04-20
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <mach/parrot.h>
#include <misc/dynamic_pinmux.h>

#define DRIVER_NAME "dynamic_pinmux"

static ssize_t dpinmux_show(struct device *dev,  struct device_attribute *attr, char *buf)
{
	struct dynamic_pinmux_platform_data *pdata;

	pdata = dev->platform_data;

	return sprintf(buf, "%d\n", pdata->current_mode);
}

static ssize_t dpinmux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) 
{
	struct dynamic_pinmux_platform_data *pdata;
	int value;

	pdata = dev->platform_data;

	sscanf(buf, "%d", &value);

	value = !!value;
	pdata->current_mode = value;

	if (value)
		parrot_init_pin(pdata->mux_mode_1);
	else
		parrot_init_pin(pdata->mux_mode_0);

	return count;
}

static DEVICE_ATTR(dpinmux, S_IRUSR | S_IWUSR, dpinmux_show, dpinmux_store);

static int __devinit dynamic_pinmux_probe(struct platform_device* pdev)
{
	struct dynamic_pinmux_platform_data *pdata;
	int err = 0;

	pdata = pdev->dev.platform_data;
	dev_attr_dpinmux.attr.name = pdata->name;

	if ((err = device_create_file(&pdev->dev, &dev_attr_dpinmux)))
		dev_err(&pdev->dev, "Failed to register sysfs file\n");

	return err;
}

static int __devexit dynamic_pinmux_remove(struct platform_device* pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_dpinmux);

	return 0;
}

static struct platform_driver dynamic_pinmux_driver = {
	.driver         = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
	.probe          = &dynamic_pinmux_probe,
	.remove         = &dynamic_pinmux_remove,
};

static int __init dynamic_pinmux_init(void)
{
	return platform_driver_register(&dynamic_pinmux_driver);
}
module_init(dynamic_pinmux_init);

static void __exit dynamic_pinmux_exit(void)
{
	platform_driver_unregister(&dynamic_pinmux_driver);
}
module_exit(dynamic_pinmux_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aurelien Lefebvre <aurelien.lefebvre@parrot.com>");
MODULE_DESCRIPTION("Dynamic pinmuxing for p6i");
