/* drivers/power/acpower_battery.c
 *
 * Power supply driver for the acpower emulator
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>


struct acpower_battery_data {
	uint32_t reg_base;
	int irq;
	spinlock_t lock;

	struct power_supply battery;
	struct power_supply ac;
	struct power_supply usb;
};


/* temporary variable used between acpower_battery_probe() and acpower_battery_open() */
static struct acpower_battery_data *battery_data;


static int acpower_usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int acpower_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int acpower_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1; 
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = 100;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property acpower_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property acpower_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property acpower_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};



static int acpower_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct acpower_battery_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);

	data->battery.properties = acpower_battery_props;
	data->battery.num_properties = ARRAY_SIZE(acpower_battery_props);
	data->battery.get_property = acpower_battery_get_property;
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;

	data->ac.properties = acpower_ac_props;
	data->ac.num_properties = ARRAY_SIZE(acpower_ac_props);
	data->ac.get_property = acpower_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;

	data->usb.properties = acpower_usb_props;
	data->usb.num_properties = ARRAY_SIZE(acpower_usb_props);
	data->usb.get_property = acpower_usb_get_property;
	data->usb.name = "usb";
	data->usb.type = POWER_SUPPLY_TYPE_USB;

	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret)
		goto err_ac_failed;

	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
		goto err_battery_failed;

	ret = power_supply_register(&pdev->dev, &data->usb);
	if (ret)
		goto err_usb_failed;
	
	platform_set_drvdata(pdev, data);
	battery_data = data;

	return 0;


err_usb_failed:
	power_supply_unregister(&data->usb);
err_battery_failed:
	power_supply_unregister(&data->ac);
err_ac_failed:
	kfree(data);
err_data_alloc_failed:
	return ret;
}

static int acpower_battery_remove(struct platform_device *pdev)
{
	struct acpower_battery_data *data = platform_get_drvdata(pdev);

	power_supply_unregister(&data->battery);
	power_supply_unregister(&data->ac);
	power_supply_unregister(&data->usb);

	kfree(data);
	battery_data = NULL;
	return 0;
}

static struct platform_driver acpower_battery_device = {
	.probe		= acpower_battery_probe,
	.remove		= acpower_battery_remove,
	.driver = {
		.name = "p6-acpower"
	}
};

static int __init acpower_battery_init(void)
{
	return platform_driver_register(&acpower_battery_device);
}

static void __exit acpower_battery_exit(void)
{
	platform_driver_unregister(&acpower_battery_device);
}

module_init(acpower_battery_init);
module_exit(acpower_battery_exit);

MODULE_AUTHOR("Gregoire ETIENNE, <gregoire.etienne@parrot.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AC driver for the Parrot products");
