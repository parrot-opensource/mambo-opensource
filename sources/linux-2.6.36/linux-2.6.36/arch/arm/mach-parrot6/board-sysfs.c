#include <linux/string.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <linux/platform_device.h>

#include "board-sysfs.h"

struct hw_info_i2c {
	int            bus;
	unsigned short addr;
};
struct hw_info_uart {
	int            bus;
	int            cts_rts;
};
struct hw_info {
	char           *type;
	char           *name;
	char           *version;
	union {
		struct hw_info_i2c i2c;
		struct hw_info_uart uart;
	} data;
};

static int match_device(struct device *dev, void *data)
{
	struct hw_info *infos = dev_get_drvdata(dev);
	return !strcmp(infos->name, data);
}
static ssize_t hw_show_info(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct hw_info *infos = dev_get_drvdata(dev);
	if (infos == NULL)
		return -EINVAL;
	if (attr == NULL)
		return -EINVAL;
	if (!strcmp(attr->attr.name, "type"))
		return sprintf(buf, "%s\n", infos->type);
	if (!strcmp(attr->attr.name, "name"))
		return sprintf(buf, "%s\n", infos->name);
	if (!strcmp(attr->attr.name, "version"))
		return sprintf(buf, "%s\n", infos->version);
	if (!strcmp(attr->attr.name, "i2c_bus"))
		return sprintf(buf, "%d\n", infos->data.i2c.bus);
	if (!strcmp(attr->attr.name, "i2c_addr"))
		return sprintf(buf, "0x%02X\n", infos->data.i2c.addr);
	if (!strcmp(attr->attr.name, "uart_bus"))
		return sprintf(buf, "%d\n", infos->data.uart.bus);
	if (!strcmp(attr->attr.name, "uart_cts_rts"))
		return sprintf(buf, "%d\n", infos->data.uart.cts_rts);
	printk(KERN_ERR "Hardware infos : Invalid property %s\n",
	       attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(type, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(name, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(version, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(i2c_bus, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(i2c_addr, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(uart_bus, S_IRUGO, hw_show_info, NULL);
static DEVICE_ATTR(uart_cts_rts, S_IRUGO, hw_show_info, NULL);

struct hw_info *get_hw_info_new_device(const char *name, char *type)
{
	static struct class *hw_class;
	dev_t hw_info_dev;
	struct device *dev;
	struct hw_info *dev_hw_infos;
	const char *device_name;
	char unique_name[32];
	int item_nb = 0;
	int ret __maybe_unused;

	/* If class doe not exists, create it */
	if (hw_class == NULL) {
		hw_class = class_create(NULL, "hwdb");
		if (IS_ERR(hw_class))
			return NULL;
	}

	/* If necessary, use default name */
	if (name == NULL || strlen(name) == 0)
		device_name = "unknown";
	else
		device_name = name;

	/* Check if 'name' device exists, try from 'name'1 to 'name'16 */
	strncpy(unique_name, device_name, sizeof(unique_name));
	unique_name[sizeof(unique_name)-2] = '\0';
	while(class_find_device(hw_class, NULL, unique_name, match_device)) {
		item_nb++;
		if (item_nb > 16)
			return NULL;
		snprintf(unique_name, sizeof(unique_name), "%s%d",
			 device_name, item_nb);
	}

	/* Create new device for this element */
	hw_info_dev = MKDEV(0, 0);
	dev = device_create(hw_class, NULL, hw_info_dev, NULL, unique_name);

	/* Export needed file entries */
	ret = device_create_file(dev, &dev_attr_type);
	ret = device_create_file(dev, &dev_attr_name);
	ret = device_create_file(dev, &dev_attr_version);
	if (!strcmp(type, "uart")) {
		ret = device_create_file(dev, &dev_attr_uart_cts_rts);
		ret = device_create_file(dev, &dev_attr_uart_bus);
	}
	else if (!strcmp(type, "i2c")) {
		ret = device_create_file(dev, &dev_attr_i2c_bus);
		ret = device_create_file(dev, &dev_attr_i2c_addr);
	}

	/* Register device infos in device */
	dev_hw_infos = kmalloc(sizeof(struct hw_info), GFP_KERNEL);
	if (dev_hw_infos == NULL)
		return NULL;
	dev_set_drvdata(dev, dev_hw_infos);

	/* Give back device infos to allow completion */
	return dev_hw_infos;
}

/**
 * p6i_export_i2c_hw_infos - Create sysfs entries
 *                             for i2c user hw information.
 */
void __init p6i_export_i2c_hw_infos(int i2c_bus, unsigned short addr,
				      char *version, const char *name)
{
	struct hw_info *new_i2c_entry;
	printk(KERN_INFO "hw_info : i2c-%d device 0x%02X : %s (%s)\n",
	       i2c_bus, addr, name, version);
	new_i2c_entry = get_hw_info_new_device(name, "i2c");
	if (new_i2c_entry == NULL)
		return;
	new_i2c_entry->type = "i2c";
	new_i2c_entry->name = kstrdup(name, GFP_KERNEL);
	new_i2c_entry->version = kstrdup(version, GFP_KERNEL);
	new_i2c_entry->data.i2c.bus = i2c_bus;
	new_i2c_entry->data.i2c.addr = addr;
}


/**
 * p6i_export_uart_hw_infos - Create sysfs entries
 *                             for uart user hw information.
 */
void __init p6i_export_uart_hw_infos(int uart, int rts_cts, char *version)
{
	struct hw_info *new_uart_entry;
	char unid[32];
	printk(KERN_INFO "hw_info : uart %d rts/cts %d\n",
	       uart, rts_cts);
	snprintf(unid, sizeof(unid), "uart-%d", uart);
	new_uart_entry = get_hw_info_new_device(unid, "uart");
	if (new_uart_entry == NULL)
		return;
	new_uart_entry->type = "uart";
	new_uart_entry->name = kstrdup(unid, GFP_KERNEL);
	new_uart_entry->version = kstrdup(version, GFP_KERNEL);
	new_uart_entry->data.uart.bus = uart;
	new_uart_entry->data.uart.cts_rts = rts_cts;
}
