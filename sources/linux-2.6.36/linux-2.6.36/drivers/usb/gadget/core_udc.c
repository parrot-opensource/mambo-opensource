#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

static struct usb_gadget_udc *usb_gadget_udc;

int usb_gadget_register_udc(void *priv, struct usb_gadget_udc *callback)
{
	if (!callback->register_driver || !callback->unregister_driver)
		return -EINVAL;

	if (usb_gadget_udc && usb_gadget_udc != callback)
		return -EBUSY;

	usb_gadget_udc = callback;
	usb_gadget_udc->priv = priv;

	return 0;
}
EXPORT_SYMBOL (usb_gadget_register_udc);

int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
	if (!usb_gadget_udc)
		return -ENODEV;
	return usb_gadget_udc->register_driver(driver, usb_gadget_udc->priv);
}
EXPORT_SYMBOL (usb_gadget_register_driver);

int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	if (!usb_gadget_udc)
		return -ENODEV;
	return usb_gadget_udc->unregister_driver(driver, usb_gadget_udc->priv);
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);
