/*
 * Copyright (c) 2005 MontaVista Software
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Ported to 834x by Randy Vinson <rvinson@mvista.com> using code provided
 * by Hunter Wu.
 */

#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <mach/usb-p6i.h>

#include "ehci-p6i.h"

/* FIXME: Power Management is un-ported so temporarily disable it */
#undef CONFIG_PM


/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * usb_hcd_p6p_probe - initialize P6P-based HCDs
 * @drvier: Driver to be used for this HCD
 * @pdev: USB Host Controller being probed
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 */
int usb_hcd_p6p_probe(const struct hc_driver *driver,
		      struct platform_device *pdev)
{
	struct p6i_usb2_platform_data_s *pdata;
	struct usb_hcd *hcd;
	struct resource *res;
	int irq;
	int retval;

	pr_debug("initializing P6P-SOC USB Controller\n");

#if 1
	/* Need platform data for setup */
	pdata = (struct p6i_usb2_platform_data_s *)pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev,
			"No platform data for %s.\n", dev_name(&pdev->dev));
		return -ENODEV;
	}

	/*
	 * This is a host mode driver, verify that we're supposed to be
	 * in host mode.
	 */
	if (!((pdata->operating_mode == P6P_USB2_DR_HOST) ||
	      (pdata->operating_mode == P6P_USB2_MPH_HOST) ||
	      (pdata->operating_mode == P6P_USB2_DR_OTG))) {
		dev_err(&pdev->dev,
			"Non Host Mode configured for %s. Wrong driver linked.\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
#endif

	{
	static const char *clk_name = "usb0";
	struct clk *usb_clk;

	/* enable clock */
	usb_clk = clk_get(NULL, clk_name);
	if (IS_ERR(usb_clk)) {
		return -EIO;
	}
	clk_enable(usb_clk);

	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto err2;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto err2;
	}
	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);

	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto err3;
	}

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0)
		goto err4;
	return retval;

      err4:
	iounmap(hcd->regs);
      err3:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
      err2:
	usb_put_hcd(hcd);
      err1:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_p6p_remove - shutdown processing for P6P-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_p6p_probe().
 *
 */
void usb_hcd_p6p_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
}

static void p6p_setup_portsc(struct ehci_hcd *ehci,
			      enum p6i_usb2_phy_modes phy_mode,
			      unsigned int force_full_speed,
			      unsigned int port_offset)
{
	u32 portsc = 0;
	switch (phy_mode) {
	case P6P_USB2_PHY_ULPI:
		portsc |= PORT_PTS_ULPI;
		break;
	case P6P_USB2_PHY_SERIAL:
		portsc |= PORT_PTS_SERIAL;
		break;
	case P6P_USB2_PHY_UTMI_WIDE:
		portsc |= PORT_PTS_PTW;
		/* fall through */
	case P6P_USB2_PHY_UTMI:
		portsc |= PORT_PTS_UTMI;
		break;
	case P6P_USB2_PHY_NONE:
		break;
	}

	if (force_full_speed == 1)
		portsc |= PORT_PFSC;

	ehci_writel(ehci, portsc, &ehci->regs->port_status[port_offset]);
}

static void p6p_usb_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	struct p6i_usb2_platform_data_s *pdata;

	pdata = (struct p6i_usb2_platform_data_s *)hcd->self.controller->platform_data;

	if ((pdata->operating_mode == P6P_USB2_DR_HOST) ||
           (pdata->operating_mode == P6P_USB2_DR_OTG))
		p6p_setup_portsc(ehci, pdata->phy_mode,
				 pdata->force_full_speed, 0);
}

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_p6p_reinit(struct ehci_hcd *ehci)
{
	p6p_usb_setup(ehci_to_hcd(ehci));
	ehci_port_power(ehci, 0);

	return 0;
}

/* called during probe() after chip reset completes */
static int ehci_p6p_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100 +
	    HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);
	hcd->has_tt = 1;

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;


	ehci->sbrn = 0x20;

	ehci_reset(ehci);

	retval = ehci_p6p_reinit(ehci);

	return retval;
}

static const struct hc_driver ehci_p6p_hc_driver = {
	.description = hcd_name,
	.product_desc = "Parrot 6+ On-Chip EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_p6p_setup,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,
	.endpoint_reset = ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,

	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

static int ehci_p6p_drv_probe(struct platform_device *pdev)
{
	if (usb_disabled())
		return -ENODEV;

	/* FIXME we only want one one probe() not two */
	return usb_hcd_p6p_probe(&ehci_p6p_hc_driver, pdev);
}

static int ehci_p6p_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	/* FIXME we only want one one remove() not two */
	usb_hcd_p6p_remove(hcd, pdev);
	return 0;
}

MODULE_ALIAS("platform:p6i-ehci");

static struct platform_driver ehci_p6i_driver = {
	.probe = ehci_p6p_drv_probe,
	.remove = ehci_p6p_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		   .name = "p6i-ehci",
	},
};
