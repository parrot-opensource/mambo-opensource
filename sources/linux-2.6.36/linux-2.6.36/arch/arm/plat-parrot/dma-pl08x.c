/**
 * @file linux/arch/arm/plat-parrot/dma-pl08x.c
 *
 * ARM PL080/PL081 DMA controller driver
 *
 * Copyright (C) 2008 Parrot S.A.
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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <mach/dma-pl08x.h>
#include <mach/regs-pl08x.h>

#define PL080_TIMEOUT           (1000)

struct pl08x_dma_channel {
	unsigned char __iomem   *cxbase;
	const char              *devid;
	pl08x_dma_callback_t     callback;
	void                    *data;
	int                      busy;
	int                      allocated;
};

struct pl08x_dev {
	unsigned char __iomem   *base;
	struct clk              *clk;
	unsigned int             irq;
	unsigned int             nb_chans;
	struct pl08x_dma_channel chan[_PL080_MAX_CHANNELS];
	const int               (*flow)[_PL080_PERIPH_MAX][_PL080_PERIPH_MAX];
};

static struct pl08x_dev dmac = {.nb_chans = 0};

static irqreturn_t pl08x_dma_irq(int irq, void *dev_id)
{
	u32 tc, err;
	int i, chan_err, chan_tc;
	struct pl08x_dma_channel *chan;

	/* check for completion and errors */
	tc = __raw_readl(dmac.base+_PL080_INTTCSTATUS);
	err = __raw_readl(dmac.base+_PL080_INTERRORSTATUS);

	for (i = 0; i < dmac.nb_chans; i++) {

		chan_tc = (tc & (1 << i));
		chan_err = (err & (1 << i));

		if (chan_err) {
			/* clear error */
			__raw_writel(1 << i, dmac.base+_PL080_INTERRCLR);
			printk(KERN_ERR "dma%d: tc=%u err=%u\n", i, tc, err);
		}

		if (chan_tc) {
			/* transfer completed */
			__raw_writel(1 << i, dmac.base+_PL080_INTTCCLEAR);
		}

		if (chan_tc || chan_err) {
			chan = &dmac.chan[i];
			if (chan->busy && chan->callback) {
				chan->callback(i, chan->data, chan_err);
			}
			/* transfer has completed (maybe with errors) */
			chan->busy = 0;
		}
	}

	return IRQ_HANDLED;
}

/**
 * Allocate a DMA channel.
 *
 * @channel: allocated channel
 * @devid: client device string identification
 * @callback: transfer completion callback (can be NULL)
 * @data: callback private data
 * @return: zero in case of success and a negative error code in
 * case of failure.
 */
int pl08x_dma_request(unsigned int *channel, const char *devid,
		      pl08x_dma_callback_t callback, void *data)
{
	int ret = -EBUSY;
	unsigned int i;
	unsigned long flags;
	struct pl08x_dma_channel *chan;

	local_irq_save(flags);

	/* look for an available channel */
	for (i = 0; i < dmac.nb_chans; i++) {
		chan = &dmac.chan[i];

		if (!chan->allocated) {
			*channel = i;
			chan->allocated = 1;
			chan->devid = devid;
			chan->callback = callback;
			chan->data = data;
			ret = 0;
			break;
		}
	}

	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(pl08x_dma_request);

/**
 * Free an allocated DMA channel.
 *
 * @channel: channel to free
 * @return: zero in case of success and a negative error code in
 * case of failure.
 */
int pl08x_dma_free(unsigned int channel)
{
	int ret = 0;
	unsigned long flags;
	struct pl08x_dma_channel *chan = &dmac.chan[channel];

	local_irq_save(flags);

	if ((channel < dmac.nb_chans) && chan->allocated && !chan->busy) {
		chan->allocated = 0;
	}
	else {
		ret = -EINVAL;
	}

	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(pl08x_dma_free);

/**
 * Start a DMA transfer.
 *
 * @channel: channel to start
 * @cfg: configuration parameters
 * @return: zero in case of success and a negative error code in
 * case of failure.
 */
int pl08x_dma_start(unsigned int channel, struct pl08x_dma_cfg *cfg)
{
	u32 cxcfg;
	union pl08x_dma_cxctrl cxctrl;
	unsigned long flags;
	int flowctrl, ret = 0;
	struct pl08x_dma_channel *chan = &dmac.chan[channel];

	/* sanity checks */
	if ((cfg->src_periph >= _PL080_PERIPH_MAX) ||
	    (cfg->dst_periph >= _PL080_PERIPH_MAX) ||
	    (channel >= dmac.nb_chans)) {
		ret = -EINVAL;
		goto badparam;
	}
	flowctrl = (*dmac.flow)[cfg->src_periph][cfg->dst_periph];
	if (flowctrl < 0) {
		ret = -EINVAL;
		goto badparam;
	}

	cxcfg =	_PL080_CXCONFIG_ENABLE |
		(cfg->src_periph << bs_PL080_CXCONFIG_SRCPERIPH) |
		(cfg->dst_periph << bs_PL080_CXCONFIG_DSTPERIPH) |
		(flowctrl        << bs_PL080_CXCONFIG_FLOWCNTRL);

	/* adjust control */
	cxctrl = cfg->cxctrl;
	cxctrl.sahb = 0;
	cxctrl.dahb = 0;
	cxctrl.i = 1;

	local_irq_save(flags);

	if (chan->allocated && !chan->busy) {

		if (chan->callback) {
			/* use DMAC interrupts */
			cxcfg |= _PL080_CXCONFIG_ITERROR|_PL080_CXCONFIG_ITTC;
		}

		/* configure and start channel */
		__raw_writel(cfg->src_addr, chan->cxbase+_PL080_CXSRCADDR);
		__raw_writel(cfg->dst_addr, chan->cxbase+_PL080_CXDESTADDR);
		__raw_writel(cfg->lli,      chan->cxbase+_PL080_CXLLI);
		__raw_writel(cxctrl.word,   chan->cxbase+_PL080_CXCTRL);
		__raw_writel(cxcfg,         chan->cxbase+_PL080_CXCONFIG);

		chan->busy = 1;
	}
	else {
		ret = -EINVAL;
	}

	local_irq_restore(flags);

badparam:
	return ret;
}
EXPORT_SYMBOL(pl08x_dma_start);

/**
 * Abort current DMA transfer.
 *
 * Note: The channel callback is not called after an aborted transfer.
 *
 * @channel: channel to start
 * @return: zero in case of success and a negative error code in
 * case of failure.
 */
int pl08x_dma_abort(unsigned int channel)
{
	int ret = 0;
	unsigned long flags;
	struct pl08x_dma_channel *chan = &dmac.chan[channel];

	local_irq_save(flags);

	if ((channel >= dmac.nb_chans) || !chan->allocated) {
		ret = -EINVAL;
		goto badparam;
	}

	if (chan->busy) {

		/* disable channel and potentially lose data in FIFOs */
		__raw_writel(0, chan->cxbase+_PL080_CXCONFIG);
		/* clear status */
		__raw_writel(1 << channel, dmac.base+_PL080_INTTCCLEAR);
		__raw_writel(1 << channel, dmac.base+_PL080_INTERRCLR);

		chan->busy = 0;
	}

badparam:
	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(pl08x_dma_abort);

/**
 * Wait for current DMA transfer completion.
 *
 * Note: This is a busy wait with interrupts disabled; it should be used only
 * to make sure DMA has fully stopped when transfer completion was signalled
 * through other means.
 *
 * @channel: DMA channel
 * @return: zero in case of success and a negative error code in
 * case of failure.
 */
int pl08x_dma_wait(unsigned int channel)
{
	int ret = 0;
	unsigned long flags;
	u32 tc, err, timeout;
	struct pl08x_dma_channel *chan = &dmac.chan[channel];

	local_irq_save(flags);

	if ((channel >= dmac.nb_chans) || !chan->allocated) {
		ret = -EINVAL;
		goto badparam;
	}

	if (chan->busy) {

		/* wait for transfer completion */
		timeout = PL080_TIMEOUT;
		do {
			tc = __raw_readl(dmac.base + _PL080_RAWINTTCSTATUS);
			tc &= (1 << channel);
			timeout--;
		} while (timeout && !tc);

		/* get error status */
		err = __raw_readl(dmac.base+_PL080_RAWINTERRORSTATUS);
		err &= (1 << channel);

		if (!timeout || err) {
			/* something bad happened */
			printk(KERN_ERR "dma%d: tc=%u error=%u timeout=%u\n",
			       channel, tc, err, timeout);
			ret = -EIO;
		}

		/* make sure channel is disabled */
		__raw_writel(0, chan->cxbase+_PL080_CXCONFIG);

		/* clear status */
		__raw_writel(1 << channel, dmac.base+_PL080_INTTCCLEAR);
		__raw_writel(1 << channel, dmac.base+_PL080_INTERRCLR);

		chan->busy = 0;
	}

badparam:
	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(pl08x_dma_wait);

static int __init pl08x_dma_probe(struct platform_device *pdev)
{
	u32 partnum;
	int i, ret = 0;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENOENT;
		goto noregs;
	}

	/* get flowcontrol configuration table (this is chip specific) */
	dmac.flow = pdev->dev.platform_data;
	if (dmac.flow == NULL) {
		ret = -ENOENT;
		goto noregs;
	}

	dmac.base = ioremap(res->start, res->end-res->start+1);
	if (!dmac.base) {
		ret = -ENOMEM;
		goto nomap;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		ret = -ENOENT;
		goto noirq;
	}
	dmac.irq = (unsigned int)res->start;

	ret = request_irq(dmac.irq, pl08x_dma_irq, 0, "dma-pl08x", NULL);
	if (ret) {
		goto noirq;
	}

	/* enable clock */
	dmac.clk = clk_get(NULL, "dmac");
	if (dmac.clk == NULL) {
		ret = -ENOENT;
		goto noclock;
	}
	clk_enable(dmac.clk);

	/* enable DMAC and clear interrupts */
        __raw_writel(_PL080_CONFIG_ENABLE, dmac.base+_PL080_CONFIG);
	__raw_writel(0xff, dmac.base+_PL080_INTTCCLEAR);
	__raw_writel(0xff, dmac.base+_PL080_INTERRCLR);

	/* autodetect controller version */
	partnum = __raw_readl(dmac.base+_PL080_PERIPHID0);
	partnum |= (__raw_readl(dmac.base+_PL080_PERIPHID1) << 8);
	partnum &= (1 << bw_PL080_PERIPHID_PARTNUMBER)-1;

	if (partnum == _PL080_PERIPHID_PARTNUMBER_PL080) {
		/* PL080 */
		dmac.nb_chans = 8;
	}
	else {
		/* assume PL081 with 2 channels */
		dmac.nb_chans = 2;
	}

	/* initialize all channels */
	for (i = 0; i < dmac.nb_chans; i++) {
		dmac.chan[i].allocated = 0;
		dmac.chan[i].busy = 0;
		dmac.chan[i].cxbase = dmac.base + _PL080_C0OFF+i*_PL080_CXRANGE;
		/* make sure channel is disabled */
		__raw_writel(0, dmac.chan[i].cxbase+_PL080_CXCONFIG);
	}
	platform_set_drvdata(pdev, &dmac);

	printk(KERN_INFO "ARM PL08x DMA controller driver $Revision: 1.3 $\n");

	return 0;

noclock:
	free_irq(dmac.irq, NULL);
noirq:
	iounmap(dmac.base);
nomap:
noregs:
	return ret;
}

static int pl08x_dma_remove(struct platform_device *pdev)
{
	unsigned int i;
	struct pl08x_dev *dev = platform_get_drvdata(pdev);

	if (dev) {
		/* make sure all channels are stopped and disabled */
		for (i = 0; i < dmac.nb_chans; i++) {
			pl08x_dma_abort(i);
			pl08x_dma_free(i);
		}

		iounmap(dmac.base);
		clk_disable(dmac.clk);
		clk_put(dmac.clk);
		free_irq(dmac.irq, NULL);
		dmac.nb_chans = 0;

		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

static struct platform_driver pl08x_dma_driver = {
	.probe		= pl08x_dma_probe,
	.remove 	= pl08x_dma_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {.name = "dma-pl08x"},
};

int __init pl08x_dma_init(void)
{
        return platform_driver_register(&pl08x_dma_driver);
}

subsys_initcall(pl08x_dma_init);
