/*
 * p6_spi.c
 *
 * Parrot6 SPI driver
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     florent.bayendrian@parrot.com
 * @date       2008-04-10
 *
 * Initial version inspired by:
 * 	linux/drivers/spi/spi_bfin5xx.c
 * 	linux/drivers/spi/spi_imx.c
 * 	linux/drivers/spi/pxa2xx_spi.c
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
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/sched.h>

#include <asm/io.h>
#include <asm/dma.h>

#include <mach/spi.h>
#include <mach/parrot.h>
#include <mach/dma-pl08x.h>
#include <mach/gpio.h>

/*
 * P6 SPI interface registers
 */
#define P6_SPI_REG_CTRL				0x00
#define     P6_SPI_CTRL_TSETUPCS	    0
#define     P6_SPI_CTRL_THOLDCS		    4
#define     P6_SPI_CTRL_STREAM		(1<<8)
#define     P6_SPI_CTRL_MSTR		(1<<9)
#define     P6_SPI_CTRL_CPOL		(1<<10)
#define     P6_SPI_CTRL_CPHAL		(1<<11)
#define     P6_SPI_CTRL_LSB		(1<<12)
#define     P6_SPI_CTRL_ITEN		(1<<13)
#define     P6_SPI_CTRL_DMAEN           (1<<14)
#define     P6_SPI_CTRL_DMATXMODE       (1<<15)
#define     P6_SPI_CTRL_DMASIZEEN	(1<<16)
#define     P6_SPI_CTRL_DMASSMODE1	(1<<17)
#define     P6_SPI_CTRL_DMASSMODE2	(1<<18)
#define P6_SPI_REG_SPEED			0x04
#define     P6_SPI_SPEED_MAXDIV		 1023
#define P6_SPI_REG_STATUS			0x08
#define P6_SPI_REG_SIZE				0x0C
#define     P6_SPI_STATUS_TXFULL	(1<<0)
#define     P6_SPI_STATUS_TXEMPTY	(1<<1)
#define     P6_SPI_STATUS_RXFULL	(1<<2)
#define     P6_SPI_STATUS_RXEMPTY	(1<<3)
#define     P6_SPI_STATUS_ITEN		(1<<4)
#define P6_SPI_REG_THRESHOLD_RX			0x10
#define P6_SPI_REG_THRESHOLD_TX			0x14
#define P6_SPI_REG_DATA				0x40
#define    P6_SPI_DATA_TXLASTBYTE	(1<<8)
#define    P6_SPI_DATA_RXFLAG		(1<<8)
#define    P6_SPI_DATA_RXEMPTY		(1<<9)

#define P6_FIFO_SIZE		64
#define P6_HALFFIFO_SIZE	(P6_FIFO_SIZE/2)
#define P6_SPI_THRESHOLD	34
#define MIN(a,b) ((a)<(b) ? (a):(b))

#define P6_SPI_DEFAULT_DMA_BUFSIZE	(2048*4)

#define DUMMY_BYTE	0x0

/* Queue state */
#define QUEUE_RUNNING			(0)
#define QUEUE_STOPPED			(1)

#define P6_SPI_WAIT_TIMEOUT		(200)	/* event timeout in jiffies */


#define DRV_NAME	"p6_spi"
#define PFX		DRV_NAME ": "


/**
 * Debug verbosity level
 * 0: nothing
 * 1: default
 * 2: very verbose
 * 3: a little more than very verbose
 */
static unsigned int debug_level = 1;
module_param(debug_level, uint, 0644);
static int use_dma = 1;
module_param(use_dma, bool, 0644);

#define P6_SPI_DBG(_lvl, _format, ...)					\
	do {								\
		if (debug_level >= _lvl)				\
			printk(KERN_DEBUG PFX _format, ## __VA_ARGS__);	\
	} while (0);

struct driver_data {
	struct platform_device	*pdev;
	void __iomem		*iobase;
	uint32_t		base_clk_hz;
	uint32_t		spi_clk_hz;
	uint32_t		bytes_per_msec;
	struct clk		*clock;

	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;

	/* Driver message queue */
	struct workqueue_struct *workqueue;
	struct work_struct	work;
	spinlock_t		lock;
	struct list_head	queue;
	int busy;
	int run;

	/* Current message, transfer and state */
	struct spi_message	*cur_msg;
	struct spi_transfer	*cur_transfer;

	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;

	/* DMA related parameters */
	int use_dma;
	wait_queue_head_t	dma_wakeup_queue;
	int			dma_status;
	int			dma_wakeup_flag;
	unsigned int		dma_chan;
	uint32_t		*dmabuf;
	dma_addr_t		dmabuf_phys;
	dma_addr_t		dmabuf_phys_rx;
	unsigned int		dmabuf_len;
	unsigned int		dma_req;
	/* block oriented xfer */
	u8			*dma_data;
	u8			*dma_block;
	unsigned int		dma_block_len;
	unsigned		dma_tx;
};

static void pump_messages(struct work_struct *work);

static int init_queue(struct driver_data *drv_data)
{
	INIT_LIST_HEAD(&drv_data->queue);
	spin_lock_init(&drv_data->lock);

	drv_data->run = QUEUE_STOPPED;
	drv_data->busy = 0;

	/* init messages workqueue */
	INIT_WORK(&drv_data->work, pump_messages);
	drv_data->workqueue =
		create_singlethread_workqueue(dev_name(drv_data->master->dev.parent));
	if (drv_data->workqueue == NULL)
		return -EBUSY;
	return 0;
}

static int start_queue(struct driver_data *drv_data)
{
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);

	if (drv_data->run == QUEUE_RUNNING || drv_data->busy) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return -EBUSY;
	}

	drv_data->run = QUEUE_RUNNING;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	spin_unlock_irqrestore(&drv_data->lock, flags);

	queue_work(drv_data->workqueue, &drv_data->work);

	return 0;
}

static int stop_queue(struct driver_data *drv_data)
{
	unsigned long flags;
	unsigned limit = 500;
	int status = 0;

	spin_lock_irqsave(&drv_data->lock, flags);

	/* This is a bit lame, but is optimized for the common execution path.
	 * A wait_queue on the drv_data->busy could be used, but then the common
	 * execution path (pump_messages) would be required to call wake_up or
	 * friends on every SPI message. Do this instead */
	drv_data->run = QUEUE_STOPPED;
	while (!list_empty(&drv_data->queue) && drv_data->busy && limit--) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		msleep(10);
		spin_lock_irqsave(&drv_data->lock, flags);
	}

	if (!list_empty(&drv_data->queue) || drv_data->busy)
		status = -EBUSY;

	spin_unlock_irqrestore(&drv_data->lock, flags);

	return status;
}

static int destroy_queue(struct driver_data *drv_data)
{
	int status;

	status = stop_queue(drv_data);
	if (status != 0)
		return status;

	if (drv_data->workqueue)
		destroy_workqueue(drv_data->workqueue);

	return 0;
}

static void giveback(struct driver_data *drv_data)
{
	struct spi_message *msg;

	msg = drv_data->cur_msg;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	msg->state = NULL;

	if (msg->complete)
		msg->complete(msg->context);
}

static void p6_spi_xfer_pio(struct driver_data *drv_data, u8 *rx, u8 *tx, uint32_t size, int drop_ss)
{
	int byte_done = 0;
	uint32_t n, i;

	while (byte_done < size) {
		// start with TX
		n = MIN(size - byte_done, P6_FIFO_SIZE);
		if (tx) {
			for (i = 0; i < n; i++) {
				uint32_t last_byte = 0;
				if (byte_done + i == size - 1 && drop_ss) {
					last_byte = P6_SPI_DATA_TXLASTBYTE;
				}
				__raw_writel(tx[byte_done+i]|last_byte,
					drv_data->iobase + P6_SPI_REG_DATA);
			}
		} else {
			// dummy write
			for (i = 0; i < n; i++) {
				uint32_t last_byte = 0;
				if (byte_done + i == size - 1 && drop_ss) {
					last_byte = P6_SPI_DATA_TXLASTBYTE;
				}
				__raw_writel(DUMMY_BYTE|last_byte,
					drv_data->iobase + P6_SPI_REG_DATA);
			}
		}
		// finish with RX
		i = 0;
		while (i < n) {
			uint32_t tmp;
			do {
				tmp = __raw_readl(drv_data->iobase +
					P6_SPI_REG_DATA);
			} while (tmp & P6_SPI_DATA_RXEMPTY);

			if (rx) {
				rx[byte_done+i] = tmp & 0xFF;
			}
			i++;
		}
		byte_done += n;
	}
}

static int handle_spi_xfer_pio(struct driver_data *drv_data)
{
	struct spi_transfer *transfer = drv_data->cur_transfer;
	struct spi_message *msg = drv_data->cur_msg;
	u8 *tx_buf = (u8*) drv_data->cur_transfer->tx_buf;
	u8 *rx_buf = (u8*) drv_data->cur_transfer->rx_buf;
	int drop_ss = 0;

	if (transfer->transfer_list.next == &msg->transfers) {
		drop_ss = 1; // last transfer
	}

	p6_spi_xfer_pio(drv_data, rx_buf, tx_buf, transfer->len, drop_ss);

	return 0;
}

static void fill_dma_buffer(struct driver_data *drv_data)
{
	struct spi_transfer *transfer = drv_data->cur_transfer;
	struct spi_message *msg = drv_data->cur_msg;
	uint32_t i;

	for (i = 0; i < drv_data->dma_block_len; i++) {
		drv_data->dmabuf[i] = drv_data->dma_block[i];
	}

	if (transfer->transfer_list.next == &msg->transfers &&
	    drv_data->dma_block + drv_data->dma_block_len ==
	    drv_data->dma_data + transfer->len) {

		// last transfer and last byte, release CS
		drv_data->dmabuf[drv_data->dma_block_len-1] |=
			P6_SPI_DATA_TXLASTBYTE;
	}
}

void save_dma_buffer(struct driver_data *drv_data)
{
	dma_unmap_single(&drv_data->pdev->dev, drv_data->dmabuf_phys_rx, drv_data->dma_block_len, DMA_FROM_DEVICE);

	if (parrot_chip_is_p6()) {
		struct spi_message *msg = drv_data->cur_msg;
		struct spi_transfer *transfer = drv_data->cur_transfer;
		uint32_t byte_done = drv_data->dma_block - drv_data->dma_data;
		u8 *rx = transfer->rx_buf;
		uint32_t i;
		int drop_ss = 0;
		uint32_t status;

		i = drv_data->dma_block_len - 65;

		// read last bytes
		status =  __raw_readl(drv_data->iobase + P6_SPI_REG_STATUS);
		while (!(status & P6_SPI_STATUS_RXEMPTY)) {
			u8 tmp = __raw_readl(drv_data->iobase + P6_SPI_REG_DATA);
			rx[byte_done + i++] = tmp;
			status =  __raw_readl(drv_data->iobase + P6_SPI_REG_STATUS);
		}

		// finish with a last FIFO xfer
		if (transfer->transfer_list.next == &msg->transfers &&
		    drv_data->dma_block + drv_data->dma_block_len ==
		    drv_data->dma_data + transfer->len) {
		    	drop_ss = 1;
		}

		if (i != drv_data->dma_block_len) {
			p6_spi_xfer_pio(drv_data, &rx[byte_done + i], NULL, drv_data->dma_block_len - i, drop_ss);
		}
	}
}

static int is_dma_efficient(struct driver_data *drv_data, unsigned int len)
{
	struct spi_transfer *transfer = drv_data->cur_transfer;

	// DMA can only be used if one buffer is NULL
	if (transfer->rx_buf && transfer->tx_buf)
		return 0;

	// use polling mode for short transfers (<= 8 µs)
	if (parrot_chip_is_p6() && transfer->rx_buf && (len < 65 || len - 65 < (8*drv_data->bytes_per_msec)/1000))
		return 0;

	if (parrot_chip_is_p6i() && transfer->rx_buf && (len < (8*drv_data->bytes_per_msec)/1000))
		return 0;

	if (transfer->tx_buf && len < (8*drv_data->bytes_per_msec)/1000)
		return 0;

	return 1;
}

static int map_dma_buffer(struct driver_data *drv_data)
{
	struct spi_transfer *transfer = drv_data->cur_transfer;

	if (!is_dma_efficient(drv_data, transfer->len))
		return 0;

	if (transfer->tx_buf) {
		drv_data->dma_data = drv_data->dma_block =
			(u8 *) transfer->tx_buf;
		drv_data->dma_tx = 1;
	} else {
		drv_data->dma_data = drv_data->dma_block =
			(u8 *) transfer->rx_buf;
		drv_data->dma_tx = 0;
	}

	if (drv_data->dmabuf_len >> 2 < transfer->len) {
		drv_data->dma_block_len = drv_data->dmabuf_len >> 2;
	} else {
		drv_data->dma_block_len = transfer->len;
	}

	if (transfer->tx_buf)
		fill_dma_buffer(drv_data);

	return 1;
}

static int map_next_dma_buffer(struct driver_data *drv_data)
{
	struct spi_transfer *transfer = drv_data->cur_transfer;
	struct spi_message *msg = drv_data->cur_msg;
	unsigned int byte_done = drv_data->dma_block - drv_data->dma_data +
					drv_data->dma_block_len;

	if (byte_done == transfer->len)
		return 0;

	if (transfer->transfer_list.next == &msg->transfers &&
		!drv_data->dma_tx && byte_done ==  transfer->len - 1) {
	    	return 0;
	}

	drv_data->dma_block += drv_data->dma_block_len;
	drv_data->dma_block_len =
		MIN(transfer->len - byte_done, drv_data->dmabuf_len >> 2);

	if (transfer->tx_buf &&
	    is_dma_efficient(drv_data, drv_data->dma_block_len))
		fill_dma_buffer(drv_data);

	return 1;
}

int handle_spi_xfer_dma(struct driver_data *drv_data)
{
	struct spi_transfer *transfer = drv_data->cur_transfer;
	struct spi_message *msg = drv_data->cur_msg;
	uint32_t ctrl, ctrl_save;
	struct pl08x_dma_cfg dma_cfg;
	int ret = 0;
	u32 status;

	if (!is_dma_efficient(drv_data, drv_data->dma_block_len)) {
		// map_next_dma_buffer() could return a short buffer at the end
		int drop_ss = 0;
		u8 *rx = NULL;
		u8 *tx = NULL;

		if (transfer->transfer_list.next == &msg->transfers &&
		    drv_data->dma_block + drv_data->dma_block_len ==
		    drv_data->dma_data + transfer->len) {
	    		drop_ss = 1;
		}

		if (drv_data->dma_tx) {
			tx = drv_data->dma_block;
		} else {
			rx = drv_data->dma_block;
		}
		p6_spi_xfer_pio(drv_data, rx, tx, drv_data->dma_block_len, drop_ss);
		return ret;
	}

	// configure and start the DMA
	memset(&dma_cfg, 0, sizeof(dma_cfg));
	if (drv_data->dma_tx) {
		dma_cfg.src_addr = drv_data->dmabuf_phys;
		dma_cfg.dst_addr = drv_data->ioarea->start + P6_SPI_REG_DATA;
		dma_cfg.src_periph = _PL080_PERIPH_MEM;
		dma_cfg.dst_periph = drv_data->dma_req;
		dma_cfg.cxctrl.si = 1;
		dma_cfg.cxctrl.di = 0;
		dma_cfg.cxctrl.transize = drv_data->dma_block_len;
		dma_cfg.cxctrl.swidth = 2;
		dma_cfg.cxctrl.dwidth = 2;
	} else {
		drv_data->dmabuf_phys_rx =
			dma_map_single(&drv_data->pdev->dev,
				transfer->rx_buf + (drv_data->dma_block - drv_data->dma_data),
				drv_data->dma_block_len, DMA_FROM_DEVICE);
		dma_cfg.src_addr = drv_data->ioarea->start + P6_SPI_REG_DATA;
		dma_cfg.dst_addr = drv_data->dmabuf_phys_rx;
		dma_cfg.src_periph = drv_data->dma_req;
		dma_cfg.dst_periph = _PL080_PERIPH_MEM;
		dma_cfg.cxctrl.si = 0;
		dma_cfg.cxctrl.di = 1;
		if (parrot_chip_is_p6()) {
			dma_cfg.cxctrl.transize = drv_data->dma_block_len - 65;
		} else {
			dma_cfg.cxctrl.transize = drv_data->dma_block_len;
		}
		dma_cfg.cxctrl.swidth = 0;
		dma_cfg.cxctrl.dwidth = 0;
	}
	// burst size = 1
	dma_cfg.cxctrl.sbsize = 0;
	dma_cfg.cxctrl.dbsize = 0;
	dma_cfg.cxctrl.sahb = 0;
	dma_cfg.cxctrl.dahb = 0;
	pl08x_dma_start(drv_data->dma_chan, &dma_cfg);

	// enable SPI DMA mode
	ctrl_save = ctrl = __raw_readl(drv_data->iobase + P6_SPI_REG_CTRL);
	ctrl |= P6_SPI_CTRL_DMAEN;
	if (drv_data->dma_tx) {
		ctrl |= P6_SPI_CTRL_DMATXMODE;
	} else {
		ctrl |= P6_SPI_CTRL_DMASIZEEN;
		__raw_writel(dma_cfg.cxctrl.transize,
				drv_data->iobase + P6_SPI_REG_SIZE);
	}

	if (parrot_chip_is_p6i() && !drv_data->dma_tx &&
	     transfer->transfer_list.next == &msg->transfers &&
	     drv_data->dma_block + drv_data->dma_block_len ==
	     drv_data->dma_data + transfer->len) {
		// release CS after the rx transfer
		ctrl |= P6_SPI_CTRL_DMASSMODE1;
	}

	drv_data->dma_wakeup_flag = 0;
	__raw_writel(ctrl, drv_data->iobase + P6_SPI_REG_CTRL);
	// wait for the dma interrupt
	ret = wait_event_interruptible_timeout(drv_data->dma_wakeup_queue,
					       drv_data->dma_wakeup_flag,
					       P6_SPI_WAIT_TIMEOUT);
	__raw_writel(ctrl_save, drv_data->iobase + P6_SPI_REG_CTRL);
	if (!ret) {
		dev_err(&drv_data->pdev->dev, "DMA timeout\n");
		pl08x_dma_abort(drv_data->dma_chan);
		ret = -ETIMEDOUT;
	} else if (drv_data->dma_status) {
		dev_err(&drv_data->pdev->dev, "DMA error, status = 0x%x\n", drv_data->dma_status);
		ret = -EINVAL;
	}

	// empty rx fifo
	if (drv_data->dma_tx) {
		// drop bytes
		status =  __raw_readl(drv_data->iobase + P6_SPI_REG_STATUS);
		while (!(status & P6_SPI_STATUS_RXEMPTY)) {
			__raw_readl(drv_data->iobase + P6_SPI_REG_DATA);
			status =  __raw_readl(drv_data->iobase + P6_SPI_REG_STATUS);
		}
	} else {
		save_dma_buffer(drv_data);
	}

	return ret;
}

static void p6_spi_print_data(struct driver_data *drv_data)
{
	struct spi_transfer *transfer = drv_data->cur_transfer;
	int i;

	if (debug_level <= 1)
		return;

	if (transfer->tx_buf && transfer->rx_buf) {
		u8 *tx = (u8 *) transfer->tx_buf;
		u8 *rx = (u8 *) transfer->rx_buf;

		printk(KERN_DEBUG PFX "tx,rx: ");
		for (i = 0; i < transfer->len; i++)
			printk("0x%2.2x,0x%2.2x ", tx[i], rx[i]);

	} else {
		u8 *data = NULL;

		if (transfer->tx_buf) {
			data = (u8 *) transfer->tx_buf;
			printk(KERN_DEBUG PFX "tx: ");
		} else if (transfer->rx_buf) {
			data = (u8 *) transfer->rx_buf;
			printk(KERN_DEBUG PFX  "rx: ");
		}

		for (i = 0; i < transfer->len; i++)
			printk("0x%2.2x ", data[i]);
	}
	printk("\n");
}

static int handle_spi_xfer(struct driver_data *drv_data)
{
	struct spi_transfer *transfer = drv_data->cur_transfer;
	struct spi_message *msg = drv_data->cur_msg;
	int ret = 0;

	if ((!transfer->tx_buf && !transfer->rx_buf) || (transfer->len == 0))
		return 0;

	if (drv_data->use_dma && map_dma_buffer(drv_data)) {
		do {
			ret = handle_spi_xfer_dma(drv_data);
		} while (map_next_dma_buffer(drv_data));
	} else {
		ret = handle_spi_xfer_pio(drv_data);
	}

	msg->actual_length += transfer->len;

	return ret;
}

static void pump_messages(struct work_struct *work)
{
	struct driver_data *drv_data = container_of(work, struct driver_data, work);
	unsigned long flags;

	/* Lock queue and check for queue work */
	spin_lock_irqsave(&drv_data->lock, flags);

	if (list_empty(&drv_data->queue) || drv_data->run == QUEUE_STOPPED) {
		drv_data->busy = 0;
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return;
	}
	if (drv_data->cur_msg) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return;
	}
	/* Extract head of queue */
	drv_data->cur_msg = list_entry(drv_data->queue.next,
				struct spi_message, queue);
	list_del_init(&drv_data->cur_msg->queue);
	drv_data->busy = 1;
	spin_unlock_irqrestore(&drv_data->lock, flags);

	/* Initial message status */
	drv_data->cur_msg->status = 0;

	drv_data->cur_transfer =
		list_entry(drv_data->cur_msg->transfers.next,
				struct spi_transfer,
				transfer_list);

	do {
		struct spi_transfer *transfer = drv_data->cur_transfer;
		struct spi_message *msg = drv_data->cur_msg;
		u32 speed = 0;
	
		// set the correct freq if needed
		if (transfer->speed_hz) {
			u32 div;

			speed = __raw_readl(drv_data->iobase + P6_SPI_REG_SPEED);
			for (div = 0; div <= P6_SPI_SPEED_MAXDIV; div++) {
				uint32_t spi_clk_hz = drv_data->base_clk_hz/((div+1)*2);
				if (spi_clk_hz <= transfer->speed_hz) {
					break;
				}
			}

			if (div > P6_SPI_SPEED_MAXDIV) {
				dev_err(&msg->spi->dev, "unreachable frequency %d\n",
					transfer->speed_hz);
				drv_data->cur_msg->status = -EINVAL;
				break;

			}

			__raw_writel(div, drv_data->iobase + P6_SPI_REG_SPEED);
		}

		handle_spi_xfer(drv_data);

		// restore previous setting
		if (transfer->speed_hz)
			__raw_writel(speed, drv_data->iobase + P6_SPI_REG_SPEED);

		p6_spi_print_data(drv_data);

		if (drv_data->cur_msg->status) {
			// IO error, all transfers are aborted
			P6_SPI_DBG(2, "IO error\n");
			break;
		}

		if (transfer->transfer_list.next == &msg->transfers) {
		    	// it was the last transfer
			P6_SPI_DBG(2, "----------\n");
			break;
		}

		// delay if requested at end of transfer and before the next
		if (transfer->delay_usecs)
			udelay(transfer->delay_usecs);

		drv_data->cur_transfer =
			list_entry(transfer->transfer_list.next,
				struct spi_transfer, transfer_list);
	} while (1);

	giveback(drv_data);
	drv_data->busy = 0;
}

void p6_spi_dma_callback(unsigned int chan, void *data, int status)
{
	struct driver_data *drv_data = data;

	drv_data->dma_wakeup_flag = 1;
	drv_data->dma_status = status;
	wake_up_interruptible(&drv_data->dma_wakeup_queue);
}

static int p6_spi_setup(struct spi_device *spi)
{
	uint32_t ctrl = 0;
	uint32_t div;
	struct driver_data *drv_data = spi_master_get_devdata(spi->master);
	struct device *dev = &spi->dev;
	int ret = 0;

	P6_SPI_DBG(3, "setup()\n");

	/* Zero (the default) here means 8 bits */
	if (!spi->bits_per_word)
		spi->bits_per_word = 8;
	if (spi->bits_per_word != 8 || spi->mode & SPI_CS_HIGH) {
		dev_err(dev, "configuration not supported\n");
		ret = -EINVAL;
		goto end;
	}

	if (spi->mode & SPI_CPHA) {
		P6_SPI_DBG(2, "SPI_CPHA\n");
		ctrl |= P6_SPI_CTRL_CPHAL;
	}
	if (spi->mode & SPI_CPOL) {
		P6_SPI_DBG(2, "SPI_CPOL\n");
		ctrl |= P6_SPI_CTRL_CPOL;
	}
	if (spi->mode & SPI_LSB_FIRST)
		ctrl |= P6_SPI_CTRL_LSB;
	ctrl |= P6_SPI_CTRL_MSTR;

	// freq(SPI) = freq(PCLK)/((SPISPEED+1)*2)
	for (div = 0; div <= P6_SPI_SPEED_MAXDIV; div++) {
		uint32_t spi_clk_hz = drv_data->base_clk_hz/((div+1)*2);
		if (spi_clk_hz <= spi->max_speed_hz) {
			drv_data->spi_clk_hz = spi_clk_hz;
			break;
		}
	}

	if (div > P6_SPI_SPEED_MAXDIV) {
		dev_err(dev, "unreachable frequency %d\n", drv_data->spi_clk_hz);
		ret = -EINVAL;
		goto end;
	}

	P6_SPI_DBG(1, "clock = %dHz\n", drv_data->spi_clk_hz);

	if (spi->controller_data) {
		struct p6_spi_config *config = spi->controller_data;
		uint32_t period_ns = 1000000000 / drv_data->spi_clk_hz;
		int tsetupcs, tholdcs;

		tsetupcs = config->tsetupcs_ns / period_ns - 2;
		tholdcs = config->tholdcs_ns / period_ns - 2;

		if (config->tsetupcs_ns % period_ns == 0) {
			tsetupcs++;
		}
		if (config->tsetupcs_ns % period_ns == 0) {
			tholdcs++;
		}

		if (tsetupcs < 0) {
			tsetupcs = 0;
		}

		if (tholdcs < 0) {
			tholdcs = 0;
		}

		if (tsetupcs > 0xF || tholdcs > 0xF) {
			dev_err(dev, "unreachable timing values\n");
			ret = -EINVAL;
			goto end;
		}

		ctrl |= ((tsetupcs & 0xF) << P6_SPI_CTRL_TSETUPCS) |
			((tholdcs & 0xF) << P6_SPI_CTRL_THOLDCS);

		P6_SPI_DBG(1, "tsetupcs = %uns\n", (tsetupcs+2)*period_ns);
		P6_SPI_DBG(1, "tholdcs = %uns\n", (tholdcs+2)*period_ns);
		drv_data->bytes_per_msec = drv_data->spi_clk_hz/((8+2+tholdcs)*1000);
		drv_data->use_dma = !config->disable_dma;
	} else {
		P6_SPI_DBG(1, "warning, no controller data supply\n");
		drv_data->bytes_per_msec = drv_data->spi_clk_hz/((8+2)*1000);
		drv_data->use_dma = use_dma;
	}

	__raw_writel(ctrl, drv_data->iobase + P6_SPI_REG_CTRL);
	__raw_writel(div, drv_data->iobase + P6_SPI_REG_SPEED);
	__raw_writel(0x1, drv_data->iobase + P6_SPI_REG_THRESHOLD_RX);
	__raw_writel(0x1, drv_data->iobase + P6_SPI_REG_THRESHOLD_TX);

	if (drv_data->use_dma) {
		if (pl08x_dma_request(&drv_data->dma_chan, DRV_NAME, p6_spi_dma_callback, drv_data)) {
			dev_err(dev, "dma channel unavailable\n");
			ret = -EBUSY;
			goto no_dmachan;
		}
		dev_info(dev, "using dma\n");
	}
	else
		dev_info(dev, "not using dma\n");

	P6_SPI_DBG(3, "setup() done ok\n");
	return 0;

no_dmachan:
end:
	P6_SPI_DBG(1, "setup() error\n");
	return ret;
}

static int p6_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct driver_data *drv_data = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);
	if (drv_data->run == QUEUE_STOPPED) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return -ESHUTDOWN;
	}
	msg->actual_length = 0;
	msg->status = -EINPROGRESS;

	list_add_tail(&msg->queue, &drv_data->queue);
	if (drv_data->run == QUEUE_RUNNING) {
		queue_work(drv_data->workqueue, &drv_data->work);
	}
	spin_unlock_irqrestore(&drv_data->lock, flags);

	return 0;
}

static struct clk *__devinit parrot6_spi_enable(struct platform_device *pdev)
{
	static const char *clk_name[] = {"spi0", "spi1", "spi2"};
	struct clk *spi_clk;

	spi_clk = clk_get(&pdev->dev, clk_name[pdev->id > 3 ? 0 : pdev->id]);
	if (!IS_ERR(spi_clk)) {
		clk_enable(spi_clk);
	}
	return spi_clk;
}

static int __devinit parrot6_spi_probe(struct platform_device *pdev)
{
	struct driver_data *drv_data;
	struct spi_master *master;
	struct resource *res;
	int ret = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct driver_data));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		ret = -ENOMEM;
		goto no_master;
	}

	drv_data = spi_master_get_devdata(master);
	memset(drv_data, 0, sizeof(struct driver_data));
	drv_data->master = spi_master_get(master);
	drv_data->pdev = pdev;
	init_waitqueue_head(&drv_data->dma_wakeup_queue);

	drv_data->clock = parrot6_spi_enable(pdev);
	if (IS_ERR(drv_data->clock) || (drv_data->clock == NULL)) {
		dev_err(&pdev->dev, "unable to activate spi clock\n");
		goto clk_err;
	}
	drv_data->base_clk_hz = clk_get_rate(NULL);
#if defined(CONFIG_VERSATILE_PARROT6)
	drv_data->base_clk_hz = 30000000; // check your FPGA configuration for that
#endif
	master->bus_num = pdev->id;
	master->cleanup = NULL;
	master->setup = p6_spi_setup;
	master->transfer = p6_spi_transfer;
	master->num_chipselect = 1;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;

	ret = init_queue(drv_data);
	if (ret != 0) {
		dev_err(&pdev->dev, "problem initializing queue\n");
		goto no_queue;
	}
	ret = start_queue(drv_data);
	if (ret != 0) {
		dev_err(&pdev->dev, "problem starting queue\n");
		goto no_queue;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto no_iores;
	}

	drv_data->ioarea = request_mem_region(res->start, res->end - res->start + 1,
					pdev->name);
	if (drv_data->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve IO region\n");
		ret = -ENXIO;
		goto no_iores;
	}

	drv_data->iobase = ioremap(res->start, res->end - res->start + 1);
	if (drv_data->iobase == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		ret = -ENXIO;
		goto no_iomap;
	}

	if (use_dma) {
		drv_data->dmabuf_len = P6_SPI_DEFAULT_DMA_BUFSIZE;
		drv_data->dmabuf = dma_alloc_coherent(&pdev->dev, drv_data->dmabuf_len,
				&drv_data->dmabuf_phys, GFP_KERNEL);
		if (!drv_data->dmabuf) {
			ret = -ENOMEM;
			goto no_bufdma;
		}

		switch (pdev->id) {
#if !defined(CONFIG_VERSATILE_PARROT6)
			case 1:
				drv_data->dma_req = _PL080_PERIPH_SPI1;
				break;
			case 2:
				drv_data->dma_req = _PL080_PERIPH_SPI2;
				break;
#endif
			default:
				drv_data->dma_req = _PL080_PERIPH_SPI0;
		}
	}

	/* Register with the SPI framework */
	platform_set_drvdata(pdev, drv_data);
	P6_SPI_DBG(3, "register SPI\n");
	ret = spi_register_master(master);
	if (ret != 0) {
		dev_err(&pdev->dev, "problem registering spi master\n");
		goto no_register;
	}

	dev_dbg(&pdev->dev, "controller probe successfully\n");
	return 0;

no_register:
	if (use_dma)
		dma_free_coherent(&pdev->dev, drv_data->dmabuf_len,
			drv_data->dmabuf, drv_data->dmabuf_phys);
no_bufdma:
	iounmap(drv_data->iobase);
no_iomap:
	release_resource(drv_data->ioarea);
	kfree(drv_data->ioarea);
	clk_put(drv_data->clock);
no_iores:
no_queue:
clk_err:
	spi_master_put(drv_data->master);
no_master:
	dev_err(&pdev->dev, "controller probe failed\n");
	return ret;
}

static int __devexit parrot6_spi_remove(struct platform_device *pdev)
{
	struct driver_data *drv_data = platform_get_drvdata(pdev);
	int status;

	if (!drv_data)
		return 0;

	/* remove the queue */
	status = destroy_queue(drv_data);
	if (status != 0) {
		dev_err(&pdev->dev, "queue remove failed (%d)\n", status);
		return status;
	}

	/* release dma resources */
	if (use_dma) {
		pl08x_dma_free(drv_data->dma_chan);
		dma_free_coherent(&pdev->dev, drv_data->dmabuf_len,
				drv_data->dmabuf, drv_data->dmabuf_phys);
	}

	/* release map resources */
	iounmap(drv_data->iobase);
	release_resource(drv_data->ioarea);
	kfree(drv_data->ioarea);

	/* disable spi clk */
	clk_disable(drv_data->clock);
	clk_put(drv_data->clock);

	/* disconnect from the SPI framework */
	spi_unregister_master(drv_data->master);
	spi_master_put(drv_data->master);

	/* prevent double remove */
	platform_set_drvdata(pdev, NULL);

	dev_dbg(&pdev->dev, "remove succeded\n");

	return 0;
}


#ifdef CONFIG_PM
static int parrot6_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct driver_data *drv_data = platform_get_drvdata(pdev);
	int status = 0;

	status = stop_queue(drv_data);
	if (status != 0) {
		dev_warn(&pdev->dev, "suspend cannot stop queue\n");
		return status;
	}

	dev_dbg(&pdev->dev, "suspended\n");

	return 0;
}

static int parrot6_spi_resume(struct platform_device *pdev)
{
	struct driver_data *drv_data = platform_get_drvdata(pdev);
	int status = 0;

	/* Start the queue running */
	status = start_queue(drv_data);
	if (status != 0)
		dev_err(&pdev->dev, "problem starting queue (%d)\n", status);
	else
		dev_dbg(&pdev->dev, "resumed\n");

	return status;
}

#else
#define parrot6_spi_suspend NULL
#define parrot6_spi_resume  NULL
#endif /* CONFIG_PM */

static struct platform_driver parrot6_spi_driver = {
	.probe		= parrot6_spi_probe,
	.remove		= __devexit_p(parrot6_spi_remove),
	.suspend	= parrot6_spi_suspend,
	.resume		= parrot6_spi_resume,
	.driver		= {
		.name	= "p6-spi",
		.owner	= THIS_MODULE,
	},
};

static int __init parrot6_spi_init(void)
{
	printk(KERN_INFO "Parrot6 SPI driver $Revision: 1.8 $\n");
	return platform_driver_register(&parrot6_spi_driver);
}

static void __exit parrot6_spi_exit(void)
{
	platform_driver_unregister(&parrot6_spi_driver);
}

module_init(parrot6_spi_init);
module_exit(parrot6_spi_exit);

MODULE_DESCRIPTION("Parrot6 SPI Driver");
MODULE_ALIAS("parrot6-spi");
MODULE_AUTHOR("Florent Bayendrian, <florent.bayendrian@parrot.com>");
MODULE_LICENSE("GPL");
