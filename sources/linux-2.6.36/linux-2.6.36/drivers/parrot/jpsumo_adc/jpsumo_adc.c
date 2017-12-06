/**
 ******************************************************************************
 * @file jpsumo_adc.c
 * @brief Jumping Sumo ADC (aai) driver
 *
 * Copyright(C) 2017 Parrot S.A.
 *
 * @author     Samir Ammenouche <samir.ammenouche@parrot.com>
 * @author     Hugo Grostabussiat <hugo.grostabussiat@parrot.com>
 * @date       2017-02-22
 ******************************************************************************
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include <mach/regs-aai-p6.h>
#include <mach/jpsumo_adc.h>

#include "timer.h"
#include "jpsumo_adc.h"

/**
 * jsadc_device - struct which contains the informations needed by the driver.
 * @lock: used by the spinlock
 * @waitq: the wait queue used to poll
 * @adr: the dmable memory area
 * @map: the aai mapped register
 * @adc_status: adc mgmt status bit field
 * @bus: the physical address of the dmable memory
 * @numirq: the aai irq number
 * @mem_start: physical start address of registers
 * @mem_end: physical end address of registers
 * @gpio_mux_vbat_jump: number of the GPIO controlling the ADC0 mux
 * @gpio_mux_wheels: number of the GPIO controlling the ADC1-2 swapper
 * @cur_src: ADC source used for the current STREAMING operation
 * @completion: completion signaling the end of MONITORING data acquisition
 * @monitor_pass: current pass in ADC MONITORING acquisition mode
 * @monitor_data: structure with values sampled from the ADC in MONITORING mode
 * @streaming: flag indicating whether ADC data STREAMING is active
 * @overrun: flag signaling an overrun condition while STREAMING
 * @monitoring: flag indicating that ADC MONITORING acquisition is active
 */
struct jsadc_device {
	struct device			*device;
	struct clk			*clock;
	wait_queue_head_t		waitq;
	uint32_t			*adr;
	uint8_t				*aai;
	dma_addr_t			bus;
	uint32_t			numirq;
	uint32_t			mem_start;
	uint32_t			mem_end;
	int				gpio_mux_vbat_jump;
	int				gpio_mux_wheels;

	/* lock protects: cur_src, streaming, overrun and monitoring.
	 * Also protects jsadc_fdata members, mem and metadata */
	spinlock_t			lock;
	/* monitor_mut ensures that only one thread can perform monitoring
	 * or change mux at once. Protects: monitor_pass and monitor_data */
	struct mutex			monitor_mut;

	enum jsadc_src			cur_src;
	struct completion		completion;
	unsigned int			monitor_pass;
	struct jsadc_data		monitor_data;
	unsigned int			streaming : 1;
	unsigned int			overrun : 1;
	unsigned int			monitoring : 1;
};

struct jsadc_fdata {
	enum jsadc_src src;
	unsigned int streaming : 1;
};

/* Allocate SIZEMAX bytes + one more buffer used in case of overrun */
#define DMA_ALLOC_SIZE	(JSADC_SIZE_ALLBUFS + JSADC_DMA_SIZE)

#define SCRAPBUF_OFFSET	(JSADC_SIZE_ALLBUFS)

#define bufidx2off(x) ((x) << JSADC_DMA_SHIFT)

/* "Undocumented" AAI_DMACTL bit to enable DMA for ultrasound */
#define AAI_DMA_CTRL_ULTRA	(1 << 11)
#define MAX_AAI_ADC_INDEX 8

#define aai_writel(_value_, _reg_)\
	writel(_value_, _reg_ + adc_dev.aai)

#define aai_readl(_reg_) readl(_reg_ + adc_dev.aai)

/*
 * Mux for ADC 0 can be set on 2 positions:
 * 0 to read VBAT
 * 1 to read IJUMP
 */
enum {
	ADC0_MUX_VBAT = 0,
	ADC0_MUX_IJUMP
};

/*
 * Mux for ADC 1&2 can be set on 2 positions:
 * 0 ADC1 -> left wheel, ADC2 -> right wheel
 * 1 ADC1 -> right wheel, ADC2 -> left wheel
 */
enum {
	ADC1_MUX_LEFT = 0,
	ADC1_MUX_RIGHT
};

/*
 * struct mem - internal structure used to track dma memory allocation for
 *              streaming mode
 *
 * @pin: is the input buffer index
 * @pout is the output buffer index
 * @free: is the number of free buffers, read/write to this value must be
 *	mutually excluded
 * @usr_ptr: is the next buffer index that will be given to userspace.
 *	usr_ptr must refer to a 'done' buffer, except when usr_ptr == pin
 * @usr_full: flag indicating whether the userspace is holding all the buffers.
 *	It is done so that it is possible to differentiate between whether
 *	the userpace is holding all the buffers or none when usr_ptr == pin
 */
struct mem {
	int pin;
	int pout;
	int usr_ptr;
	int usr_full;
	int free;
};

/*
 * struct buf_metadata - keep track of buffer metadata in streaming mode
 *
 * @missed: Number of missed ADC buffers due to an overrun condition since the
 *	last buffer.
 * @src: Source from which this buffer was acquired
 */
struct buf_metadata {
	unsigned int missed;
	enum jsadc_src src;
};

static struct jsadc_device adc_dev = {
	.adr		= NULL,
	.aai		= NULL,
	.cur_src	= JSADC_SRC_VBAT,
};

static struct mem mem = {
	.pin      = 0,
	.pout     = 0,
	.usr_ptr  = 0,
	.usr_full = 0,
	.free     = JSADC_NUM_BUFS,
};
static struct buf_metadata metadata[JSADC_NUM_BUFS];

/**
 * Mark one DMA buffer as done so that userspace can request it.
 * This should be called when the ultrasound interrupt fires to signal the end
 * of a DMA transfer.
 * 
 * Returns 0 on success, -ENOMEM otherwise.
 */
static int mark_dma_done(void)
{
	if (mem.free > 0) {
		mem.pin = (mem.pin + 1) % JSADC_NUM_BUFS;
		mem.free -= 1;
		return 0;
	} else {
		return -ENOMEM;
	};
}

/**
 * Mark one of the DMA buffer as free so that it can be used for DMA again.
 * This should be called when the userspace releases the buffer.
 * 
 * Returns 0 on success, -ENOMEM otherwise.
 */
static int mark_dma_free(void)
{
	if (mem.free < JSADC_NUM_BUFS) {
#ifdef DEBUG
		/* Fill with known pattern to detect incomplete frames sent to
		 * userspace. */
		memset((uint8_t*)adc_dev.adr + bufidx2off(mem.pout),
		       0x5a, JSADC_DMA_SIZE);
#endif
		mem.pout = (mem.pout + 1) % JSADC_NUM_BUFS;
		mem.free += 1;
		return 0;
	} else {
		return -ENOMEM;
	};
}

static int get_dma_free(void)
{
	return mem.free;
}

/**
 * Returns whether a DMA buffer is available for userspace
 *
 * lock must be help
 */
static int is_dma_available_for_user(void)
{
	/* A buffer is available for userspace if:
	 * - the pool is neither full nor empty of 'done' buffers
	 * - userspace doesn't already have all 'done' buffers
	 *   (usr_ptr != pin)
	 * or
	 * - all buffers are 'done' (free == 0) (also implies
	 *   pin == pout && usr_ptr == pout)
	 * - the userspace doesn't have all 'done' buffers (!usr_full)
	 *   (with the previous condition, this implies that userspace
	 *   has no buffer at all)
	 */
	return (mem.usr_ptr != mem.pin) ||
	       (!mem.usr_full && (get_dma_free() == 0));
}

static void reset_dma_buffers(void)
{
	mem.pin      = 0;
	mem.pout     = 0;
	mem.usr_ptr  = 0;
	mem.usr_full = 0;
	mem.free     = JSADC_NUM_BUFS;
	adc_dev.overrun = 0;
	memset(&metadata, 0, sizeof(metadata));
}

static void streaming_set_source(enum jsadc_src src)
{
	uint32_t src_val;
	uint32_t reg;

	adc_dev.cur_src = src;
	switch(src) {
	case JSADC_SRC_VBAT:
		gpio_set_value(adc_dev.gpio_mux_vbat_jump, ADC0_MUX_VBAT);
		src_val = 0;
		break;
	case JSADC_SRC_IJUMP:
		gpio_set_value(adc_dev.gpio_mux_vbat_jump, ADC0_MUX_IJUMP);
		src_val = 0;
		break;
	case JSADC_SRC_LWHEEL:
		gpio_set_value(adc_dev.gpio_mux_wheels, ADC1_MUX_LEFT);
		src_val = 1;
		break;
	case JSADC_SRC_RWHEEL:
		gpio_set_value(adc_dev.gpio_mux_wheels, ADC1_MUX_LEFT);
		src_val = 2;
		break;
	default:
		return;
	}

	reg = aai_readl(AAI_ADC_MODES) & ~AAI_ADC_MODES_ULTRA_IN;
	aai_writel(reg | (src_val << 8), AAI_ADC_MODES);
	mmiowb();
}

/**
 * Start ADC data streaming.
 * Must be called with adc_dev.lock held.
 */
static void streaming_start(struct file *filp)
{
	struct jsadc_fdata *fdata = filp->private_data;
	WARN_ON(fdata->streaming);
	WARN_ON(adc_dev.streaming);
	reset_dma_buffers();
	fdata->streaming = 1;
	adc_dev.streaming = 1;

#ifdef DEBUG
	/* Fill with known pattern to detect incomplete frames sent to
	 * userspace. */
	memset((uint8_t*)adc_dev.adr, 0xff, JSADC_SIZE_ALLBUFS);
#endif

	/* Set ADC source */
	streaming_set_source(fdata->src);

	/* In case ultrasound is still active, set DMA start and follow
	 * registers to someplace safe (the scrap buffer). */
	aai_writel(adc_dev.bus + SCRAPBUF_OFFSET, AAI_DMASA_ULTRA);
	aai_writel(adc_dev.bus + SCRAPBUF_OFFSET, AAI_DMAFA_ULTRA);
	/* Next chunk of memory 8Kb*/
	aai_writel(0x7, AAI_ULTRA_DMA_COUNT);
	aai_writel(adc_dev.bus + bufidx2off(mem.pin), AAI_DMASA_ULTRA);
	aai_writel(adc_dev.bus + (bufidx2off(mem.pin + 1) % JSADC_SIZE_ALLBUFS),
	           AAI_DMAFA_ULTRA);
	aai_writel(1, AAI_ADC_ULTRA);
	mmiowb();
}

/**
 * Stop ADC data streaming.
 * Must be called with adc_dev.lock held.
 */
static void streaming_stop(struct file *filp)
{
	struct jsadc_fdata *fdata = filp->private_data;
	WARN_ON(!fdata->streaming);
	WARN_ON(!adc_dev.streaming);
	reset_dma_buffers();
	fdata->streaming = 0;
	adc_dev.streaming = 0;
}

static int jsadc_open(struct inode *inode, struct file *filp)
{
	dev_dbg(adc_dev.device, "jsadc_open for inode %lu\n", inode->i_ino);
	filp->private_data = kzalloc(sizeof(struct jsadc_fdata), GFP_KERNEL);
	if (!filp->private_data) {
		dev_err(adc_dev.device, "could not allocate memory\n");
		return -ENOMEM;
	}
	return 0;
}

static int jsadc_release(struct inode *inode, struct file *filp)
{
	unsigned long irqstate;
	struct jsadc_fdata *fdata = filp->private_data;

	dev_dbg(adc_dev.device, "jsadc_release for inode %lu\n", inode->i_ino);

	spin_lock_irqsave(&adc_dev.lock, irqstate);
	if (fdata->streaming) {
		streaming_stop(filp);
	}
	spin_unlock_irqrestore(&adc_dev.lock, irqstate);

	kfree(fdata);
	filp->private_data = NULL;
	return 0;
}

static int jsadc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int res;
	ssize_t const sz = vma->vm_end - vma->vm_start;
	if (sz <= 0 || sz > JSADC_SIZE_ALLBUFS) {
		dev_err(adc_dev.device,
		        "invalid vma region: 0x%08lx-0x%08lx.\n",
		        vma->vm_start, vma->vm_end);
		return -EINVAL;
	}
	if (vma->vm_flags & VM_WRITE) {
		return -EPERM;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	res = remap_pfn_range(vma, vma->vm_start, PFN_DOWN(adc_dev.bus),
			    vma->vm_end - vma->vm_start , vma->vm_page_prot);
	if (res) {
		dev_err(adc_dev.device,
		        "remap_pfn_range failed: %d\n", res);
		return -EAGAIN;
	}
	return 0;
}

/**
 * Returns 1 if streaming is occurring on ADC0.
 * Must be called with adc_dev.lock held.
 */
static inline int is_streaming_conflicting_with_monitoring(void)
{
	if (adc_dev.streaming && (adc_dev.cur_src == JSADC_SRC_VBAT ||
	                          adc_dev.cur_src == JSADC_SRC_IJUMP)) {
		return 1;
	}
	return 0;
}

static int get_data(struct file *filp)
{
	int ret = 0;
	unsigned long irqstate;
	int adc0_mux_set = 0;

	spin_lock_irqsave(&adc_dev.lock, irqstate);
	BUG_ON(adc_dev.monitoring);
	adc_dev.monitoring = 1;
	if (!is_streaming_conflicting_with_monitoring()) {
		adc0_mux_set = 1;
	}
	spin_unlock_irqrestore(&adc_dev.lock, irqstate);

	/* Reset data and pass number */
	adc_dev.monitor_pass = 0;
	memset(&adc_dev.monitor_data, 0, sizeof(adc_dev.monitor_data));

	if (adc0_mux_set) {
		gpio_set_value(adc_dev.gpio_mux_vbat_jump,
		                      ADC0_MUX_VBAT);
	}
	gpio_set_value(adc_dev.gpio_mux_wheels, ADC1_MUX_LEFT);

	/* trigger first ADC conversion */
	aai_writel(1, AAI_ADC_MONITOR);

	/* wait for transfer to be complete */
	wait_for_completion(&adc_dev.completion);
	return ret;
}

/**
 * Read data from the first pass of ADC MONITORING.
 * @param data0 Value read from AAI_ADC_DATA.
 */
static void read_monitor_data_first_pass(uint32_t data0)
{
	struct jsadc_data *mondata = &adc_dev.monitor_data;
	/* Streaming is running on ADC0, there will be only one pass.
	 * Read all data. */
	if (is_streaming_conflicting_with_monitoring()) {
		if (adc_dev.cur_src == JSADC_SRC_VBAT) {
			mondata->has_vbat = 1;
			mondata->vbat = (uint16_t)data0;
		} else {
			mondata->has_ijump = 1;
			mondata->ijump = (uint16_t)data0;
		}
		mondata->has_lwheel = 1;
		mondata->lwheel = (uint16_t)aai_readl(AAI_ADC_DATA + 4);
		mondata->has_rwheel = 1;
		mondata->rwheel = (uint16_t)aai_readl(AAI_ADC_DATA + 8);
	} else {
		/* When streaming is not active on ADC0, VBAT is always
		 * chosen for the first monitoring pass. */
		mondata->has_vbat = 1;
		mondata->vbat = data0;
		/* We leave wheels data for the second pass. */
	}
}

/**
 * Read data from the second pass of ADC MONITORING.
 * @param data0 Value read from AAI_ADC_DATA.
 */
static void read_monitor_data_second_pass(uint32_t data0)
{
	struct jsadc_data *mondata = &adc_dev.monitor_data;
	WARN_ON(!mondata->has_vbat && !mondata->has_ijump);
	if (!mondata->has_vbat) {
		mondata->has_vbat = 1;
		mondata->vbat = (uint16_t)data0;
	} else if (!mondata->has_ijump) {
		mondata->has_ijump = 1;
		mondata->ijump = (uint16_t)data0;
	}
	mondata->has_lwheel = 1;
	mondata->lwheel = (uint16_t)aai_readl(AAI_ADC_DATA + 4);
	mondata->has_rwheel = 1;
	mondata->rwheel = (uint16_t)aai_readl(AAI_ADC_DATA + 8);
}

static void trigger_monitor_second_pass(void)
{
	/* We can't be streaming on VBAT of IJUMP as we will flip the mux */
	WARN_ON(is_streaming_conflicting_with_monitoring());
	adc_dev.monitor_pass = 1;
	/* Set ADC0 mux */
	if (!adc_dev.monitor_data.has_vbat) {
		gpio_set_value(adc_dev.gpio_mux_vbat_jump,
		                      ADC0_MUX_VBAT);
	} else if (!adc_dev.monitor_data.has_ijump) {
		gpio_set_value(adc_dev.gpio_mux_vbat_jump,
		                      ADC0_MUX_IJUMP);
	} else {
		dev_WARN(adc_dev.device,
		         "vbat and ijump should not be both already filled\n");
	}
	aai_writel(1, AAI_ADC_MONITOR);
}

static long jsadc_ioctl(struct file *filp,
			    unsigned int cmd,
			    unsigned long arg)
{
	int res = 0;
	int idx;
	unsigned long sp_lock;
	struct jsadc_fdata *fdata = filp->private_data;

	switch (cmd) {
	case JSADC_GET_DATA:

		res = mutex_lock_interruptible(&adc_dev.monitor_mut);
		if (res)
			return res;

		res = get_data(filp);
		if (res)
			goto exit_get_data;

		if (copy_to_user((void __user *) arg,
				(void *)&adc_dev.monitor_data,
				sizeof(struct jsadc_data))) {
			res = -EFAULT;
		}

exit_get_data:
		mutex_unlock(&adc_dev.monitor_mut);
		return res;
		break;

	case JSADC_GETBUFFER:
		/*
		 * pin is on the next free space,
		 * pin-1 is the latest buffer available for userspace,
		 */
		spin_lock_irqsave(&adc_dev.lock, sp_lock);
		if (!fdata->streaming) {
			spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
			return -EBADFD;
		}
		if (is_dma_available_for_user()) {
			struct jsadc_timed_buf tb = {
				.index = mem.usr_ptr,
				.missed = metadata[mem.usr_ptr].missed,
				.src = metadata[mem.usr_ptr].src
			};
			/* Reset missed counter (we already copied it) */
			metadata[mem.usr_ptr].missed = 0;

			mem.usr_ptr = (mem.usr_ptr + 1) % JSADC_NUM_BUFS;
			if (mem.usr_ptr == mem.pin)
				mem.usr_full = 1;

			spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
			res = copy_to_user((void __user *)arg,
			                   (void *)&tb, sizeof(tb));
			if (res)
				res = -EFAULT;
			return res;
		}
		/* No done buffer available. */
		spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
		return -EAGAIN;
		break;

	case JSADC_RELEASEBUFFER:

		idx = (int)arg;
		if ((idx < 0) || (idx >= JSADC_NUM_BUFS))
			return -EINVAL;

		spin_lock_irqsave(&adc_dev.lock, sp_lock);
		if (!fdata->streaming) {
			spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
			return -EBADFD;
		}
		/*
		 * The next IF describes the following conditions:
		 * Buffer must be released in order
		 * Cannot release a buffer the userspace didn't GET beforehand
		 */
		if ((idx != mem.pout) || (mem.usr_ptr == mem.pout)) {
			spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
			return -EINVAL;
		}

		if (mark_dma_free()) {
			/* Worst that can happen: can't free memory because
			 * it's already all freed. We can go on safely, but
			 * let's output a scary warning so that the user knows
			 * there is something wrong in the driver. */
			dev_WARN(adc_dev.device, "mark_dma_free() failed!\n");
		}
		mem.usr_full = 0;

		/* Resume streaming if suspended because of an overrun */
		if (adc_dev.overrun &&
		    (get_dma_free() >= 2)) {
			dev_info(adc_dev.device, "resuming from overrun\n");
			adc_dev.overrun = 0;
		}

		spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
		break;

	case JSADC_START:
		res = mutex_lock_interruptible(&adc_dev.monitor_mut);
		if (res)
			return res;

		spin_lock_irqsave(&adc_dev.lock, sp_lock);
		if (adc_dev.streaming) {
			spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
			mutex_unlock(&adc_dev.monitor_mut);
			return -EBUSY;
		}
		streaming_start(filp);
		spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
		mutex_unlock(&adc_dev.monitor_mut);
		break;

	case JSADC_STOP:
		spin_lock_irqsave(&adc_dev.lock, sp_lock);
		if (!fdata->streaming) {
			spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
			return -EBADFD;
		}
		streaming_stop(filp);

		spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
		break;

	case JSADC_SET_SRC:
		if (!(arg == JSADC_SRC_VBAT) &&
		    !(arg == JSADC_SRC_IJUMP) &&
		    !(arg == JSADC_SRC_LWHEEL) &&
		    !(arg == JSADC_SRC_RWHEEL)) {
			return -EINVAL;
		}

		spin_lock_irqsave(&adc_dev.lock, sp_lock);
		if (fdata->streaming) {
			spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
			return -EBUSY;
		}
		fdata->src = arg;
		spin_unlock_irqrestore(&adc_dev.lock, sp_lock);
		break;

	}

	return 0;
}

static unsigned int jsadc_poll(struct file *filp, poll_table *wait)
{
	unsigned long irqstate;
	int retval = 0;
	struct jsadc_fdata *fdata = filp->private_data;

	poll_wait(filp, &adc_dev.waitq, wait);

	spin_lock_irqsave(&adc_dev.lock, irqstate);
	if (!fdata->streaming) {
		retval = POLLERR;
	} else if (is_dma_available_for_user()) {
		retval = POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&adc_dev.lock, irqstate);
	return retval;
}

static const struct file_operations jpsumo_adc_fops = {
	.open           = jsadc_open,
	.release        = jsadc_release,
	.mmap           = jsadc_mmap,
	.unlocked_ioctl = jsadc_ioctl,
	.poll           = jsadc_poll,
	.owner = THIS_MODULE
};

static struct miscdevice jpsumo_adc_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = JSADC_DEVICE_NAME,
	.fops  = &jpsumo_adc_fops,
};

/* When an ultrasound IRQ occurs, the value we wrote into AAI_DMAFA_ULTRA is
 * copied to AAI_DMASA_ULTRA and a DMA transfer is started to this address.
 * While this transfer is running, the IRQ handler must acknowledge the IRQ
 * and write a new address into AAI_DMASA_ULTRA for the next transfer.
 *
 * The user manual isn't very clear about the steps to follow in order to
 * arm a new transfer, but this sequence seems to work:
 * - Acknowledge the interrupt in AAI_DMA_INT_ACK
 * - Write the new address into AAI_DMAFA_ULTRA
 * - Write into AAI_ADC_ULTRA to rearm the DMA for the next transfer
 *
 * When this IRQ handler is run, mark_dma_done() increments mem.pin to make
 * the transfer that just finished (the one that triggerred the IRQ)
 * available to userspace. So mem.pin now points to the area currently being
 * filled by the DMA, and (mem.pin + 1) % JSADC_NUM_BUFS is where the next
 * transfer will occur.
 *
 * To sum up, after mark_dma_done():
 * pin - 1 <-- last complete transfer (done)
 * pin <-- current transfer (free)
 * pin + 1 <-- next transfer (free)
 *
 * done == chunk contains fresh data ready to be processed in userspace, not
 *         a target for DMA operations
 * free == chunk was processed and released by userspace, it will be used for
 *         DMA transfer
 *
 * In case there is no free buffer available for the next transfer, the overrun
 * condition is set and we set the next transfer to occur in the scrap buffer
 * until the overrun condition is cleared. Doing this allows us to count how
 * many buffers were missed during the overrun condition, which would not be
 * possible is we stopped acquiring data.
 * As soon as a buffer becomes free (i.e. the user performs the
 * JSADC_RELEASEBUFFER ioctl), the overrun condition is cleared, and the
 * recovery flag is set to let the IRQ handler know that it should consider
 * the next completed buffer as missed (since it's the scrap buffer), clear
 * the recovery flag and set the address of the next transfer to the free
 * buffer.
 */
static inline void ultrasound_handler(int irq, void *dev_id)
{
	aai_writel(AAI_ITS_ULTRA, AAI_DMA_INT_ACK);
	mmiowb();
	if (!adc_dev.streaming)
		return;

	if (get_dma_free() > 0) {
		metadata[mem.pin].src = adc_dev.cur_src;
		WARN_ON(mark_dma_done());
		wake_up_interruptible(&adc_dev.waitq);
	}

	/* If we have no free buffer left, perform next transfer to the scrap
	 * buffer. */
	if (get_dma_free() < 2) {
		if (!adc_dev.overrun) {
			dev_warn(adc_dev.device, "buffer overrun!\n");
			adc_dev.overrun = 1;
		}
		/* Increment the counter for the next buffer userspace will
		 * GET (after it has RELEASE'd it). */
		metadata[mem.pout].missed++;

		aai_writel(SCRAPBUF_OFFSET + adc_dev.bus, AAI_DMAFA_ULTRA);
		aai_writel(1, AAI_ADC_ULTRA);
		mmiowb();
	} else {
		/* Load next DMA transfer */
		uint32_t fa_off;
		fa_off = bufidx2off(mem.pin + 1) % JSADC_SIZE_ALLBUFS;
		dev_dbg(adc_dev.device, "FA: 0x%.8x SA: 0x%.8x\n", fa_off,
		        aai_readl(AAI_DMASA_ULTRA) - adc_dev.bus);
		aai_writel(fa_off + adc_dev.bus, AAI_DMAFA_ULTRA);
		aai_writel(1, AAI_ADC_ULTRA);
		mmiowb();
	}
}

static inline void monitor_handler(int irq, void *dev_id)
{
	/* Read ADC0. This will acknowledge the interrupt. */
	uint32_t data0 = aai_readl(AAI_ADC_DATA);
	switch(adc_dev.monitor_pass) {
		case 0:
			/* Read data from this pass */
			read_monitor_data_first_pass(data0);
			/* Streaming is inactive, perform a second pass. */
			if (!is_streaming_conflicting_with_monitoring()) {
				trigger_monitor_second_pass();
			} else {
				/* Wake the thread waiting for completion */
				complete(&adc_dev.completion);
				adc_dev.monitoring = 0;
			}
			break;
		case 1:
			read_monitor_data_second_pass(data0);
			/* Operation complete, release the thread
			 * waiting for us. */
			complete(&adc_dev.completion);
			adc_dev.monitoring = 0;
			break;
		default:
			dev_WARN(adc_dev.device, "Invalid monitor_pass: %d\n",
					adc_dev.monitor_pass);
			break;
	}
}

static irqreturn_t jpsumo_adc_it(int irq, void *dev_id)
{
	uint32_t reg;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;

	spin_lock_irqsave(&adc_dev.lock, flags);

	/* read interrupt status register*/
	reg = aai_readl(AAI_ITS);
	/* Ultrasound (streaming mode) */
	if (reg & AAI_ITS_ULTRA) {
		dev_dbg(adc_dev.device, "AAI_ITS_ULTRA\n");
		ultrasound_handler(irq, dev_id);
		ret = IRQ_HANDLED;
	}
	/* Monitoring */
	if (reg & AAI_ITS_MONITOR) {
		dev_dbg(adc_dev.device, "AAI_ITS_MONITOR\n");
		monitor_handler(irq, dev_id);
		ret = IRQ_HANDLED;
	}

	spin_unlock_irqrestore(&adc_dev.lock, flags);
	return ret;
}

static void hw_init(void)
{
	int reg;

	/* set ADC configuration */
	reg = AAI_ADC_MODES_MONITORING | AAI_ADC_MODES_ULTRA_SOUNDS;
	aai_writel(reg, AAI_ADC_MODES);

	/* Configure AAI clocks for ADC */
	reg = aai_readl(AAI_CFG);
	reg |= AAI_CFG_MUSIC_ICH0;
	reg |= AAI_CFG_RUN_MULT;

	/* Ultra sound Frequency :
	 * ultra sound will work at freq = 3 * base_frequency
	 *
	 * if AAI_CFG_COMPACT is set,
	 * base_frequency = 44099.507Hz (~44.1kHz)
	 *
	 * if AAI_CFG_COMPACT is cleared,
	 * base_frequency = 48004.150Hz (~48kHz)
	 */
	reg |= AAI_CFG_COMPACT;		/* => 3*44.1kHz = 132.3kHz */
	aai_writel(reg, AAI_CFG);

	reg = aai_readl(AAI_DMACTL);
	reg |= AAI_DMA_CTRL_ULTRA;
	aai_writel(reg, AAI_DMACTL);
	mmiowb();
}

static int jpsumo_adc_probe(struct platform_device *pdev)
{
	int32_t res;
	struct resource *resm;
	struct parrot_jsadc_platform_data *pdata = pdev->dev.platform_data;

	adc_dev.device = &pdev->dev;

	dev_dbg(adc_dev.device, "probing\n");

	if (!pdata) {
		dev_err(adc_dev.device, "No platform data!\n");
		return -EINVAL;
	}
	if (unlikely(!gpio_is_valid(pdata->gpio_mux_vbat_jump))) {
		dev_err(adc_dev.device, "Invalid GPIO for ADC0 mux: %d\n",
				pdata->gpio_mux_vbat_jump);
		return -EINVAL;
	}
	if (unlikely(!gpio_is_valid(pdata->gpio_mux_wheels))) {
		dev_err(adc_dev.device, "Invalid GPIO for ADC1-2 mux: %d\n",
				pdata->gpio_mux_wheels);
		return -EINVAL;
	}
	adc_dev.gpio_mux_vbat_jump = pdata->gpio_mux_vbat_jump;
	adc_dev.gpio_mux_wheels    = pdata->gpio_mux_wheels;
	dev_dbg(adc_dev.device, "GPIO MUX: ADC0: %d, ADC1-2: %d\n",
			adc_dev.gpio_mux_vbat_jump, adc_dev.gpio_mux_wheels);

	gpio_direction_output(adc_dev.gpio_mux_vbat_jump, ADC0_MUX_VBAT);
	gpio_direction_output(adc_dev.gpio_mux_wheels, ADC1_MUX_LEFT);

	/* HACK AAI ressource does not declare coherent dma mask */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	init_waitqueue_head(&adc_dev.waitq);

	init_completion(&adc_dev.completion);
	spin_lock_init(&adc_dev.lock);
	mutex_init(&adc_dev.monitor_mut);

	/* power on & clock the AAI IP */
	adc_dev.clock = clk_get(NULL, "aai");
	if (IS_ERR(adc_dev.clock)) {
		pr_err("Unable to get clock: errno %d\n", (int32_t)adc_dev.clock);
		res = -ENODEV;
		goto err_gclk;
	}

	res = clk_enable(adc_dev.clock);
	if (res) {
		pr_err("Unable to enable clock: errno %d\n", res);
		goto err_eclk;
	}

	resm = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resm) {
		pr_err("Unable to get ressource\n");
		res = -ENXIO;
		goto err_reg;
	}
	pr_debug("ressources get 0x%x 0x%x\n", resm->end, resm->start);
	adc_dev.mem_start = (uint32_t)resm->start;
	adc_dev.mem_end   = (uint32_t)resm->end;

	/*map the aai registers region*/
	adc_dev.aai = (int8_t *)ioremap(resm->start, resm->end - resm->start);
	if (adc_dev.aai == NULL) {
		res = -ENXIO;
		goto nomap;
	}

	/* alloc dma memory*/
	adc_dev.adr = dma_alloc_coherent(adc_dev.device, PAGE_ALIGN(DMA_ALLOC_SIZE),
				       &adc_dev.bus, GFP_USER);
	if (!adc_dev.adr) {
		res = -ENOMEM;
		goto nodma;
	}

	pr_debug("%d dma_alloc_coherent Virt 0x%x, phy 0x%x\n",
		 __LINE__, (unsigned int)adc_dev.adr,
		 (unsigned int)adc_dev.bus);

	/*Get interrupt*/
	resm =  platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!resm) {
		pr_err("Unable to get irq ressource\n");
		res = -ENXIO;
		goto noirq;
	}

	adc_dev.numirq = resm->start;

	/* Set irq handler */
	res = request_irq(adc_dev.numirq, &jpsumo_adc_it, IRQF_SHARED,
			  JSADC_DEVICE_NAME, adc_dev.device);
	if (res)
		goto noirq;

	res = misc_register(&jpsumo_adc_miscdev);
	if (res < 0)
		goto nochdev;

	hw_init();

	return 0;

nochdev:
	free_irq(adc_dev.numirq, adc_dev.device);
noirq:
	dma_free_coherent(adc_dev.device,
			  PAGE_ALIGN(DMA_ALLOC_SIZE), adc_dev.adr, adc_dev.bus);
nodma:
	iounmap(adc_dev.aai);
nomap:
err_reg:
	clk_disable(adc_dev.clock);
err_eclk:
	clk_put(adc_dev.clock);
err_gclk:
	dev_err(&pdev->dev, "probe failed, error %d\n", res);
	return res;
}

static int jpsumo_adc_cleanup(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Removing driver...\n");

	if (adc_dev.adr)
		dma_free_coherent(adc_dev.device,
		                  PAGE_ALIGN(DMA_ALLOC_SIZE),
		                  adc_dev.adr, adc_dev.bus);

	free_irq(adc_dev.numirq, adc_dev.device);
	iounmap(adc_dev.aai);
	clk_disable(adc_dev.clock);
	clk_put(adc_dev.clock);
	misc_deregister(&jpsumo_adc_miscdev);

	return 0;
}

/* We use the aai ressource s*/
static struct platform_driver jsadc_driver = {
	.remove = &jpsumo_adc_cleanup,
	.probe  = &jpsumo_adc_probe,
	.driver = {
		.name  = JSADC_DEVICE_NAME,
		.owner = THIS_MODULE,
	},
};

static int usnd_init(void)
{
	return platform_driver_register(&jsadc_driver);
}
module_init(usnd_init);

static void usnd_exit(void)
{
	platform_driver_unregister(&jsadc_driver);
}
module_exit(usnd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Parrot");

