/**
 ******************************************************************************
 * @file ultra_snd.c
 * @brief Delos ultra sound(aai + spi)  driver
 *
 * Copyright(C) 2013 Parrot S.A.
 *
 * @author     Samir Ammenouche <samir.ammenouche@parrot.com>
 * @date       2013-04-02
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
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uaccess.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include <mach/dma-pl08x.h>
#include <mach/regs-aai-p6.h>
#include <mach/ultra_snd.h>

#include "timer.h"
#include "ultra_snd.h"

/* SPI base address */
#define P6_SPI1_BAD	0xd00c0000
#define SPI_CTRL	0x0000
#define SPI_SPEED	0x0004
#define SPI_STATUS	0x0008
#define SPI_SIZE	0x000c
#define SPI_THRES_RX	0x0010
#define SPI_THRES_TX	0x0014
#define SPI_DATA	0x0040

#define aai_writel(_value_, _reg_)\
	writel(_value_, _reg_ + u_snd.map)

#define aai_readl(_reg_) readl(_reg_ + u_snd.map)

#define spi_writel(_value_, _reg_)\
	writel(_value_, _reg_ + spi_dev.map)

#define spi_readl(_reg_) readl(_reg_ + spi_dev.map)

/*
 * usnd_device is the struct which contains the informations needed\
 * by the driver.
 * \struct lock: used by the spinlock
 * \struct waitq: the wait queue used to poll
 * \struct adr: the dmable memory area
 * \struct map: the aai mapped register
 * \struct adc_status: adc mgmt status bit field
 * \struct bus: the physical address of the dmable memory
 * \struct size the user requested size
 * \struct irqcount: the number of needed irq to fill the dmable memory
 * \struct numirq: the aai irq number
 * \struct mem_start: physical start address of registers
 * \struct mem_end: physical end address of registers
 * \struct adc_data: structure with 50hz results in
 * \struct vbat_value: read on adc 0 on interruption
 */
struct usnd_device {
	struct device			*device;
	struct clk			*clock;
	spinlock_t			lock;
	wait_queue_head_t		waitq;
	uint32_t			*adr;
	uint8_t				*map;
	uint8_t				adc_status;
	dma_addr_t			bus;
	uint32_t			size;
	int32_t				irqcount;
	uint32_t			numirq;
	uint32_t			mem_start;
	uint32_t			mem_end;
	uint16_t			vbat_value;
	uint8_t                         temperature_adc_num;
	uint16_t			temperature_value;

	struct completion		completion;
	struct semaphore		adc_lock;
};

/*
 * define usnd_device.adc_status bit field
 */
#define ADC_VBAT_ENABLE             0x08	/* Only ADC0 vbat enable */
#define ADC_VBAT_USE_BAT_REG        0x10	/* Real battery mode */

#define MAX_AAI_ADC_INDEX 8
/*
 * spi_device is the used struct to store the SPI informations
 * @struct map: the spi mapped register
 * @struct len: the len of the spi pattern to send
 * @struct adr: the allocated space for the spi pattern to be send
 * */

struct spi_device {
	struct clk		*clock;
	uint8_t			*map;
	uint8_t			len;
	/* DMA-related data */
	uint32_t		*dmabuf_cpu;
	dma_addr_t		dmabuf_bus;
	unsigned int		dmachan;
	struct completion	dma_completion;
	/* This mutex protects from performing more than one USND_PULSES or
	 * USND_RAW ioctl at the same time.
	 */
	struct mutex		mut;
};
static struct spi_device spi_dev;

/*
 * struct mem is the internal structure used to track dma memory allocation
 * @struct pin: is the input offset pointer relative to the start DMA address
 * @struct pout is the output offset pointer relative to the start DMA address
 * @struct free: is the number of free element, read/write to this value must
 * be mutually excluded
 */
struct mem {
	int pin;
	int pout;
	int usr_ptr;
	int free;
};

static struct usnd_device u_snd = {
	.irqcount   = 0,
	.adr	    = NULL,
	.map	    = NULL,
	.size	    = 0,
	.adc_status = 0
};

static int ultra_snd_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ultra_snd_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ultra_snd_mmap(struct file *filep, struct vm_area_struct *vma)
{
	size_t const sz = vma->vm_end - vma->vm_start;
	if (sz <= 0) {
		pr_err("invalid vma region: 0x%08lx-0x%08lx.\n",
		       vma->vm_start, vma->vm_end);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, PFN_DOWN(u_snd.bus),
			    vma->vm_end - vma->vm_start , vma->vm_page_prot)) {
		pr_err("%s: io_remap_pfn_range failed\n", __func__);
		return -EAGAIN;
	}
	return 0;
}


static int temperature_get_data(struct usnd_device *usnd)
{
	int ret = 0;

	ret = down_interruptible(&usnd->adc_lock);
	if (ret < 0)
		return ret;

	/* trigger ADC conversion */
	aai_writel(1, AAI_ADC_MONITOR);

	/* wait for transfer to be complete */
	wait_for_completion(&usnd->completion);

	up(&usnd->adc_lock);
	return ret;
}

static void battery_config(int bat_adc_num)
{
	int reg;
	unsigned long sp_lock;

	spin_lock_irqsave(&u_snd.lock, sp_lock);

	/* set AAI configuration */
	reg = aai_readl(AAI_ADC_MODES);

	/* configure battery input*/
	reg |= AAI_ADC_MODES_BATTERY_ALARM;
	/* mandatory to check battery*/
	reg |= AAI_ADC_MODES_MONITORING;
	reg &= ~(AAI_ADC_MODES_BATTERY_IN);
	reg |= AAI_ADC_MODES_BATTERY_IN & (bat_adc_num << 11);
	u_snd.adc_status |= ADC_VBAT_USE_BAT_REG;

	u_snd.adc_status |= ADC_VBAT_ENABLE;
	aai_writel(reg, AAI_ADC_MODES);
	wmb();

	reg = aai_readl(AAI_CFG);
	reg |= AAI_CFG_MUSIC_ICH0;
	reg |= AAI_CFG_RUN_MULT;
	reg &= ~(AAI_CFG_COMPACT);
	aai_writel(reg, AAI_CFG);
	wmb();

	spin_unlock_irqrestore(&u_snd.lock, sp_lock);

	pr_debug("AAI_CFG 0x320: 0x%x\n", aai_readl(AAI_CFG));
	pr_debug("AAI_ADC_MODES 0x39c: 0x%x\n", aai_readl(AAI_ADC_MODES));
}

static void ultra_sound_config(void)
{
	int reg;

	/* set AAI configuration */
	reg = aai_readl(AAI_ADC_MODES);
	reg |= AAI_ADC_MODES_MONITORING;

	/* ultra sound is connected in adc 0 */
	reg |= AAI_ADC_MODES_ULTRA_SOUNDS;
	reg &= ~(AAI_ADC_MODES_ULTRA_IN);
	aai_writel(reg, AAI_ADC_MODES);
	wmb();

	reg = aai_readl(AAI_CFG);
	reg |= AAI_CFG_MUSIC_ICH0;
	reg |= AAI_CFG_RUN_MULT;
	reg |= AAI_CFG_COMPACT;
	aai_writel(reg, AAI_CFG);
	wmb();

	reg = aai_readl(AAI_DMACTL);
	/* the Ghost bit of jeanpol not documented! */
	reg |= AAI_DMA_CTRL_ULTRA;
	aai_writel(reg, AAI_DMACTL);
	wmb();

	pr_debug("Line %d: AAI_ADC_MODES 0x39c: 0x%x\n",
		 __LINE__, aai_readl(AAI_ADC_MODES));
	pr_debug("Line %d: AAI_CFG 0x320: 0x%x\n",
		 __LINE__, aai_readl(AAI_CFG));
	pr_debug("Line %d: AAI_DMACTL 0x37C: 0x%x\n",
		 __LINE__, aai_readl(AAI_DMACTL));
}

static void delos_temperature_config(int temperature_adc_num)
{
	int reg;
	unsigned long sp_lock;

	spin_lock_irqsave(&u_snd.lock, sp_lock);

	u_snd.temperature_adc_num = temperature_adc_num;

	/* set AAI configuration */
	reg = aai_readl(AAI_ADC_MODES);
	reg |= AAI_ADC_MODES_MONITORING;

	aai_writel(reg, AAI_ADC_MODES);
	wmb();

	spin_unlock_irqrestore(&u_snd.lock, sp_lock);
}

static void adc_trigger(void)
{
	unsigned long state;
	spin_lock_irqsave(&u_snd.lock, state);
	/*Enable ADC*/
	aai_writel(1, AAI_ADC_ULTRA);
	spin_unlock_irqrestore(&u_snd.lock, state);
}

static void perform_dma(unsigned int size, unsigned int tholdcs, bool inc)
{
	struct pl08x_dma_cfg dma_cfg;
	uint32_t ctrl, ctrl_save;
	unsigned long irqstate;

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.src_addr = spi_dev.dmabuf_bus;
	dma_cfg.dst_addr = P6_SPI1_BAD + SPI_DATA;
	dma_cfg.src_periph = _PL080_PERIPH_MEM;
	dma_cfg.dst_periph = _PL080_PERIPH_SPI1;
	dma_cfg.cxctrl = (union pl08x_dma_cxctrl){
		.si = inc ? 1 : 0,
		.di = 0,
		.transize = size,
		.swidth = 2,
		.dwidth = 2,
		.sbsize = 4,
		.dbsize = 4,
		.sahb = 0,
		.sahb = 0
	};
	pl08x_dma_start(spi_dev.dmachan, &dma_cfg);

	adc_trigger();

	/* Setup SPI controller */
	spin_lock_irqsave(&u_snd.lock, irqstate);
	ctrl_save = ctrl = spi_readl(SPI_CTRL);
	/* Set THOLDCS value. */
	ctrl &= ~0x000000f0;
	ctrl |= (tholdcs & 0xf) << 4;
	/* Enable DMA mode */
	ctrl |= (1 << 14); /* DMA_EN */
	ctrl |= (1 << 15); /* DMA_TX_MODE */
	spi_writel(ctrl, SPI_CTRL);
	spin_unlock_irqrestore(&u_snd.lock, irqstate);

	wait_for_completion(&spi_dev.dma_completion);
	pl08x_dma_wait(spi_dev.dmachan);

	spin_lock_irqsave(&u_snd.lock, irqstate);
	spi_writel(ctrl_save, SPI_CTRL);
	spin_unlock_irqrestore(&u_snd.lock, irqstate);
}

static void perform_pulses_dma(unsigned int pulses)
{
	unsigned int tholdcs;

	spi_dev.dmabuf_cpu[0] = 0xff;

	/* Set THOLDCS to 7 -> Add 8 SCLK cycles between each byte.
	 * That way, we have eight 1's sent on MOSI, followed by eight
	 * 0's, effectively forming a pulse at frequency
	 * (SPICLK / (8 * 2)) == 641991 / 16 ~= 40 KHz */
	tholdcs = 7;

	/* Perform DMA, do not increment source address. */
	perform_dma(pulses, tholdcs, false);
}

static inline void perform_raw_dma(unsigned int size, unsigned int tholdcs)
{
	perform_dma(size, tholdcs, true);
}

static long ultra_snd_ioctl(struct file *filp,
			    unsigned int cmd,
			    unsigned long arg)
{
	uint32_t reg, res = 0, dma;
	struct usnd_raw_buf raw_buf;
	unsigned long sp_lock;

	switch (cmd) {
	case BATTERY_INIT:
		if (access_ok(VERIFY_READ, (void *)arg, sizeof(uint32_t))) {
			res = copy_from_user((void *) &reg,
					     (void __user *)arg,
					     sizeof(uint32_t));
			if (res)
				return -EINVAL;
		}
		if (!(u_snd.adc_status & ADC_VBAT_ENABLE))
			battery_config(reg);
		break;

	case TEMPERATURE_INIT:
		if (access_ok(VERIFY_READ, (void *)arg, sizeof(uint32_t))) {
			res = copy_from_user((void *) &reg,
					(void __user *)arg,
					sizeof(uint32_t));
			if (res)
				return -EINVAL;
		}

		if (reg > MAX_AAI_ADC_INDEX)
			return -EINVAL;

		delos_temperature_config(reg);
		break;
	case USND_SETSIZE:
		/* TODO Pimo: DELOS_INIT??? */
		spin_lock_irqsave(&u_snd.lock, sp_lock);
		res = copy_from_user((void *)&(u_snd.size),
				     (void __user *)arg,
				     sizeof(uint32_t));

		if ((res != 0) || (u_snd.size > SIZEMAX)) {
			spin_unlock_irqrestore(&u_snd.lock, sp_lock);
			return -EINVAL;
		}

		memset(u_snd.adr, 0, u_snd.size);

		ultra_sound_config();
		aai_writel(u_snd.bus, AAI_DMASA_ULTRA);
		pr_debug("%d AAI_DMASA_ULTRA 0x%x\n",
			 __LINE__, aai_readl(AAI_DMASA_ULTRA));

		/* The obtained SIZE is expressed in bytes.
		 * Each DMA transfer moves 16 *32bits words = 64 bytes.
		 * We must first set the AAI ULTRA DMA COUNT register.
		 * This register contain the number of DMA transfers performed.
		 * So, SIZE is devided by 64. This register is limited to
		 * 2^7=128 tranfers. If needs more, the variable irqcount
		 * contains the number of DMA tansfer
		 */
		dma = u_snd.size >> 6;
		if (dma > 128) {
			/* The requested size > maximum transfarable by DMA(8kB)
			 * Needs(asize / 8K) it.
			 * Following address is(base + 8K)
			 * The number of dma transfer is maximum: 8K.
			 */
			u_snd.irqcount = dma >> 7;
			pr_debug("%d u_snd.irqcount %d\n",
				 __LINE__, u_snd.irqcount);
			aai_writel(u_snd.bus + (1 << MAXDMA), AAI_DMAFA_ULTRA);
			aai_writel(0x7, AAI_ULTRA_DMA_COUNT);
		} else	{
			/* The requested size < maximum transfarable by DMA
			 * Needs just one it. Following address is meaningless
			 * The number of dma transfer is:
			 * log2(size) - 1
			 */
			u_snd.irqcount = 1;
			aai_writel(u_snd.bus + (1 << 13), AAI_DMAFA_ULTRA);
			/*
			 * ffs function returns log2 of dma
			 * dma is power 2(mandatory)
			 */
			aai_writel(ffs(dma) - 1, AAI_ULTRA_DMA_COUNT);
		}
		reg = aai_readl(AAI_ULTRA_DMA_COUNT);
		pr_debug("%d AAI_ULTRA_DMA_COUNT 0x%x, irqcount %d\n",
			 __LINE__, reg, u_snd.irqcount);
		spin_unlock_irqrestore(&u_snd.lock, sp_lock);
		break;

	case USND_COPYSAMPLE:
		if (access_ok(VERIFY_WRITE, (void *)arg, u_snd.size)) {
			pr_debug("%d: arg 0x%x, u_snd.adr 0x%x"
				   "u_snd.size 0x%x\n", __LINE__,
				   (unsigned int) arg,
				   (unsigned int)u_snd.adr,
				   (unsigned int)u_snd.size);
			res = copy_to_user((void __user *)arg,
					   (void *)u_snd.adr, u_snd.size);
			if (!res)
				return 0;
		}
		return -EINVAL;
		break;

	case USND_PULSES:
		res = mutex_lock_interruptible(&spi_dev.mut);
		if (res)
			return res;
		if (arg == 0) {
			res = 0;
			goto exit_usnd_pulses;
		} else if (arg > USND_MAX_PULSES) {
			res = -EINVAL;
			goto exit_usnd_pulses;
		}

		perform_pulses_dma(arg);
		res = 0;
exit_usnd_pulses:
		mutex_unlock(&spi_dev.mut);
		return res;
		break;

	case USND_RAW:
		res = mutex_lock_interruptible(&spi_dev.mut);
		if (res)
			return res;
		if (copy_from_user(&raw_buf, (void __user *)arg,
					sizeof(struct usnd_raw_buf))) {
			res = -EFAULT;
			goto exit_usnd_raw;
		}
		if (raw_buf.length == 0) {
			res = 0;
			goto exit_usnd_raw;
		} else if (raw_buf.length > USND_MAX_RAW_BUFLEN) {
			res = -EINVAL;
			goto exit_usnd_raw;
		} else if ((raw_buf.length & 0x3) != 0) {
			res = -EINVAL;
			goto exit_usnd_raw;
		}
		if (raw_buf.tholdcs > 0xf) {
			res = -EINVAL;
			goto exit_usnd_raw;
		}
		/* Copy to DMA coherent buffer */
		if (copy_from_user(spi_dev.dmabuf_cpu,
					(void __user *)raw_buf.buf,
					raw_buf.length)) {
			res = -EFAULT;
			goto exit_usnd_raw;
		}

		perform_raw_dma(raw_buf.length >> 2, raw_buf.tholdcs);
		res = 0;
exit_usnd_raw:
		mutex_unlock(&spi_dev.mut);
		return res;
		break;

	case TEMPERATURE:
		if (access_ok(VERIFY_WRITE, (void *)arg, sizeof(uint32_t))) {

			temperature_get_data(&u_snd);

			reg = u_snd.temperature_value;
			res = copy_to_user((void __user *) arg,
					   (void *) &reg,
					   sizeof(uint32_t));
			if (res)
				return -EINVAL;
		}
		break;

	case BATTERY:
		if (!(u_snd.adc_status & ADC_VBAT_ENABLE))
			return -EINVAL;

		if (access_ok(VERIFY_WRITE, (void *)arg, sizeof(uint32_t))) {
			/*delos*/
			reg = aai_readl(AAI_ADC_BATTERY_VALUE) & 0x3FF;

			res = copy_to_user((void __user *) arg,
					   (void *) &reg,
					   sizeof(uint32_t));
			if (res)
				return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static unsigned int ultra_sound_poll(struct file *filep, poll_table *wait)
{
	unsigned long irqstate;
	poll_wait(filep, &u_snd.waitq, wait);
	spin_lock_irqsave(&u_snd.lock, irqstate);
	if (u_snd.irqcount <= 0) {
		pr_debug("%s %d  irqcount %d\n",
			 __func__, __LINE__, u_snd.irqcount);
		spin_unlock_irqrestore(&u_snd.lock, irqstate);
		return POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&u_snd.lock, irqstate);
	return 0;
}

static const struct file_operations ultra_snd_fops = {
	.open           = ultra_snd_open,
	.release        = ultra_snd_release,
	.mmap           = ultra_snd_mmap,
	.unlocked_ioctl = ultra_snd_ioctl,
	.poll           = ultra_sound_poll,
	.owner = THIS_MODULE
};

static struct miscdevice ultra_snd_miscdev = {
	.minor = 0,
	.name  = "ultra_snd",
	.fops  = &ultra_snd_fops,
};

static irqreturn_t u_snd_it(int irq, void *dev_id)
{
	uint32_t reg;
	uint16_t addr;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;

	spin_lock_irqsave(&u_snd.lock, flags);

	/* read interrupt status register*/
	reg = aai_readl(AAI_ITS);
	if (reg & AAI_ITS_ULTRA) {
		aai_writel(reg & AAI_ITS_ULTRA, AAI_DMA_INT_ACK);
		ret = IRQ_HANDLED;
		u_snd.irqcount--;

		wake_up_interruptible(&u_snd.waitq);
		if (u_snd.irqcount > 0) {
			aai_writel(1, AAI_ADC_ULTRA);
		}
	} else if (reg & AAI_ITS_MONITOR) {
		ret = IRQ_HANDLED;

		/* read temperature on ADC 2 */
		addr = AAI_ADC_DATA + 0x4 * u_snd.temperature_adc_num;
		u_snd.temperature_value = aai_readl(addr);

		/*
		 * Reading AAI_ADC_DATA acknowledges interrupt.
		 * Unlock synchronous with usnd->completion
		 */

		complete(&u_snd.completion);
	}

	spin_unlock_irqrestore(&u_snd.lock, flags);
	return ret;
}

static void spi_dma_callback(unsigned int chan, void *data, int status)
{
	if (status) {
		pr_err("SPI1 DMA transfer error.\n");
	}
	complete(&spi_dev.dma_completion);
}

/*
 * Second part of probe SPI.
 * We use the plateforme device aai.
 * So the spi ressources are not getted from bsp.
 * The ressource are hard written in the driver.
 */
static int __init probe_spi(void)
{
	int32_t res;
	uint32_t reg;

	/* spi clock */
	spi_dev.clock = clk_get(NULL, "spi1");
	if (IS_ERR(u_snd.clock)) {
		pr_err("Unable to get spi clock: errno %d\n",
		       (int32_t)spi_dev.clock);
		res = -EINVAL;
	}

	res = clk_enable(spi_dev.clock);
	if (res) {
		pr_err("Unable to enable spi clock: errno %d\n", res);
		goto spiclk;
	}

	/* spi register map */
	spi_dev.map = (int8_t *)ioremap(P6_SPI1_BAD, 0x1000);
	if (spi_dev.map == NULL) {
		res = -ENOMEM;
		goto nospimap;
	}

	/*
	 * Configure the spi interface
	 * bit, signification: value
	 * 0-3, TSETUPCS: 0
	 * 4-7, THOLDCS: 7 (hold MOSI during 8 SCLK periods after every byte)
	 *   8, spi stream mode: an interrupt is generated when FIFO is empty
	 *   9, spi master/slave mode: master
	 *  10, spi clock polarity: active high
	 *  11, spi clock phase bit: off clock edge
	 *	NOTE: to dephase signal inverse the bit 11.
	 *  12, spi tx mode: LSB
	 *  13, spi interrupt: disable
	 *  14, enable dma access: no
	 *  15, select dma accesses mode: write mode.
	 */
	reg = spi_readl(SPI_CTRL);
	reg = (0x0);
	reg |= (0x7 << 4);
	reg &= ~(1 << 8);
	reg |= (1 << 9);
	reg &= ~(1 << 10);
	reg &= ~(1 << 11);
	reg |= (1 << 12);
	reg &= ~(1 << 13);
	reg &= ~(1 << 14);
	reg &= ~(1 << 15);
	spi_writel(reg, SPI_CTRL);

	/* spi clock divider: (PCLK/ 2 * (107 + 1)) / 2 ~40Khz * (2 * 8) */
	spi_writel(107, SPI_SPEED);

	/* Allocate spi memory.
	 * The same buffer is used for USND_PULSES and USND_RAW ioctls. */
	spi_dev.dmabuf_cpu = dma_alloc_coherent(u_snd.device,
			PAGE_ALIGN(USND_MAX_RAW_BUFLEN), &spi_dev.dmabuf_bus, GFP_KERNEL);
	if (!spi_dev.dmabuf_cpu) {
		return -ENOMEM;
	}

	init_completion(&spi_dev.dma_completion);
	mutex_init(&spi_dev.mut);

	/* request a DMA channel */
	res = pl08x_dma_request(&spi_dev.dmachan, "ultra_snd", spi_dma_callback,
			NULL);
	if (res) {
		pr_err("Could not get a DMA channel: errno %d\n", res);
		res = -EBUSY;
		goto nodma;
	}

	return 0;
nodma:
	dma_free_coherent(u_snd.device, PAGE_ALIGN(USND_MAX_RAW_BUFLEN),
			spi_dev.dmabuf_cpu, spi_dev.dmabuf_bus);
nospimap:
	clk_disable(spi_dev.clock);
spiclk:
	clk_put(spi_dev.clock);
	return res;
}

static int ultra_snd_probe(struct platform_device *pdev)
{
	int32_t res;
	unsigned long flags;
	struct resource *resm;

	u_snd.lock =  __SPIN_LOCK_UNLOCKED(u_snd.lock);
	u_snd.device = &pdev->dev;

	/* HACK AAI ressource does not declare coherent dma mask */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	init_waitqueue_head(&u_snd.waitq);

	pr_info("ultra_snd probe#\n");

	init_completion(&u_snd.completion);
	init_MUTEX(&u_snd.adc_lock);

	spin_lock_init(&u_snd.lock);
	res = misc_register(&ultra_snd_miscdev);
	if (res < 0)
		goto nochdev;

	/* power on & clock the AAI IP */
	spin_lock_irqsave(&u_snd.lock, flags);
	u_snd.clock = clk_get(NULL, "aai");
	if (IS_ERR(u_snd.clock)) {
		pr_err("Unable to get clock: errno %d\n", (int32_t)u_snd.clock);
		spin_unlock_irqrestore(&u_snd.lock, flags);
		res = -ENODEV;
		goto err_gclk;
	}

	res = clk_enable(u_snd.clock);
	if (res) {
		pr_err("Unable to enable clock: errno %d\n", res);
		spin_unlock_irqrestore(&u_snd.lock, flags);
		goto err_eclk;
	}
	spin_unlock_irqrestore(&u_snd.lock, flags);

	resm = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resm) {
		pr_err("Unable to get ressource\n");
		res = -ENXIO;
		goto err_reg;
	}
	pr_debug("ressources get 0x%x 0x%x\n", resm->end, resm->start);
	u_snd.mem_start = (uint32_t)resm->start;
	u_snd.mem_end   = (uint32_t)resm->end;

	/*map the aai registers region*/
	u_snd.map = (int8_t *)ioremap(resm->start, resm->end - resm->start);
	if (u_snd.map == NULL) {
		res = -ENXIO;
		goto nomap;
	}

	/* alloc dma memory*/
	u_snd.adr = dma_alloc_coherent(u_snd.device, PAGE_ALIGN(SIZEMAX),
				       &u_snd.bus, GFP_USER);
	if (!u_snd.adr) {
		res = -ENOMEM;
		goto nodma;
	}

	pr_debug("%d dma_alloc_coherent Virt 0x%x, phy 0x%x\n",
		 __LINE__, (unsigned int)u_snd.adr, (unsigned int)u_snd.bus);

	/*Get interrupt*/
	resm =  platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!resm) {
		pr_err("Unable to get irq ressource\n");
		res = -ENXIO;
		goto noirq;
	}

	u_snd.numirq = resm->start;

	/* Set irq handler */
	res = request_irq(u_snd.numirq, &u_snd_it, IRQF_SHARED,
			  ULTRA_SND_NAME, u_snd.device);
	if (res)
		goto noirq;

	/* Call the SPI probe */
	res = probe_spi();
	if (res) {
		pr_err("SPI driver probe error\n");
		goto noirq;
	}

	return 0;
noirq:
	dma_free_coherent(u_snd.device,
			  PAGE_ALIGN(SIZEMAX), u_snd.adr, u_snd.bus);
nodma:
	iounmap(u_snd.map);
nomap:
	/* release_mem_region(resm->start, resm->end - resm->start); */
err_reg:
	clk_disable(u_snd.clock);
err_eclk:
	clk_put(u_snd.clock);
err_gclk:
	misc_deregister(&ultra_snd_miscdev);
nochdev:
	pr_err("probe failed, error %d\n", res);
	return res;
}

static int ultra_snd_cleanup(struct platform_device *pdev)
{
	pr_info("Removing driver...\n");

	if (u_snd.adr)
		dma_free_coherent(u_snd.device,
				  PAGE_ALIGN(SIZEMAX), u_snd.adr, u_snd.bus);

	free_irq(u_snd.numirq, u_snd.device);
	iounmap(u_snd.map);
	clk_disable(u_snd.clock);
	clk_put(u_snd.clock);
	iounmap(spi_dev.map);
	clk_disable(spi_dev.clock);
	clk_put(spi_dev.clock);
	dma_free_coherent(u_snd.device, PAGE_ALIGN(USND_MAX_RAW_BUFLEN),
			spi_dev.dmabuf_cpu, spi_dev.dmabuf_bus);
	pl08x_dma_free(spi_dev.dmachan);
	misc_deregister(&ultra_snd_miscdev);

	return 0;
}

/* We use the aai ressource s*/
static struct platform_driver usnd_driver = {
	.remove = &ultra_snd_cleanup,
	.probe  = &ultra_snd_probe,
	.driver = {
		.name  = "ultra_snd",
		.owner = THIS_MODULE,
	},
};

static int usnd_init(void)
{
	return platform_driver_register(&usnd_driver);
}
module_init(usnd_init);

static void usnd_exit(void)
{
	platform_driver_unregister(&usnd_driver);
}
module_exit(usnd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Parrot");
