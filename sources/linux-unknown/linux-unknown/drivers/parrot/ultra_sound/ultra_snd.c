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
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uaccess.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include <mach/regs-aai-p6.h>
#include <mach/ultra_snd.h>

#include "timer.h"
#include "ultra_snd.h"

enum monitor_mode {
	JPSUMO_MONITOR_MOTORS,
	JPSUMO_MONITOR_VBAT,
	DELOS_MONITOR_TEMPERATURE
};

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
	uint64_t			cycles;
	struct adc_data_t		adc_data;
	uint16_t			vbat_value;
	uint8_t                         temperature_adc_num;
	uint16_t			temperature_value;
	int				gpio_mux_vbat_jump;
	int				gpio_mux_wheels;

	struct completion		completion;
	enum monitor_mode	        monitor_mode;
	struct semaphore		adc_lock;
};

/*
 * define usnd_device.adc_status bit field
 */
#define ADC_JS_ENABLE               0x01
#define ADC_JS_50_RUN               0x02	/* ADC 0 ->vbat, jump;
						   ADC 2 ->wheel */
#define ADC_JS_8K_RUN               0x04	/* oversampled 8K mode */
#define ADC_VBAT_ENABLE             0x08	/* Only ADC0 vbat enable */
#define ADC_VBAT_USE_BAT_REG        0x10	/* Real battery mode */

#define MAX_AAI_ADC_INDEX 8
/*
 * mux for ADC 0 can be set on 2 positions:
 * 0 to read IJUMP
 * 1 to read VBAT
 */
enum {
	ADC_JS_MUX_VBAT,
	ADC_JS_MUX_IJUMP
};

/*
 * spi_device is the used struct to store the SPI informations
 * @struct map: the spi mapped register
 * @struct len: the len of the spi pattern to send
 * @struct adr: the allocated space for the spi pattern to be send
 * */

struct spi_device {
	struct clk	*clock;
	uint8_t		*map;
	uint8_t		len;
	uint32_t	*adr;
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

static struct mem mem = {
	.pin     = 0,
	.pout    = 0,
	.usr_ptr = 0,
	.free    = SIZEMAX,
};

static struct timed_buf tb;

static int mem_put(struct mem *V, int size)
{
	int old;
	if (size < V->free) {
		V->pin = (V->pin + size) % SIZEMAX;
		V->free -= size;
		return size;
	} else {
		V->pin = (V->pin + V->free) % SIZEMAX;
		old = V->free;
		V->free = 0;
		return old;
	};
}

static int mem_get(struct mem *V, int size)
{
	int old;
	if (size < SIZEMAX - V->free) {
		V->pout = (V->pout + size) % SIZEMAX;
		V->free += size;
		return size;
	} else {
		V->pout = (V->pout + SIZEMAX - V->free) % SIZEMAX;
		old = SIZEMAX - V->free;
		V->free = SIZEMAX;
		return old;
	};
}

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

static int jpsumo_adc_get_data(struct usnd_device *usnd)
{
	int ret = 0;

	ret = down_interruptible(&usnd->adc_lock);
	if (ret < 0)
		return ret;

	/* reset gpio mux to read ijump on ADC 0 */
	if (!gpio_is_valid(usnd->gpio_mux_vbat_jump)) {
		ret = -EINVAL;
		goto exit;
	}

	gpio_direction_output(u_snd.gpio_mux_vbat_jump, ADC_JS_MUX_IJUMP);

	usnd->monitor_mode = JPSUMO_MONITOR_MOTORS;

	/* trigger ADC conversion */
	aai_writel(1, AAI_ADC_MONITOR);

	/* wait for transfer to be complete */
	wait_for_completion(&usnd->completion);
exit:
	up(&usnd->adc_lock);
	return ret;
}

static int jpsumo_vbat_get_data(struct usnd_device *usnd)
{
	int ret = 0;

	ret = down_interruptible(&usnd->adc_lock);
	if (ret < 0)
		return ret;

	/* Then set gpio mux to read vbat on ADC 0 */
	if (!gpio_is_valid(usnd->gpio_mux_vbat_jump)) {
		ret = -EINVAL;
		goto exit;
	}

	gpio_direction_output(u_snd.gpio_mux_vbat_jump, ADC_JS_MUX_VBAT);

	usnd->monitor_mode = JPSUMO_MONITOR_VBAT;

	/* trigger ADC conversion */
	aai_writel(1, AAI_ADC_MONITOR);

	/* wait for transfer to be complete */
	wait_for_completion(&usnd->completion);
exit:
	up(&usnd->adc_lock);
	return ret;
}

static int temperature_get_data(struct usnd_device *usnd)
{
	int ret = 0;

	ret = down_interruptible(&usnd->adc_lock);
	if (ret < 0)
		return ret;

	usnd->monitor_mode = DELOS_MONITOR_TEMPERATURE;

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

	if (u_snd.adc_status & ADC_JS_ENABLE) {
		/* JS enabled*/
		reg &= ~(AAI_ADC_MODES_BATTERY_ALARM);
		u_snd.adc_status &= ~(ADC_VBAT_USE_BAT_REG);
		u_snd.vbat_value = 0xFFFF;
	} else {
		/* configure battery input*/
		reg |= AAI_ADC_MODES_BATTERY_ALARM;
		/* mandatory to check battery*/
		reg |= AAI_ADC_MODES_MONITORING;
		reg &= ~(AAI_ADC_MODES_BATTERY_IN);
		reg |= AAI_ADC_MODES_BATTERY_IN & (bat_adc_num << 11);
		u_snd.adc_status |= ADC_VBAT_USE_BAT_REG;
	}

	u_snd.adc_status |= ADC_VBAT_ENABLE;
	aai_writel(reg, AAI_ADC_MODES);
	wmb();

	/* already done by JS_INIT*/
	if (!(u_snd.adc_status & ADC_JS_ENABLE)) {
		reg = aai_readl(AAI_CFG);
		reg |= AAI_CFG_MUSIC_ICH0;
		reg |= AAI_CFG_RUN_MULT;
		reg &= ~(AAI_CFG_COMPACT);
		aai_writel(reg, AAI_CFG);
		wmb();
	}

	spin_unlock_irqrestore(&u_snd.lock, sp_lock);

	pr_debug("AAI_CFG 0x320: 0x%x\n", aai_readl(AAI_CFG));
	pr_debug("AAI_ADC_MODES 0x39c: 0x%x\n", aai_readl(AAI_ADC_MODES));
}

static void ultra_sound_config(void)
{
	int reg;
	unsigned long sp_lock;

	spin_lock_irqsave(&u_snd.lock, sp_lock);

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

	spin_unlock_irqrestore(&u_snd.lock, sp_lock);

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

static void jpsumo_config(void)
{
	int reg;
	unsigned long sp_lock;

	spin_lock_irqsave(&u_snd.lock, sp_lock);

	/* set AAI configuration */
	reg = aai_readl(AAI_ADC_MODES);

	/* vbat enabled */
	if (u_snd.adc_status & ADC_VBAT_ENABLE) {
		/* cancel battery mode */
		reg &= ~(AAI_ADC_MODES_BATTERY_ALARM);
		u_snd.adc_status &= ~(ADC_VBAT_USE_BAT_REG);
	}

	/*ultra sound is connected in adc 1*/
	reg &= ~(AAI_ADC_MODES_ULTRA_IN);
	reg |= AAI_ADC_MODES_ULTRA_IN & (0x01 << 8);
	reg |= AAI_ADC_MODES_ULTRA_SOUNDS;
	reg |= AAI_ADC_MODES_MONITORING;

	/* mandatory to get wheels */
	aai_writel(reg, AAI_ADC_MODES);
	wmb();

	if (!(u_snd.adc_status & ADC_VBAT_ENABLE)) {
		/* already done by BATTERY_INIT */
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
		wmb();
	}

	reg = aai_readl(AAI_DMACTL);
	reg |= AAI_DMA_CTRL_ULTRA;
	aai_writel(reg, AAI_DMACTL);
	wmb();

	u_snd.irqcount = 1;	/* 1 chunk of 8kb */
	spin_unlock_irqrestore(&u_snd.lock, sp_lock);

	pr_debug("Line %d: AAI_ADC_MODES 0x39c: 0x%x\n",
		 __LINE__, aai_readl(AAI_ADC_MODES));
	pr_debug("Line %d: AAI_CFG 0x320: 0x%x\n",
		 __LINE__, aai_readl(AAI_CFG));
	pr_debug("Line %d: AAI_DMACTL 0x37C: 0x%x\n",
		 __LINE__, aai_readl(AAI_DMACTL));
}

static long ultra_snd_ioctl(struct file *filp,
			    unsigned int cmd,
			    unsigned long arg)
{
	uint32_t reg, res = 0, dma, spi_sz = 0;
	int32_t cpt;
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
		res = copy_from_user((void *)&(u_snd.size),
				     (void __user *)arg,
				     sizeof(uint32_t));

		if ((res != 0) || (u_snd.size > SIZEMAX))
			return -EINVAL;

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

	case USND_SPI_LEN:
		if (access_ok(VERIFY_READ, (void *)arg, sizeof(uint8_t))) {
			res = copy_from_user((void *) &(spi_dev.len),
					     (void __user *)arg,
					     sizeof(uint8_t));
			if (!res)
				return 0;
		}
		return -EINVAL;
		break;

	case USND_SPI_DAT:
		/* SPI_lEN is a bitfiled
		 * The [0-4] bits are the number of phased pulses
		 * The [5-7] bits are the number of dephased pulses
		 * The sum of these two bitfiled cannot exceed SPIMAX
		 * No need to check
		 * */
		spi_sz = spi_dev.len & 0xFF;
		if (access_ok(VERIFY_READ, (void *)arg,
			      spi_sz * sizeof(uint32_t))) {
			res = copy_from_user((void *)spi_dev.adr,
					     (void __user *)arg,
					     spi_sz * sizeof(uint32_t));

			*(spi_dev.adr + spi_sz - 1) |= 0x100;

			if (!res) {
				spin_lock_irqsave(&u_snd.lock, sp_lock);
				/*Enable ADC*/
				aai_writel(1, AAI_ADC_ULTRA);
				/*Enable SPI*/
				for (cpt = 0; cpt < spi_sz; cpt++)
					spi_writel(*(spi_dev.adr + cpt),
						   SPI_DATA);

				spin_unlock_irqrestore(&u_snd.lock, sp_lock);
				return 0;
			}
		}
		return -EINVAL;
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
			if (!(u_snd.adc_status & ADC_VBAT_USE_BAT_REG)) {
				/*JS*/
				res = jpsumo_vbat_get_data(&u_snd);
				if (res < 0)
					return -EINVAL;

				reg = u_snd.vbat_value;
			} else {
				/*delos*/
				reg = aai_readl(AAI_ADC_BATTERY_VALUE)
					& 0x3FF;
			}

			res = copy_to_user((void __user *) arg,
					   (void *) &reg,
					   sizeof(uint32_t));
			if (res)
				return -EINVAL;
		}
		break;

	case JS_GETBUFFER:
		/*
		 * pin is on next space to write,
		 * pin-1buff is being written,
		 * pin-2buff is the last readable buffer,
		 * that's why it is (2*(1 << MAXDMA))...
		 */
		spin_lock_irqsave(&u_snd.lock, sp_lock);
		if ((mem.pin - mem.usr_ptr + SIZEMAX) % SIZEMAX >
		    (2 * (1 << MAXDMA))) {
			tb.offset = mem.usr_ptr;
			mem.usr_ptr = (mem.usr_ptr + (1 << MAXDMA)) % SIZEMAX;

			if (access_ok(VERIFY_WRITE, (void *)arg,
				      sizeof(struct timed_buf))) {
				spin_unlock_irqrestore(&u_snd.lock, sp_lock);
				res = copy_to_user((void __user *)arg,
						   (void *)&tb, sizeof(tb));
				return res;
			}
			res = -EINVAL;
		} else {
			res = -EAGAIN;
		}

		spin_unlock_irqrestore(&u_snd.lock, sp_lock);
		return res;
		break;

	case JS_RELEASEBUFFER:
		if (access_ok(VERIFY_READ, (void *)arg, sizeof(uint32_t))) {
			res = copy_from_user((void *)&(reg),
					     (void __user *)arg,
					     sizeof(uint32_t));
			if (res)
				return -EINVAL;

			/*
			 * The next IF describes the following conditions:
			 * Buffer must be released in order
			 * Cannot release more buffer than requested
			 */
			spin_lock_irqsave(&u_snd.lock, sp_lock);
			if ((reg != mem.pout) ||
			    ((1 << MAXDMA) > ((SIZEMAX+mem.usr_ptr - reg) %
					      SIZEMAX)))
				res = -EINVAL;

			res = mem_get(&mem, 1 << MAXDMA);
			if (res !=  1 << MAXDMA)
				res = -EINVAL;
			else
				res = 0;

			spin_unlock_irqrestore(&u_snd.lock, sp_lock);
		}
		return res;
		break;

	case JS_INIT:
		u_snd.size = 1 << 18;
		memset(u_snd.adr, 0, u_snd.size);

		jpsumo_config();
		mem.pin = 0; mem.usr_ptr = 0; mem.pout = 0; mem.free = SIZEMAX;

		/* gpio mux init for adc 0 to read ijump*/
		if (gpio_is_valid(u_snd.gpio_mux_vbat_jump))
			gpio_direction_output(u_snd.gpio_mux_vbat_jump,
					      ADC_JS_MUX_IJUMP);

		spin_lock_irqsave(&u_snd.lock, sp_lock);
		aai_writel(u_snd.bus, AAI_DMASA_ULTRA);
		/* Next chunk of memory 8Kb*/
		aai_writel(u_snd.bus + (1 << MAXDMA), AAI_DMAFA_ULTRA);
		aai_writel(0x7, AAI_ULTRA_DMA_COUNT);
		reg = aai_readl(AAI_ULTRA_DMA_COUNT);
		res = mem_put(&mem, 1 << MAXDMA);
		if (res == (1<<MAXDMA)) {
			/*mem_put increase pin, so the @ is pin - size*/
			aai_writel(u_snd.bus + mem.pin - (1<<MAXDMA),\
				   AAI_DMASA_ULTRA);
			aai_writel(u_snd.bus + mem.pin, AAI_DMAFA_ULTRA);
		}
		u_snd.adc_status |= ADC_JS_ENABLE;
		spin_unlock_irqrestore(&u_snd.lock, sp_lock);

		if (res == (1 << MAXDMA))
			pr_debug("JS_INIT done\n");
		else
			return -EAGAIN;
		break;

	case JS_START_8K:
		spin_lock_irqsave(&u_snd.lock, sp_lock);
		if (u_snd.adc_status & ADC_JS_ENABLE) {
			/* start 8kHz capture */
			aai_writel(1, AAI_ADC_ULTRA);
			u_snd.adc_status |= ADC_JS_8K_RUN;
		}
		spin_unlock_irqrestore(&u_snd.lock, sp_lock);
		break;

	case JS_START:
		if (u_snd.adc_status & ADC_JS_ENABLE) {
			if (gpio_is_valid(u_snd.gpio_mux_vbat_jump))
				gpio_direction_output(u_snd.gpio_mux_vbat_jump,
						      ADC_JS_MUX_IJUMP);

			if (!(u_snd.adc_status & ADC_JS_50_RUN))
				u_snd.adc_status |= ADC_JS_50_RUN;
		}
		break;

	case JS_STOP:
		spin_lock_irqsave(&u_snd.lock, sp_lock);
		mem.pin     = 0;
		mem.pout    = 0;
		mem.usr_ptr = 0;

		/* disable JS */
		u_snd.adc_status &= ~(ADC_JS_ENABLE | ADC_JS_8K_RUN);
		u_snd.adc_status &= ~ADC_JS_50_RUN;
		spin_unlock_irqrestore(&u_snd.lock, sp_lock);
		break;

	case JS_GET_DATA:
		/* Mandatory JS_START HAVE to be performed before */
		if (!(u_snd.adc_status & ADC_JS_50_RUN))
			return -EINVAL;

		res = jpsumo_adc_get_data(&u_snd);
		if (res < 0)
			return -EINVAL;

		if (access_ok(VERIFY_WRITE, (void *)arg,
			      sizeof(struct adc_data_t))) {
			res = copy_to_user((void __user *) arg,
					   (void *)&u_snd.adc_data,
					   sizeof(struct adc_data_t));
			if (res)
				return -EINVAL;
		}
		break;

	case JS_MUX_ADC_1_2:
		/*
		 * arg = 0 for LWh on ADC1 and RWh on ADC2
		 * arg = 1 for RWh on ADC1 and Wh on ADC2
		 */
		if (access_ok(VERIFIY_READ, (void *)arg, sizeof(uint32_t))) {
			res = copy_from_user((void *)&(reg),
					     (void __user *)arg,
					     sizeof(uint32_t));
			if (res)
				return -EINVAL;
		}
		break;
	}
	return 0;
}

static unsigned int ultra_sound_poll(struct file *filep, poll_table *wait)
{
	poll_wait(filep, &u_snd.waitq, wait);
	if (u_snd.irqcount <= 0) {
		if (u_snd.adc_status & ADC_JS_ENABLE)
			u_snd.irqcount = 1;
		pr_debug("%s %d  irqcount %d\n",
			 __func__, __LINE__, u_snd.irqcount);
		return POLLIN | POLLRDNORM;
	}
	return 0;
}

static const struct file_operations ultra_snd_fops = {
	.open           = ultra_snd_open,
	.release        = ultra_snd_release,
	.mmap           = ultra_snd_mmap,
	.unlocked_ioctl = ultra_snd_ioctl,
	.poll           = ultra_sound_poll,
};

static struct miscdevice ultra_snd_miscdev = {
	.minor = 0,
	.name  = "ultra_snd",
	.fops  = &ultra_snd_fops,
};

static irqreturn_t u_snd_it(int irq, void *dev_id)
{
	uint32_t res, reg;
	uint16_t addr;
	unsigned long flags;
	irqreturn_t ret = IRQ_HANDLED;

	spin_lock_irqsave(&u_snd.lock, flags);

	/* read interrupt status register*/
	reg = aai_readl(AAI_ITS);
	if (reg & AAI_ITS_ULTRA) {
		aai_writel(reg & AAI_ITS_ULTRA, AAI_DMA_INT_ACK);
		u_snd.irqcount--;

		wake_up_interruptible(&u_snd.waitq);
		if ((u_snd.irqcount > 0) ||
		    (u_snd.adc_status & ADC_JS_ENABLE)) {
			aai_writel(1, AAI_ADC_ULTRA);
			ret = IRQ_HANDLED;
		} else {
			ret = IRQ_NONE;
		}

		tb.cycles = parrot6_get_cycles();

		if (u_snd.adc_status & ADC_JS_8K_RUN) {
			res = mem_put(&mem, 1 << MAXDMA);
			if (res == (1 << MAXDMA))
				aai_writel(u_snd.bus + mem.pin,
					   AAI_DMAFA_ULTRA);
			else
				u_snd.adc_status &= ~(ADC_JS_8K_RUN);
		}
	} else if (reg & AAI_ITS_MONITOR) {
		switch (u_snd.monitor_mode) {
		case JPSUMO_MONITOR_MOTORS:
			/* read wheel 50Hz on ADC 2*/
			u_snd.adc_data.wheel50hz =
				aai_readl(AAI_ADC_DATA + 0x8);

			/* read ijump on ADC 0*/
			u_snd.adc_data.ijump = aai_readl(AAI_ADC_DATA);
			break;
		case JPSUMO_MONITOR_VBAT:
			/* read vbat on ADC 0 */
			u_snd.vbat_value = aai_readl(AAI_ADC_DATA);
			break;
		case DELOS_MONITOR_TEMPERATURE:
			/* read temperature on ADC 2 */
			addr = AAI_ADC_DATA + 0x4 * u_snd.temperature_adc_num;
			u_snd.temperature_value = aai_readl(addr);
			break;
		default:
			break;
		}

		/*
		 * Reading AAI_ADC_DATA acknowledges interrupt.
		 * Unlock synchronous with usnd->completion
		 */

		complete(&u_snd.completion);
	}

	spin_unlock_irqrestore(&u_snd.lock, flags);
	return ret;
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
	 * 4-7, THOLDCS: 0
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

	/* allocate spi memory */
	spi_dev.adr = kzalloc(SPIMAX, GFP_KERNEL);
	if (!PTR_ERR(spi_dev.adr))
		return -ENOMEM;

	return 0;
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
	struct parrot_ultra_snd_platform_data *pdata = pdev->dev.platform_data;

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

	if (pdata) {
		u_snd.gpio_mux_vbat_jump = pdata->gpio_mux_vbat_jump;
		u_snd.gpio_mux_wheels    = pdata->gpio_mux_wheels;
		pr_debug(" GPIO MUX: VBAT_JUMP %d, WHEELS %d\n",
			 u_snd.gpio_mux_vbat_jump, u_snd.gpio_mux_wheels);
	}

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
	pr_info("Removing driver...");

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
	misc_deregister(&ultra_snd_miscdev);
	kfree(spi_dev.adr);

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
