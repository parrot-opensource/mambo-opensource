#ifndef INCONCE_AAI_HW_H
#define INCONCE_AAI_HW_H

/*
 * linux/drivers/io/aai_hw.h
 *	Copyright (c) Parrot SA
 *
 *	Written by Gregoire ETIENNE <gregoire.etienne@parrot.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    Parrot Advanced Audio Interface Driver
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/ioport.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <mach/regs-aai-p5p.h>

//! Mode stream definition for SPK, BACK, MIC12, MIC34 (P5+ only)
typedef enum {
    AAI_STREAM_MODE_8KHZ_16W     = 0x00, //!<  8kHz rate with 16 words FIFO
    AAI_STREAM_MODE_16KHZ_32W    = 0x01, //!< 16kHz rate with 32 words FIFO
    AAI_STREAM_MODE_44_48KHZ_16W = 0x02, //!< 44,1/48kHz rate with 16 words FIFO
    AAI_STREAM_MODE_44_48KHZ_32W = 0x03, //!< 44,1/48kHz rate with 32 words FIFO
} aai_stream_mode_t;

#define AAI_FE_SPK_FREQ(_fe)                             \
    (((_fe) & AAI_FE_SPK_MODE) >> AAI_FE_SPK_MODE_SHIFT)

#define AAI_FE_BACK_FREQ(_fe)                                       \
    (((_fe) & AAI_FE_FEEDBACK_MODE) >> AAI_FE_FEEDBACK_MODE_SHIFT)

#define AAI_FE_MIC12_FREQ(_fe)                                  \
    (((_fe) & AAI_FE_MIC12_MODE) >> AAI_FE_MIC12_MODE_SHIFT)

#define AAI_FE_MIC34_FREQ(_fe)                                  \
    (((_fe) & AAI_FE_MIC34_MODE) >> AAI_FE_MIC34_MODE_SHIFT)


//! Sample size
typedef enum {
    AAI_MONO   = 1,
    AAI_STEREO = 2,
    AAI_QUAD   = 4,
} aai_sample_size_t;

//! Channel fifo mode
typedef enum {
    AAI_MONO_8,
    AAI_MONO_16,
    AAI_STEREO_8,
    AAI_STEREO_16,
    AAI_QUAD_8,
    AAI_QUAD_16,
} aai_fifo_mode_t;



/*---------------------------------------------------------------------------*/
/* Exported prototypes of aai_hw.c                                           */
/*---------------------------------------------------------------------------*/
extern aai_device_t *aai_hw_get_pchans( void );
extern int  aai_hw_init           (struct card_data_t *aai);
extern void aai_hw_status         (struct card_data_t *aai);
extern int  aai_hw_rate_rule      (void);
extern int  aai_hw_start_channel  (aai_device_t *chan);
extern int  aai_hw_stop_channel   (aai_device_t *chan);
extern int  aai_hw_set_channelrate(aai_device_t *chan, int rate);
extern int  aai_hw_setvolume		 (aai_device_t *chan);
extern int  aai_hw_getvolume		 (aai_device_t *chan);

extern aai_device_t aai_channels[];

/*---------------------------------------------------------------------------*/
/* Exported prototypes of aai_irq.c                                          */
/*---------------------------------------------------------------------------*/
extern irqreturn_t aai_irq(int irq, void *dev);



static inline void aai_writereg(struct card_data_t *aai, unsigned int val, int offset)
{
	writel(val, aai->iobase+offset);
}

static inline unsigned int aai_readreg(struct card_data_t *aai, int offset)
{
	return readl(aai->iobase+offset);
}


static inline void aai_set_nbit(struct card_data_t *aai, unsigned int offset,
										 unsigned int mask, unsigned int shift, unsigned int val)
{
	unsigned int reg = aai_readreg(aai,offset);
	//_printk("setting bits 0x%08x to %d@0x%03x (0x%08x)\n", mask, val, offset, reg);
	reg &= ~(mask<<shift);	/* set all concerned bits to 0 */
   reg |= (val<<shift);    /* set needed bits to 1        */
	aai_writereg(aai,reg, offset);
	//aai_printk("register written(0x%08x)\n", reg);
}

static inline int aai_get_nbit(struct card_data_t *aai, unsigned int offset,
										 unsigned int mask, unsigned int shift)
{
	unsigned int reg = aai_readreg(aai,offset);
	//_printk("setting bits 0x%08x to %d@0x%03x (0x%08x)\n", mask, val, offset, reg);
   reg = reg >> shift;
   return (reg&mask);/* set needed bits to 1        */
}


static inline void aai_setbit(struct card_data_t *aai, unsigned int offset,
                              unsigned int mask, unsigned int val)
{
	unsigned int reg  =  aai_readreg(aai,offset );
	//aai_printk("setting bit 0x%08x to %d@0x%03x (0x%08x)\n", mask, val, offset, reg);
	if( !!(reg&mask) != (!!val) ){ /* if the bit has to be modified */
		reg &= ~(mask);      /* reset concerned bit to 0 */
		if(val) reg |= mask; /* set it if needed */
		aai_writereg(aai,reg, offset );
		//aai_printk("register written(0x%08x)\n", reg);
	}
}

/**
 * get bit value
 */
static inline unsigned int aai_getbit(struct card_data_t *aai, unsigned int offset,
                              unsigned int mask)
{
	unsigned int reg;
	reg  =  aai_readreg(aai,offset );
	return !!(reg&mask);
}


#endif






