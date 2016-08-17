/*
 * linux/drivers/io/aai_ops.c
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
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "aai.h"
#include "aai_hw.h"

static struct snd_pcm_hardware aai_pcm_hw_mult = {
	.info			      = SNDRV_PCM_INFO_MMAP            |
				           SNDRV_PCM_INFO_MMAP_VALID      |
				           SNDRV_PCM_INFO_INTERLEAVED     |
				           SNDRV_PCM_INFO_BLOCK_TRANSFER  |
				           SNDRV_PCM_INFO_RESUME,

	.formats		      = SNDRV_PCM_FMTBIT_S16_LE |
				           SNDRV_PCM_FMTBIT_U16_LE,

	.rates			   = SNDRV_PCM_RATE_8000  |
                       SNDRV_PCM_RATE_16000 |
                       SNDRV_PCM_RATE_44100 |
                       SNDRV_PCM_RATE_48000,
	.rate_max		   = 48000,
	.rate_min		   = 8000,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= 64*1024,
	.period_bytes_min	= 8*2,
	.period_bytes_max	= PAGE_SIZE*2*64,
	.periods_min		= 4,
	.periods_max		= PAGE_SIZE / 16,
};

static struct snd_pcm_hardware aai_pcm_hw_pcm = {
	.info			      = SNDRV_PCM_INFO_MMAP            |
				           SNDRV_PCM_INFO_MMAP_VALID      |
				           SNDRV_PCM_INFO_INTERLEAVED     |
				           SNDRV_PCM_INFO_BLOCK_TRANSFER  |
				           SNDRV_PCM_INFO_RESUME,

	.formats		      = SNDRV_PCM_FMTBIT_S16_LE |
				           SNDRV_PCM_FMTBIT_U16_LE,

	.rates			   = SNDRV_PCM_RATE_8000  |
                       SNDRV_PCM_RATE_16000,
	.rate_max		   = 16000,
	.rate_min		   = 8000,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= 64*1024,
	.period_bytes_min	= 8*2,
	.period_bytes_max	= PAGE_SIZE*2*64,
	.periods_min		= 4,
	.periods_max		= PAGE_SIZE / 16,
};



void devdma_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *buf = runtime->dma_buffer_p;

	if (runtime->dma_area == NULL)
		return;

	if (buf != &substream->dma_buffer) {
		dma_free_coherent(/*substream->pcm->card->dev*/NULL, buf->bytes, buf->area, buf->addr);
		kfree(runtime->dma_buffer_p);
	}

	snd_pcm_set_runtime_buffer(substream, NULL);
}

int devdma_hw_alloc(struct snd_pcm_substream *substream, size_t size)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *buf = runtime->dma_buffer_p;

	int ret = 0;

	if (buf) {
		if (buf->bytes >= size)
			goto out;
		devdma_hw_free(substream);
	}

	if (substream->dma_buffer.area != NULL && substream->dma_buffer.bytes >= size) {
		buf = &substream->dma_buffer;
	} else {
		buf = kmalloc(sizeof(struct snd_dma_buffer), GFP_KERNEL);
		if (!buf)
			goto nomem;

		buf->dev.type = SNDRV_DMA_TYPE_DEV;
		buf->dev.dev  = /*substream->pcm->card->dev*/NULL;
		buf->area     = dma_alloc_coherent(/*substream->pcm->card->dev*/NULL, size, &buf->addr, GFP_KERNEL);
		buf->bytes    = size;
		buf->private_data = NULL;

		if (!buf->area)
			goto free;
	}
	snd_pcm_set_runtime_buffer(substream, buf);
	ret = 1;
 out:
	runtime->dma_bytes = size;
	return ret;

 free:
	kfree(buf);
 nomem:
	return -ENOMEM;
}

int devdma_mmap(struct device *dev, struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	return dma_mmap_coherent(/*substream->pcm->card->dev*/NULL, vma, runtime->dma_area, runtime->dma_addr, runtime->dma_bytes);
}

static int aai_ops_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	return devdma_mmap(NULL, substream, vma);
}



/**
 * snd_soc_set_runtime_hwparams - set the runtime hardware parameters
 * @substream: the pcm substream
 * @hw: the hardware parameters
 *
 * Sets the substream runtime hardware parameters.
 */
static inline int copy_runtime_hwparams(struct snd_pcm_runtime *runtime,
	                                        const struct snd_pcm_hardware *hw)
{
	runtime->hw.info           = hw->info;
	runtime->hw.formats        = hw->formats;
	runtime->hw.channels_min   = hw->channels_min;
	runtime->hw.channels_max   = hw->channels_max;
	runtime->hw.rates          = hw->rates;
	runtime->hw.rate_min       = hw->rate_min;
	runtime->hw.rate_max       = hw->rate_max;
	runtime->hw.buffer_bytes_max = hw->buffer_bytes_max;
	runtime->hw.period_bytes_min = hw->period_bytes_min;
	runtime->hw.period_bytes_max = hw->period_bytes_max;
	runtime->hw.periods_min    = hw->periods_min;
	runtime->hw.periods_max    = hw->periods_max;
	runtime->hw.fifo_size      = hw->fifo_size;
	return 0;
}





/*-----------------------------------------------------------------------------
 *
 * Common operators
 *
 *---------------------------------------------------------------------------*/


/** Open operator
 *
 */
static int aai_ops_open(struct snd_pcm_substream *substream)
{
   struct snd_pcm_runtime *runtime = substream->runtime;
   aai_device_t * chan = aai_pchan_from_substream(substream);
	int ret=0;
   struct card_data_t *aai = chan->pdrvdata;


   /* more hardware-initialization will be done here */
   aai_dbg(aai->dev,"%s(%d) - open\n", chan->name, chan->ipcm);

	//Copy params to rumtime struct
   if(chan->ipcm < AAI_CHANNEL_BT_OUT)
      copy_runtime_hwparams(runtime, &aai_pcm_hw_mult);
   else
      copy_runtime_hwparams(runtime, &aai_pcm_hw_pcm);

	//runtime->private_data = ; num chan ?


	/**
	 * Add rule describing hardware rate dependency
	 * on the configuration.
	 */
/*	ret = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
				                 hw_rule_rate_by_channels, NULL,
				                 SNDRV_PCM_HW_PARAM_RATE, -1);*/


	if (ret < 0) {
      aai_err(aai->dev,"%s adding rule failed\n", __FUNCTION__);
   }

	return ret;
}

/** Close operator
 * Reset channel to initial state
 */
static int aai_ops_close(struct snd_pcm_substream *substream)
{
   int err = 0;
   aai_device_t * chan = aai_pchan_from_substream(substream);
   struct card_data_t *aai = chan->pdrvdata;

   err = aai_hw_set_channelrate(chan, 8000);
   aai_dbg(aai->dev,"%s(%d) - close\n", chan->name, chan->ipcm);
   return err;
}

/** Params operator
 * This is called when the hardware parameter (hw_params) is set up by
 * the application, that is, once when the buffer size, the period size,
 * the format, etc. are defined for the pcm substream.
 * Many hardware setups should be done in this callback, including
 * the allocation of buffers.
 * Parameters to be initialized are retrieved by params_xxx() macros
 */
static int aai_ops_params(struct snd_pcm_substream *substream,
				              struct snd_pcm_hw_params *hw_params)
{
   int nbchans;
   aai_device_t * chan = aai_pchan_from_substream(substream);
   struct card_data_t *aai = chan->pdrvdata;
   struct snd_pcm_runtime *runtime = substream->runtime;

   int err = 0;

   chan->fifo[0].bufsize = params_buffer_bytes(hw_params);

   /*------------------------------------------------
    * Audio stream buffer allocation
    *----------------------------------------------*/
	err = devdma_hw_alloc(substream, chan->fifo[0].bufsize+1);
   if(err != 1){
      aai_err(aai->dev,"%s(%d) - can't allocate buffer of %d bytes\n", chan->name, chan->ipcm, chan->fifo[0].bufsize);
      goto out;
   }else{
      /*aai_dbg("%s(%d) - allocated a %d bytes buffer @%p \n",
               chan->name, chan->ipcm , chan->bufsize, (void *)substream->runtime->dma_area);*/
   }

   /*------------------------------------------------
    * Init buffer pointers
    *----------------------------------------------*/
   runtime->dma_bytes-=1; /* XXX patch due to allocation of +1 byte (see later)*/

   chan->fifo[0].pstart	  = (void *)runtime->dma_area;
   chan->fifo[0].pend	  = chan->fifo[0].pstart + chan->fifo[0].bufsize;
   chan->fifo[0].pcurrent = chan->fifo[0].pstart;

   chan->rate   = params_rate(hw_params);
   chan->format = params_format(hw_params);
   nbchans      = params_channels(hw_params);

   /*------------------------------------------------
    * Check mono/stereo configuration
    *----------------------------------------------*/
   if( chan->format != 2 ) {
      aai_err(aai->dev,"%s(%d) - invalid data format\n",chan->name, chan->ipcm);
      err = -EINVAL;
      goto out;
   }

   /*------------------------------------------------
    * Check mono/stereo configuration
    *----------------------------------------------*/
   if(  (((chan->rate==44100)||(chan->rate==48000)) && (nbchans!=2))
      ||(((chan->rate== 8000)||(chan->rate==16000)) && (nbchans!=1)) ) {
      aai_err(aai->dev,"%s(%d) - can't set %d Hz with %d chans\n",chan->name, chan->ipcm, chan->rate,nbchans);
      err = -EINVAL;
      goto out;
   }


out:
	return err;
}



/** Prepare operator.
 *
 * This callback is called when the pcm is "prepared". You can set
 * the format type, sample rate, etc. here. The difference from hw_params
 * is that the prepare callback will be called each time snd_pcm_prepare()
 * is called, i.e. when recovering after underruns, etc.
 */
static int aai_ops_prepare(struct snd_pcm_substream *substream)
{
   int err = 0;
   struct snd_pcm_runtime *runtime = substream->runtime;
   aai_device_t * chan = aai_pchan_from_substream(substream);
   struct card_data_t *aai = chan->pdrvdata;

   /*------------------------------------------------
    * Init channel configuration
    *----------------------------------------------*/
	chan->period_frames = runtime->period_size;
	chan->period_bytes  = frames_to_bytes(runtime, runtime->period_size);
   chan->fifo[0].mute= chan->fifo[0].mute = 0;

   /*------------------------------------------------
    * Frequency configuration
    *----------------------------------------------*/
   err = aai_hw_set_channelrate(chan, chan->rate);
   if(err != 0){
      aai_err(aai->dev,"%s(%d) - can't set %d Hz it's an illegal frequency\n",chan->name, chan->ipcm, chan->rate);
      goto out;
   }else{
      aai_dbg(aai->dev,"%s(%d) - frequency is now %d Hz \n", chan->name, chan->ipcm, chan->rate);
   }

   aai_info(chan2dev(chan),"%s(%d) - %d Hz, Period: %d frames(%d bytes)\n",
                                 chan->name, chan->ipcm,chan->rate,
                                 chan->period_frames,chan->period_bytes);

out:
	return err;
}

/** Free operator.
 *
 */
static int aai_ops_free(struct snd_pcm_substream *substream)
{
   devdma_hw_free(substream);
   return 0;
}

/** Pointer operator.
 * return the amount of frames used within the audio buffer
 */
static snd_pcm_uframes_t aai_ops_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
   aai_device_t * chan = aai_pchan_from_substream(substream);

	ssize_t bytes = chan->fifo[0].pcurrent - chan->fifo[0].pstart;

	return bytes_to_frames(runtime, bytes);
}


/**
 * Trigger operator.
 */
extern int   nbirq;


static int aai_ops_trigger(struct snd_pcm_substream *substream,
                           int   cmd)
{
   aai_device_t * chan    = aai_pchan_from_substream(substream);
   struct card_data_t *aai = chan->pdrvdata;

	int err = 0;

   if(chan == NULL){
      aai_err(aai->dev,"channel pointer is NULL substream@%p", substream);
      err = -EINVAL;
      goto out;
   }
   if(chan->ipcm >= AAI_NB_CHANNELS){
      aai_err(aai->dev,"substream@%p - channel identifier %d is out of range",
               substream, chan->ipcm);
      err = -EINVAL;
      goto out;
   }

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
      aai_info(aai->dev,"%s(%d) - TRIGGER_START\n", chan->name, chan->ipcm);
      chan->bytes_count = 0;nbirq = 0;
      aai_hw_start_channel(chan);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
      aai_info(aai->dev,"%s(%d) - TRIGGER_RESUME\n", chan->name, chan->ipcm);
		//snd_card_dummy_pcm_timer_start(dpcm);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
      aai_info(aai->dev,"%s(%d) - TRIGGER_STOP %d\n", chan->name, chan->ipcm, nbirq);
      aai_hw_stop_channel(chan);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
      aai_info(aai->dev,"%s(%d) - TRIGGER_SUSPEND\n", chan->name, chan->ipcm);
		//snd_card_dummy_pcm_timer_stop(dpcm);
		break;
	default:
      aai_info(aai->dev,"%s(%d) - TRIGGER UNKNOWN\n", chan->name, chan->ipcm);
		err = -EINVAL;
		break;
	}

out:
	return err;
}


static int aai_ops_ioctl(struct snd_pcm_substream *substream,
				  unsigned int cmd, void *arg)
{
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}




struct snd_pcm_ops snd_pcm_multplay_ops = {
	.open      = aai_ops_open,
	.close     = aai_ops_close,
	.ioctl     = aai_ops_ioctl,
	.hw_params = aai_ops_params,
	.hw_free   = aai_ops_free,
	.prepare   = aai_ops_prepare,
	.trigger   = aai_ops_trigger,
	.pointer   = aai_ops_pointer,
	.mmap      = aai_ops_mmap,
};

struct snd_pcm_ops snd_pcm_multcap_ops = {
	.open      = aai_ops_open,
	.close     = aai_ops_close,
	.ioctl     = aai_ops_ioctl,
	.hw_params = aai_ops_params,
	.hw_free   = aai_ops_free,
	.prepare   = aai_ops_prepare,
	.trigger   = aai_ops_trigger,
	.pointer   = aai_ops_pointer,
	.mmap      = aai_ops_mmap,
};


