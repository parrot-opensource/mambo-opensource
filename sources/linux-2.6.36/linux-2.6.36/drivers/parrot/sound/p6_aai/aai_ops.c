/**
 * @file   aai_ops.c
 * @brief  AAI ALSA callback operators
 *
 * @author gregoire.etienne@parrot.com
 * @date   2008-10-02
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *      Parrot Advanced Audio Interface Driver
 *
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>

#include "aai.h"
#include "aai_hw.h"

/*
 * BUFFER MEMORY MANAGEMENT
 */

static void devdma_hw_free(struct snd_pcm_substream *substream)
{
    int err;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_dma_buffer *buf __maybe_unused = runtime->dma_buffer_p;
    #if CONFIG_AAI_DBG_LEVEL > 1
    aai_device_t * chan = substream2chan(substream);
    #endif

    err = snd_pcm_lib_free_pages(substream);
    if (!err)
        aai_info(chan2dev(chan),"%s (%d) - free @0x%p\n",
                 chan->name, chan->ipcm, buf);
    else
        aai_info(chan2dev(chan),"%s (%d) - unable to free dma area\n",
                 chan->name, chan->ipcm);
}

static int devdma_hw_alloc(struct snd_pcm_substream *substream, size_t size)
{
    int ret;
    struct snd_pcm_runtime *runtime __maybe_unused = substream->runtime;
    #if CONFIG_AAI_DBG_LEVEL > 1
    aai_device_t * chan = substream2chan(substream);
    #endif

    ret = snd_pcm_lib_malloc_pages(substream, size);
    if (ret >= 0)
        aai_info(chan2dev(chan), "%s(%d) - allocate %d bytes @0x%p\n",
                                 chan->name, chan->ipcm, size, runtime->dma_buffer_p);
    else
        aai_info(chan2dev(chan), "%s(%d) - failed to allocate %d bytes\n",
                                 chan->name, chan->ipcm, size);

    return ret;
}

static int devdma_mmap(struct device *dev, struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    return dma_mmap_coherent(NULL, vma, runtime->dma_area, runtime->dma_addr, runtime->dma_bytes);
}

static int aai_ops_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
    return devdma_mmap(NULL, substream, vma);
}

static inline void* aai_bufaddr(struct snd_pcm_runtime *runtime)
{
    return (void *)runtime->dma_addr;/* physical address */
}

/**
 * @brief Sets the substream runtime hardware parameters.
 *        snd_soc_set_runtime_hwparams - set the runtime hardware parameters
 *
 * @param substream  the pcm substream
 * @param hw         the hardware parameters
 *
 */
static inline int copy_runtime_hwparams(struct snd_pcm_runtime *runtime, struct snd_pcm_hardware *hw)
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

/**
 * Open operator
 * Initialize hw_params of the substream's runtime
 */
static int aai_ops_open(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    aai_device_t * chan = substream2chan(substream);
    int ret=0;

    aai_api(chan2dev(chan),"%s(%d) - %s\n",
                           chan->name, chan->ipcm, __FUNCTION__);

    ret = copy_runtime_hwparams(runtime, chan->hw);
    if (ret != 0)
    {
        aai_err(chan2dev(chan),"%s(%d) - hwparams copy failed\n", chan->name, chan->ipcm);
        goto out;
   }

    /*
     * Add rule describing hardware rate dependency
     * on the configuration.
     */
    /*ret = snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
                                 hw_rule_rate_by_channels, NULL,
                                 SNDRV_PCM_HW_PARAM_RATE, -1);*/

    if (ret < 0)
    {
        aai_err(chan2dev(chan),"adding rule failed\n");
    }

out:
    return ret;
}

/**
 * Close operator
 * Disable device
 */
static int aai_ops_close(struct snd_pcm_substream *substream)
{
    int err = 0;
    aai_device_t * chan = substream2chan(substream);
    struct card_data_t *aai = chan->pdrvdata;

    aai_api(chan2dev(chan),"%s(%d) - %s\n",
            chan->name, chan->ipcm, __FUNCTION__);
    aai->spec_ops->dma_close(chan);

    spin_lock(&aai->hwlock);
    aai->deven = DEV_STOP(aai->deven,chan->ipcm);
    aai_dbg(aai->dev,"0x%08x: DEVEN STOP\n", aai->deven );
    spin_unlock(&aai->hwlock);

    return err;
}

/**
 * Params operator
 *
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
    int nbchans, format __maybe_unused, bufsize, err= 0;
    uint32_t access;
    aai_device_t * chan = substream2chan(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;

    aai_api(chan2dev(chan), "%s(%d) - %s\n",
                            chan->name, chan->ipcm, __FUNCTION__);

    chan->bufsize = bufsize = params_buffer_bytes(hw_params);
    chan->rate  = params_rate(hw_params);

    access  = params_access(hw_params);
    format  = params_format(hw_params);
    nbchans = params_channels(hw_params);

   /*
    * Audio stream buffer allocation
    */
    err = devdma_hw_alloc(substream, (bufsize+1));
    if (err < 0)
    {
        aai_err(chan2dev(chan),"%s(%d) - can't allocate %d bytes\n",
                               chan->name, chan->ipcm, bufsize);
        goto out;
    }

   /*
    * Access mode
    * If access mode is forced in configuration, do nothing
    */
    if (!chan->access)
    {
        if ((access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED) ||
            (access == SNDRV_PCM_ACCESS_RW_INTERLEAVED))
        {
            chan->access = AAI_INTERLEAVED;
        }
        else if ((access == SNDRV_PCM_ACCESS_MMAP_NONINTERLEAVED) ||
                 (access == SNDRV_PCM_ACCESS_RW_NONINTERLEAVED))
        {
            chan->access = AAI_NONINTERLEAVED;
        }
        else
        {
            aai_err(chan2dev(chan),"%s(%d) - unknown access type 0x%x\n", chan->name, chan->ipcm, access);
            goto out;
        }
    }
    else
    {
        aai_dbg(chan2dev(chan), "%s(%d) - access mode forced in configuration (%d)\n",
                chan->name, chan->ipcm, chan->access);
    }

   /*------------------------------------------------
    * Init buffer pointers
    *----------------------------------------------*/
   runtime->dma_bytes -= 1; /* XXX patch due to allocation of +1 byte (see later)*/

   /* NON INTERLEAVED devices */
   if ((chan->access == AAI_NONINTERLEAVED))
    {
      /* When dealing with non-interleaved devices, buffer is split */
      bufsize /= nbchans; /* nbchans should be 2 */

      chan->fifo[0].pstart  = aai_bufaddr(runtime);
      chan->fifo[0].pend    = chan->fifo[0].pstart + (bufsize-1);
      chan->fifo[0].pcurrent= chan->fifo[0].pstart;

      chan->fifo[1].pstart  = chan->fifo[0].pstart + (bufsize);
      chan->fifo[1].pend    = chan->fifo[1].pstart + (bufsize-1);
      chan->fifo[1].pcurrent= chan->fifo[1].pstart;

      aai_dbg(chan2dev(chan),"%s(%d) - buffer size 0x%x @[0x%p-0x%p][0x%p-0x%p]\n",
                  chan->name, chan->ipcm, bufsize*nbchans,
                  chan->fifo[0].pstart, chan->fifo[0].pend,
                  chan->fifo[1].pstart, chan->fifo[1].pend);
   }
   else
    {
        /* INTERLEAVED devices */
        chan->fifo[0].pstart    = aai_bufaddr(runtime);
        chan->fifo[0].pend      = chan->fifo[0].pstart + (bufsize-1);
        chan->fifo[0].pcurrent  = chan->fifo[0].pstart;

        aai_dbg(chan2dev(chan),"%s(%d) - buffer size 0x%x @[0x%p-0x%p]\n",
                chan->name, chan->ipcm, bufsize, chan->fifo[0].pstart, chan->fifo[0].pend);
   }

out:
    return err;
}

/**
 * Prepare operator.
 *
 * This callback is called when the pcm is "prepared". You can set
 * the format type, sample rate, etc. here. The difference from hw_params
 * is that the prepare callback will be called each time snd_pcm_prepare()
 * is called, i.e. when recovering after underruns, etc.
 */
static int aai_ops_prepare(struct snd_pcm_substream *substream)
{
    int i, err = 0;

    struct snd_pcm_runtime *runtime = substream->runtime;
    aai_device_t * chan = substream2chan(substream);
    struct card_data_t *aai = chan->pdrvdata;

    aai_api(aai->dev,"%s(%d) - %s\n",
                      chan->name, chan->ipcm, __FUNCTION__);

    /*
     * Init channel configuration
     */
    chan->period_frames = runtime->period_size;
    chan->period_bytes  = frames_to_bytes(runtime, runtime->period_size);
    chan->dma_area = runtime->dma_area;

    for (i = 0; i < QUAD_STREAM; i++)
    {
        /* XXX read volume at init */
        chan->mute[i]   = 0;
        chan->volume[i] = 0;
    }

    /*
     * Mark this channel as "enabled"
     */
    spin_lock(&aai->hwlock);

    if (DEV_CONFLICT(aai->deven,chan->devexclusion))
    {
        /* this device can't be set due to another opened device */
        aai_err(aai->dev,"%s(%d) - device conflict deven:0x%08x\n", chan->name, chan->ipcm, aai->deven);
        err = -EINVAL;
        spin_unlock(&aai->hwlock);
        goto out;
    }
    else
    {
        aai->deven = DEV_START(aai->deven,chan->ipcm);
        aai_dbg(aai->dev,"0x%08x: DEVEN START\n", aai->deven );
    }

    /*
     * Rate configuration
     */
    err = aai_hw_rules(aai, chan->regrules);
    if (err != 0)
    {
        aai->deven = DEV_STOP(aai->deven,chan->ipcm);
        goto out;
    }

    err = aai->spec_ops->set_channelrate(chan, chan->rate);
    if(err != 0)
    {
        aai->deven = DEV_STOP(aai->deven, chan->ipcm);
        goto out;
    }

    /*
     * DMA configuration
     */
    chan->fifo[0].pstart    = aai_bufaddr(runtime);
    chan->fifo[0].pend      = chan->fifo[0].pstart + (chan->bufsize-1);
    chan->fifo[0].pcurrent  = chan->fifo[0].pstart;

    aai->spec_ops->dma_prepare(chan);

out:
    spin_unlock(&aai->hwlock);
    return err;
}

/**
 * Free operator.
 */
static int aai_ops_free(struct snd_pcm_substream *substream)
{
    #if CONFIG_AAI_DBG_LEVEL > 2
    aai_device_t * chan = substream2chan(substream);
    #endif
    aai_api(chan2dev(chan),"%s(%d) - %s\n",
            chan->name, chan->ipcm, __FUNCTION__);
    devdma_hw_free(substream);
    return 0;
}

/**
 * Pointer operator.
 * return the amount of frames used within the audio buffer
 */
static snd_pcm_uframes_t aai_ops_pointer(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    aai_device_t * chan = substream2chan(substream);
    ssize_t bytes;

    if (chan->access == AAI_NONINTERLEAVED)
    {
        bytes = chan->fifo[0].pcurrent - chan->fifo[0].pstart
              + chan->fifo[1].pcurrent - chan->fifo[1].pstart;
    }
    else
    {
        bytes = chan->fifo[0].pcurrent - chan->fifo[0].pstart;
    }

    return bytes_to_frames(runtime, bytes);
}

/**
 * Trigger operator.
 */
static int aai_ops_trigger(struct snd_pcm_substream *substream,
                           int    cmd)
{
    aai_device_t * chan = substream2chan(substream);
    struct card_data_t *aai = chan->pdrvdata;
    int err = 0;

    if (chan == NULL)
    {
        aai_err(chan2dev(chan),"device pointer is NULL substream@%p", substream);
        err = -EINVAL;
        goto out;
    }
    if (chan->ipcm >= aai->pcms_cnt)
    {
        aai_err(chan2dev(chan),"substream@%p - device identifier %d is out of range",
                substream, chan->ipcm);
        err = -EINVAL;
        goto out;
    }

    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
            chan->bytes_count = 0;
            aai_api(chan2dev(chan),"%s(%d) - TRIGGER_START\n", chan->name, chan->ipcm);
            aai_hw_start_channel(chan);
            break;
        case SNDRV_PCM_TRIGGER_RESUME:
            aai_api(chan2dev(chan),"%s(%d) - TRIGGER_RESUME\n", chan->name, chan->ipcm);
            break;
        case SNDRV_PCM_TRIGGER_STOP:
            aai_api(chan2dev(chan),"%s(%d) - TRIGGER_STOP it:%d (fiq %d) %d\n",
                    chan->name, chan->ipcm, chan->nbirq, chan->nbfiq, chan->nbper); /* TODO: only for tests */
            /* aai_info(chan2dev(chan),"%s(%d) - TRIGGER_STOP\n", chan->name, chan->ipcm); */
            aai_hw_stop_channel(chan);
            break;
        case SNDRV_PCM_TRIGGER_SUSPEND:
            aai_api(chan2dev(chan),"%s(%d) - TRIGGER_SUSPEND\n", chan->name, chan->ipcm);
            break;
        default:
            aai_warn(chan2dev(chan),"%s(%d) - TRIGGER UNKNOWN\n", chan->name, chan->ipcm);
            err = -EINVAL;
            break;
    }

out:
    return err;
}

static int aai_ops_ioctl(struct snd_pcm_substream *substream,
                         unsigned int cmd,
                         void *arg)
{
    return snd_pcm_lib_ioctl(substream, cmd, arg);
}

struct snd_pcm_ops aai_pcm_ops =
{
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

