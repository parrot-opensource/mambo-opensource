/**
 * @file   aai_hw_p6.c
 * @brief  AAI P6 specific hardware layer
 *
 * @author adrien.charruel@parrot.com
 * @date   2010-05-05
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *      Parrot Advanced Audio Interface Driver
 *
 */

#include "aai.h"
#include "aai_hw.h"
#include "aai_hw_p6i.h"
#include "aai_codec_p6i.h"
#include <mach/aai.h>

/**
 * This table describes the Auxiliary DMA size unit
 *  depending the AUX fifo depth configuration
 */
static uint32_t dmasize_pcm[2] =   /* 8k */         /* 16k */
                                {DMASIZE_2w32bits, DMASIZE_4w32bits};

/**
 * Set multimedia fixed rate.
 * Set channel rate if it's possible.
 *
 * @param chan pointer to an audio device descriptor
 * @param rate numerical value
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
static int aai_hw_set_multfixrate(aai_device_t *chan, int rate)
{
    uint32_t dac_rate,val;
    struct card_data_t *aai = chan->pdrvdata;

    if ((rate != 44100)&&(rate != 48000))
        goto error;

    dac_rate = aai_readreg(aai,AAI_CFG) & 0x1 ? 441 : 480; /* dac_rate unit is 100hz */

    if ((dac_rate*100) == rate)
        goto out; /* no need to change compact bit */

    if (compactbit_locked(aai->deven, chan->ipcm))
    {
        aai_err(aai->dev,"%s(%d) - %dHz compact bit is used deven:0x%08x\n",
                chan->name, chan->ipcm, rate, aai->deven);
        goto error;
    }

    val = (rate == 48000) ? 0 : 1; /* get compact bit value to set */
    aai_setbit(aai, AAI_CFG, AAI_CFG_COMPACT, val);/* set compact bit value */

out:
    return 0;

error:
    return -EINVAL;
}

/**
 * Get hardware volume.
 * get volume value for one channel of a device
 *
 * @param aai pointer to driver descriptor
 * @param chan pointer to an audio device descriptor
 * @param i volume channel index
 * @param offset volume register offset
 */
static void hw_getvolume(struct card_data_t *aai, aai_device_t *chan, int i, uint32_t offset)
{
    int vol;

    if (i > QUAD_STREAM)
        return;

    vol = aai_readreg(aai, offset);

    if (vol == MUTE_VOLUME)
    {
        chan->mute[i]=1;
    }
    else
    {
        chan->mute[i] = 0;
        chan->volume[i] = vol / UNIT_PER_DB;
    }
}

static void codec_init(struct card_data_t *aai)
{
    aai_dbg(aai->dev, "%s\n", __FUNCTION__);

    /* configure internal codec: power-up all channels */
    writel(0x3fe, PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL0);

    /* record sel = input1 (differential), i2sfs = 48 kHz */
    writel(3|(3 << 8)|(1 << (3+17)), PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL1);

    /* gains = 0 dB on all channels */
    writel(0x0c | (0x0c << 8) | (0x12|(0x12 << 8)) << 16, PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL2);

    /* unmute all channels */
    writel(0, PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL3);

    aai_codec_latch(1);
}

static int aai_getdmacnt2bytes(aai_device_t *chan)
{
    struct card_data_t *aai = chan->pdrvdata;
    uint32_t reg = 0;
    uint32_t dmasize = 0;
    uint32_t countpow;
    uint32_t count;

    if (DEV_MUSIC(chan->ipcm))
    {
        reg = AAI_MUSIC_DMA_COUNT;
        dmasize = chan->hw->period_bytes_min;
    }
    else if ((DEV_VOICE_8K(chan->ipcm)) || (DEV_VOICE_16K(chan->ipcm)))
    {
        reg = AAI_VOICE_DMA_COUNT;
        dmasize = chan->hw->period_bytes_min;
    }
    else if (DEV_PCM0(chan->ipcm))
    {
        reg = AAI_VOICE_DMA_COUNT;
        dmasize = dmasize_pcm[PCM0_16K(aai_readreg(aai,AAI_PCM0_CFG))];
    }
    else
    {
        aai_err(aai->dev, "%s(%d) - unknown channel\n", chan->name, chan->ipcm);
        return 0;
    }

#if CONFIG_AAI_IRQ_SHARED
    count = chan->period_bytes / dmasize;

    countpow = 0;
    while (count > 0 && countpow < 3)
    {
        countpow++;
        count >>= 1;
    }

    aai_writereg(aai, countpow, reg);
#else
    countpow = aai_readreg(aai, reg);
#endif

    count = 1;
    while (countpow != 0)
    {
        countpow -= 1;
        count *= 2;
    }
    aai_dbg(aai->dev, "%s(%d) - get dmacnt=%d, dmasize=%d -> %d bytes\n",
            chan->name, chan->ipcm, count, dmasize, (dmasize * count));

    return dmasize * count;
}

static int aai_hw_getdmacnt(struct card_data_t *aai,
                            uint32_t           ipcm)
{
    uint32_t count = 1;
    uint32_t countpow = 0;
    uint32_t reg;
    aai_device_t *chan = &(aai->chans[ipcm]);

    spin_lock(&aai->hwlock);

    /*
     * check if any other device of it's group has already been started
     * because this would lead to a major failure
     */
    /* TODO: add P6i devices */
    if (DEV_MUSIC(chan->ipcm))
    {
        reg = AAI_MUSIC_DMA_COUNT;
    }
    else if (DEV_VOICE_8K(chan->ipcm))
    {
        reg = AAI_VOICE_DMA_COUNT;
    }
    else if (DEV_VOICE_16K(chan->ipcm))
    {
        reg = AAI_VOICE_DMA_COUNT;
    }
    else if (DEV_PCM0(chan->ipcm))
    {
        reg = AAI_VOICE_DMA_COUNT;
    }
    else
    {
        goto out;
    }

    countpow = aai_readreg(aai, reg);
    while (countpow != 0)
    {
        /* @review identique a count = 1 << countpow; */
        countpow -= 1;
        count *= 2;
    }

    spin_unlock(&aai->hwlock);
    return count;

out:
    spin_unlock(&aai->hwlock);
    aai_warn(aai->dev,"%s(%d) - Get DMASIZE failed\n", chan->name, chan->ipcm);
    return 0;
}

static int aai_hw_setdmacnt(struct card_data_t *aai,
                            uint32_t           ipcm,
                            int                nbdma)
{
    int32_t err = -EINVAL;
    uint32_t countpow = 0;
    uint32_t reg;
    aai_device_t *chan = &(aai->chans[ipcm]);

    spin_lock(&aai->hwlock);

    /*
     * check if any other device of it's group has already been started
     * if it does, DMACNT can't be modified */
    /* TODO: add P6i devices */
    if (DEV_MUSIC(chan->ipcm) && (!dmacnt_music_locked(aai->deven)))
    {
        reg = AAI_MUSIC_DMA_COUNT;
    }
    else if (DEV_VOICE_8K(chan->ipcm) && (!dmacnt_voice_locked(aai->deven)))
    {
        reg = AAI_VOICE_DMA_COUNT;
    }
    else if (DEV_VOICE_16K(chan->ipcm) && (!dmacnt_voice_locked(aai->deven)))
    {
        reg = AAI_VOICE_DMA_COUNT;
    }
    else if (DEV_PCM0(chan->ipcm) && (!dmacnt_voice_locked(aai->deven)))
    {
        reg = AAI_VOICE_DMA_COUNT;
    }
    else
    {
        goto out;
    }

    /* TODO: add P6i restrictions */
    if (((reg == AAI_VOICE_DMA_COUNT) && (nbdma>AAI_DMACNT_MAX_VOICE)) ||
        ((reg == AAI_MUSIC_DMA_COUNT) && (nbdma>AAI_DMACNT_MAX_MUSIC)))
        goto out; /* illegal value */

    while (nbdma != 0)
    {
        /* @review identique a count = 1 >> countpow; */
        nbdma /= 2;
        countpow++;
    }
    aai_writereg(aai, countpow - 1, reg);
    aai_dbg(aai->dev,"%s(%d) - dmacnt=%08x \n",
            chan->name, chan->ipcm, (countpow - 1));

    spin_unlock(&aai->hwlock);
    return 0;

out:
    spin_unlock(&aai->hwlock);
    return err;
}

static int aai_hwdma_prepare(aai_device_t *chan)
{
    uint32_t reg;
    struct card_data_t *aai = chan->pdrvdata;

    spin_lock(&aai->hwlock);

   /*
    * Enable DMA control registers
    * all DMA devices are set during aai init except from spk-out0 and spk-out1
    * which dmactl bit depends on access type
    */
   if ((chan->ipcm == AAI_SPK_OUT0) || (chan->ipcm == AAI_SPK_OUT1))
   {
      if (chan->ipcm == AAI_SPK_OUT0)
      {
         chan->dmaflag = (chan->access == AAI_INTERLEAVED) ? AAI_DMACTL_OUT0_INTERLEAVE : 0;
      }
      else if (chan->ipcm == AAI_SPK_OUT1)
      {
         chan->dmaflag = (chan->access == AAI_INTERLEAVED) ? AAI_DMACTL_OUT1_INTERLEAVE : 0;
      }

      /* Enable DMA flag */
      reg = aai_readreg(aai, AAI_DMACTL) | chan->dmaflag;
      aai_writereg(aai, reg, AAI_DMACTL);
   }

   /*
    * Set DMA buffer registers
    */
   chan->dmaxfersz = aai_getdmacnt2bytes(chan);

   /* If we're dealing with non-interleaved devices */
   if (DEV_SRCONV_OUT(chan->ipcm) && (chan->access == AAI_NONINTERLEAVED))
   {
       chan->dmaxfersz /= 2;
       /* Write DMA buffer registers */
       chan->fifo[0].pfollow = chan->fifo[0].pcurrent + chan->dmaxfersz;
       aai_writereg(aai, (uint32_t)chan->fifo[0].pcurrent, (uint32_t)chan->fifo[0].dmasa);
       aai_writereg(aai, (uint32_t)chan->fifo[0].pfollow,  (uint32_t)chan->fifo[0].dmafa);

       chan->fifo[1].pfollow = chan->fifo[1].pcurrent + chan->dmaxfersz;
       aai_writereg(aai, (uint32_t)chan->fifo[1].pcurrent, (uint32_t)chan->fifo[1].dmasa);
       aai_writereg(aai, (uint32_t)chan->fifo[1].pfollow , (uint32_t)chan->fifo[1].dmafa);
   }
   else
   {
       /*
        * If we're dealing with interleaved devices
        * Write DMA buffer registers
        */
       chan->fifo[0].pfollow = chan->fifo[0].pcurrent + chan->dmaxfersz;
       aai_writereg(aai, (uint32_t)chan->fifo[0].pcurrent, (uint32_t)chan->fifo[0].dmasa);
       aai_writereg(aai, (uint32_t)chan->fifo[0].pfollow,  (uint32_t)chan->fifo[0].dmafa);
   }
   spin_unlock(&aai->hwlock);

   return 0;
}

static int aai_hwdma_close(aai_device_t *chan)
{
    uint32_t reg;
    struct card_data_t *aai = chan->pdrvdata;

    if ((chan->ipcm == AAI_SPK_OUT0) || (chan->ipcm == AAI_SPK_OUT1))
    {
        /*
         * must be done in case that prepare has not been called
         * (ie when doing open/close on a device ... portaudio does on startup
         */
        if(chan->ipcm == AAI_SPK_OUT0)
        {
            chan->dmaflag = (chan->access == AAI_INTERLEAVED) ? AAI_DMACTL_OUT0_INTERLEAVE : 0;
        }
        else if (chan->ipcm == AAI_SPK_OUT1)
        {
            chan->dmaflag = (chan->access == AAI_INTERLEAVED) ? AAI_DMACTL_OUT1_INTERLEAVE : 0;
        }

        /* Disable DMA flag */
        spin_lock(&aai->hwlock);
        reg = aai_readreg(aai, AAI_DMACTL) & ~chan->dmaflag;
        aai_writereg(aai, reg, AAI_DMACTL);
        spin_unlock(&aai->hwlock);
   }
   return 0;
}

/**
 * Set PCM devices rate.
 * Set devices rate if it's possible.
 *
 * @param chan pointer to an audio device descriptor
 * @param rate numerical value (8000 or 16000)
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
static int aai_hw_set_pcmrate(aai_device_t *chan, int rate)
{
    int val, err = 0;
    struct card_data_t *aai = chan->pdrvdata;

    val = (rate == 8000) ? 0 : 1; /* get rate bit value*/

    if (DEV_PCM0(chan->ipcm))
    {
        /*
         * PCM 0
         */
        if ((!!(AAI_PCM0_CFG_16K & aai_readreg(aai, AAI_PCM0_CFG))) != val)
        {
            if (!pcm0_rate_locked(aai->deven, chan->ipcm))
            {
                aai_setbit(aai, AAI_PCM0_CFG, AAI_PCM0_CFG_16K, val);
            }
            else
            {
                aai_warn(aai->dev,"%s(%d) - Can't set to %d Hz, pcm is locked (deven[0x%08x])\n",
                         chan->name, chan->ipcm, rate, aai->deven);
                err = -EINVAL;
            }
       }
    }
    else
    {
        /*
         * not a PCM0 device
         */
        aai_err(aai->dev, "%s(%d) - is not a pcm device\n", chan->name, chan->ipcm);
        err = -EINVAL;
    }
    return err;
}

#define SRCONVOUTLOCKEDCHANS(_ipcm_) ((1<<_ipcm_) & ((1<<AAI_MIC0_MUSIC)    \
                                                  | (1<<AAI_MIC2_MUSIC)     \
                                                  | (1<<AAI_FBACK_MUSIC)))
static int aai_hw_set_channelrate(aai_device_t *chan, int rate)
{
    int32_t srconvout;
    int32_t i2smaster;
    int32_t err = -EINVAL;
    struct card_data_t *aai = chan->pdrvdata;

    spin_lock(&aai->hwlock);
    srconvout = aai_readreg(aai,AAI_ITEN) & AAI_ITEN_SRC_OUT ? 1 : 0;
    i2smaster = aai_readreg(aai,AAI_CFG) & AAI_CFG_AAI_SLAVE ? 0 : 1;

    /*
     * BUG:
     * When SRConv is applied on ouputs, ICH0, ICH2 and ICH3 are disabled.
     */
    if (srconvout && (SRCONVOUTLOCKEDCHANS(chan->ipcm))!= 0)
    {
        aai_dbg(chan2dev(chan),"%s(%d) - conflicts with SRCONV_OUT\n", chan->name, chan->ipcm);
    }
    else
    {
        if (DEV_SRCONV_OUT(chan->ipcm) && srconvout && i2smaster)
        {
            err = aai_hw_set_srconvrate(aai, rate*SRCONV_PRECISION);

            /*
             * SRConvIn depends on i2s rate. If AAI is slave, input rate is unknown
             * If AAI is master, input rate is 44,1kHz or 48kHz
             */
        }
        else if(DEV_SRCONV_IN(chan->ipcm) && (!srconvout) && (!i2smaster))
        {
            err = 0;
        }
        else if (DEV_MULTIMEDIA(chan->ipcm) && ((rate == 44100) || (rate == 48000)))
        {
            err = aai_hw_set_multfixrate(chan,rate);
        }
        else if (DEV_VOICE_8K(chan->ipcm) && (rate == 8000))
        {
            err = 0; /* do nothing */
        }
        else if (DEV_VOICE_16K(chan->ipcm) && (rate == 16000))
        {
            err = 0; /* do nothing */
        }
        else if (DEV_PCM(chan->ipcm) && ((rate == 8000) || (rate == 16000)))
        {
            err = aai_hw_set_pcmrate(chan, rate);
        }
        else
        {
            aai_dbg(chan2dev(chan), "%s(%d) - unknown device\n", chan->name, chan->ipcm);
            err = -EINVAL;
        }
    }

    if (err != 0)
    {
        aai_err(chan2dev(chan),"%s(%d) - can't set %d Hz\n",
                chan->name, chan->ipcm, rate);
    }
    else
    {
        aai_dbg(chan2dev(chan),"%s(%d) - frequency is now %d Hz \n",
                chan->name, chan->ipcm, rate);
    }

    spin_unlock(&aai->hwlock);
    return err;
}

/**
 * Set hardware volume.
 * set volume value for one channel of a device
 *
 * @param aai pointer to driver descriptor
 * @param chan pointer to an audio device descriptor
 * @param i volume channel index
 * @param offset volume register offset
 */
static void hw_setvolume(struct card_data_t *aai, aai_device_t *chan, int i, uint32_t offset)
{
    int vol = chan->volume[i]*UNIT_PER_DB;
    int oldvol = aai_readreg(aai, offset);

    if (i > QUAD_STREAM)
        return; /* XXX return error code */

    if (chan->mute[i])
        vol = MUTE_VOLUME;

    if (vol != oldvol)
        aai_writereg(aai, vol, offset);
}

static int aai_hw_getvolume(aai_device_t *chan)
{
    int32_t err = 0;
    struct card_data_t *aai = chan->pdrvdata;

    spin_lock(&aai->hwlock);
    switch (chan->ipcm)
    {
        case AAI_SPK_OUT0:
            hw_getvolume(aai, chan, 0, AAI_LEFT_MUSIC_VOL_OUT0);
            hw_getvolume(aai, chan, 1, AAI_RIGHT_MUSIC_VOL_OUT0);
            break;
        case AAI_SPK_OUT1:
            hw_getvolume(aai, chan, 0, AAI_LEFT_MUSIC_VOL_OUT1);
            hw_getvolume(aai, chan, 1, AAI_RIGHT_MUSIC_VOL_OUT1);
            break;
        case AAI_SPK_16KHZ:
            hw_getvolume(aai, chan, 0, AAI_LEFT_16KHZ_VOL_OUT0);
            hw_getvolume(aai, chan, 1, AAI_LEFT_16KHZ_VOL_OUT0);
            hw_getvolume(aai, chan, 2, AAI_LEFT_16KHZ_VOL_OUT1);
            hw_getvolume(aai, chan, 3, AAI_LEFT_16KHZ_VOL_OUT1);
            break;
        case AAI_SPK_8KHZ:
            hw_getvolume(aai, chan, 0, AAI_LEFT_8KHZ_VOL_OUT0);
            hw_getvolume(aai, chan, 1, AAI_LEFT_8KHZ_VOL_OUT0);
            hw_getvolume(aai, chan, 2, AAI_LEFT_8KHZ_VOL_OUT1);
            hw_getvolume(aai, chan, 3, AAI_LEFT_8KHZ_VOL_OUT1);
            break;
        default:
            err = -EINVAL;
    }

    spin_unlock(&aai->hwlock);
    return err;
}

static int aai_hw_setvolume(aai_device_t *chan)
{
    int err = 0;
    struct card_data_t *aai = chan->pdrvdata;
    spin_lock(&aai->hwlock);
    switch (chan->ipcm)
    {
        case AAI_SPK_OUT0:
            hw_setvolume(aai,chan,0,AAI_LEFT_MUSIC_VOL_OUT0);
            hw_setvolume(aai,chan,1,AAI_RIGHT_MUSIC_VOL_OUT0);
            break;
        case AAI_SPK_OUT1:
            hw_setvolume(aai,chan,0,AAI_LEFT_MUSIC_VOL_OUT1);
            hw_setvolume(aai,chan,1,AAI_RIGHT_MUSIC_VOL_OUT1);
            break;
        case AAI_SPK_16KHZ:
            hw_setvolume(aai,chan,0,AAI_LEFT_16KHZ_VOL_OUT0);
            hw_setvolume(aai,chan,1,AAI_LEFT_16KHZ_VOL_OUT0);
            hw_setvolume(aai,chan,2,AAI_LEFT_16KHZ_VOL_OUT1);
            hw_setvolume(aai,chan,3,AAI_LEFT_16KHZ_VOL_OUT1);
            break;
        case AAI_SPK_8KHZ:
            hw_setvolume(aai,chan,0,AAI_LEFT_8KHZ_VOL_OUT0);
            hw_setvolume(aai,chan,1,AAI_LEFT_8KHZ_VOL_OUT0);
            hw_setvolume(aai,chan,2,AAI_LEFT_8KHZ_VOL_OUT1);
            hw_setvolume(aai,chan,3,AAI_LEFT_8KHZ_VOL_OUT1);
            break;
        default:
            err = -EINVAL;
            break;
    }

    spin_unlock(&aai->hwlock);
    return err;
}

/**
 * @brief Set Sample rate convertor direction.
 *
 * @param chan pointer to an audio device descriptor
 * @param rate numerical value
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
static int aai_hw_set_srconvdir_p6i(struct card_data_t *aai, int val)
{
    int err = -EINVAL;

    spin_lock(&aai->hwlock);
    if (0 == srconv_is_locked(aai->deven))
    {
        aai_dbg(aai->dev, "Set SRConv direction %d \n",(!!val));
        aai_setbit(aai, AAI_ITEN, AAI_ITEN_SRC_OUT, (!!val));
        err = 0;
    }
    else
    {
        aai_warn(aai->dev,"SRConv direction is locked deven:0x%08x\n", aai->deven);
        /* @review printk peut-il dormir ? si oui le spin_lock doit etre relache avant */
    }
    spin_unlock(&aai->hwlock);

    return err;
}

/**
 * Get Sample rate convertor ratio.
 *
 * @param chan pointer to an audio device descriptor
 * @param rate numerical value
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
#include <asm/div64.h>

static int aai_hw_get_srconvrate_p6i(struct card_data_t *aai)
{
    uint64_t dac_rate, ratio, rate;

    spin_lock(&aai->hwlock);
    dac_rate = aai_readreg(aai,AAI_CFG) & AAI_CFG_COMPACT ? MASTER_DAC_RATE_44K : MASTER_DAC_RATE_48K;
    ratio = aai_readreg(aai, AAI_SRCONV_RATIO);
    spin_unlock(&aai->hwlock);
    rate = (ratio * dac_rate) >> 24;
    aai_dbg(aai->dev,"Get SRConv ratio 0x%08x = %d,%dHz\n",
            (uint32_t)ratio, ((int)rate)/SRCONV_PRECISION, ((int)rate)%SRCONV_PRECISION);

    return (int)rate;
}

/**
 * Set Sample rate convertor ratio.
 *
 * @param chan pointer to an audio device descriptor
 * @param rate numerical value
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
static int aai_hw_set_srconvrate_p6i(struct card_data_t *aai, int rate)
{
    uint64_t ratio=0;
    uint64_t dac_rate64 = aai_readreg(aai,AAI_CFG) & AAI_CFG_COMPACT ? MASTER_DAC_RATE_44K : MASTER_DAC_RATE_48K;

    if ((rate < (MIN_RATIO_RATE * SRCONV_PRECISION)) ||
        ((MAX_RATIO_RATE * SRCONV_PRECISION) < rate))
    {
        aai_warn(aai->dev,"SRConv rate out of limits (%d)\n",rate);
        return -EINVAL;
    }

    ratio = ((uint64_t)rate) << 24;
    /*aai_dbg(aai->dev,"SRConv ratio (0x%llx)(0x%llx)\n",ratio ,dac_rate64);*/
    do_div(ratio, dac_rate64);
    aai_dbg(aai->dev,"Set SRConv ratio %d,%03dHz = 0x%08x(0x%llx)\n",
            rate/SRCONV_PRECISION, rate%SRCONV_PRECISION, (uint32_t)ratio ,ratio);
    aai_writereg(aai, (uint32_t)ratio, AAI_SRCONV_RATIO);

    return 0;
}

/**
 * @brief AAI P6i initialization
 */
static int aai_hw_init_p6i(struct card_data_t *aai)
{
    uint32_t cfg;
    uint32_t dmactl;
    struct parrot_aai_platform_data *pdata = aai->dev->platform_data;

    cfg = AAI_CFG_MASTER_CLK_4    |   /* master clock is serial bit clock x 4 */
          AAI_CFG_GPS_PROMPT      |   /* enable GPS prompt interface          */
          AAI_CFG_RUN_MULT        |   /* enable multimedia interfaces         */
          AAI_CFG_USE_ICH2        |   /* enable music input2 interfaces       */
          AAI_CFG_FEEDBACK        |   /* enable feeback interfaces            */
          AAI_CFG_NOISE_SIZE_16   |
          AAI_CFG_VOICE_SYNC_PCM0;    /* synchronize PCM stream               */
    if(pdata->sync_freq == 44100)
        cfg |= AAI_CFG_COMPACT;

    aai_writereg(aai, cfg, AAI_CFG);

    /*
     * CODEC must be initialized after setting the RUN_MULT bit.
     * This starts the PLL which is required to run the CODEC.
     * Also wait 1us before doing so to insure that PLL is running.
     */
    udelay(1);
    codec_init(aai);

    /* JPL fix to enable ICH2 and ICH3, from p6*/
    aai_writereg(aai, 0x0, AAI_I2S_FORMAT);

    /* enable PCM0 interfaces */
    cfg = AAI_PCM_CONF_RUN          |
          AAI_PCM_CONF_CYCLES_HIGH  |
          AAI_PCM_CONF_CYCLES_LOW;
    aai_writereg(aai, cfg, AAI_PCM0_CFG);

    /*
     * Aet all devices as dma except out0 and out1 which
     * will be set later.
     * Also disable ADC_USB which uses mic 8k and 16k fifo.
     */
    dmactl = 0xFE6CF & ~(AAI_DMACTL_OUT0_INTERLEAVE |
                         AAI_DMACTL_OUT1_INTERLEAVE |
                         AAI_DMACTL_ADC_USB);

    aai_writereg(aai, dmactl, AAI_DMACTL);
    aai_writereg(aai, 0     , AAI_VOICE_DMA_COUNT);
    aai_writereg(aai, 0     , AAI_AUX_DMA_COUNT);
    aai_writereg(aai, 0     , AAI_MUSIC_DMA_COUNT);
    aai_writereg(aai, 0     , AAI_LOUD_CTL);
    aai_writereg(aai, 0     , AAI_LOUDREF_OUT0);
    aai_writereg(aai, 0     , AAI_LOUDREF_OUT1);

    return 0;
}

static void aai_hw_enable_bitclk(void)
{
    parrot_select_pin(P6I_I2S_CLK);
}

static void aai_hw_enable_mstclk(void)
{
    parrot_select_pin(P6I_MCLK);
}

static void aai_hw_disable_bitclk(void)
{
    parrot_select_pin(P6I_I2S_CLK|1);
}

static void aai_hw_disable_mstclk(void)
{
    parrot_select_pin(P6I_MCLK|1);
}

struct spec_ops_t aai_spec_ops_p6i =
{
    .hw_init         = aai_hw_init_p6i,

    .set_channelrate = aai_hw_set_channelrate,
    .get_volume      = aai_hw_getvolume,
    .set_volume      = aai_hw_setvolume,

    .get_dma_cnt     = aai_hw_getdmacnt,
    .set_dma_cnt     = aai_hw_setdmacnt,
    .dma_prepare     = aai_hwdma_prepare,
    .dma_close       = aai_hwdma_close,

    .set_srconvdir  = aai_hw_set_srconvdir_p6i,
    .set_srconvrate = aai_hw_set_srconvrate_p6i,
    .get_srconvrate = aai_hw_get_srconvrate_p6i,

    .disable_i2s_bit_clock    = aai_hw_disable_bitclk,
    .enable_i2s_bit_clock     = aai_hw_enable_bitclk,
    .disable_i2s_master_clock = aai_hw_disable_mstclk,
    .enable_i2s_master_clock  = aai_hw_enable_mstclk,
};

int aai_init_card_p6i(struct device *dev, struct card_data_t *aai, int dev_id)
{
    int32_t err = 0;
    struct snd_card *card = aai->card;

    aai->pcms_cnt = AAI_NB_CHANNELS;

    /*
     * Set appropriate channels structure
     */
    aai->chans = aai_channels_p6i;

    aai->spec_ops = &aai_spec_ops_p6i;

#if CONFIG_AAI_DBG_LEVEL > 2
    aai->vaai = vaai_p6i;
#endif

    /*
     * Register IOCTL
     */
    aai_ioctl_hwdep_new_p6i(aai, "AAIhwdep", 0);

    /*
     * Set IRQ handler
     */
    aai->irq_handler = aai_irq_p6i;

    /*
     * Initialize Mixer
     */
    err = aai_mixer_new_p6i(aai);
    if (err < 0)
    {
        aai_err(dev, "%s mixer init failed \n", __FUNCTION__);
        goto exit;
    }
    aai_dbg(dev, "mixer initialization ok\n");

    /*
     * Set Sound Card datas
     */
    strcpy(card->driver, "AAI P6i");
    strcpy(card->shortname, "AAI");
    sprintf(card->longname, "P6i Advanced Audio Interface %i", dev_id + 1);

exit:
    return err;
}

