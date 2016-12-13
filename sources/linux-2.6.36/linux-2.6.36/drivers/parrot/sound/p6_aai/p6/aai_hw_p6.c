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
#include "aai_hw_p6.h"

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
 * TODO: factor
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
        chan->mute[i] = 1;
    }
    else
    {
        chan->mute[i] = 0;
        chan->volume[i] = vol/UNIT_PER_DB;
    }
}

/**
 * Get DMA chunk size in bytes.
 * A chunk is N DMA size unit where N is a power of 2
 *
 * @param chan pointer to an audio device descriptor
 * @return number of bytes
 */
static int aai_getdmacnt2bytes(aai_device_t *chan)
{
    struct card_data_t *aai = chan->pdrvdata;
    uint32_t reg = 0;
    uint32_t dmasize = 0;
    uint32_t countpow;
    uint32_t count = 1;

    if (DEV_MUSIC(chan->ipcm))
    {
        reg = AAI_MUSIC_DMA_COUNT;
        dmasize = chan->hw->period_bytes_min;
    }
    else if (DEV_AUX(chan->ipcm))
    {
        reg = AAI_AUX_DMA_COUNT;
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
    else if (DEV_PCM1(chan->ipcm))
    {
        reg = AAI_VOICE_DMA_COUNT;
        dmasize = dmasize_pcm[PCM1_16K(aai_readreg(aai,AAI_PCM1_CFG))];
    }
    else
    {
        aai_err(aai->dev,"%s(%d) - unknown channel\n", chan->name, chan->ipcm);
        return 0;
    }

    countpow = aai_readreg(aai, reg);

    while (countpow != 0)
    {
        countpow -= 1;
        count *= 2;
    }

    aai_dbg(aai->dev,"%s(%d) - get dmacnt=%d, dmasize=%d -> %d bytes\n",
            chan->name, chan->ipcm, count, dmasize, (dmasize * count));

    return dmasize * count;
}

static int aai_hw_getdmacnt(struct card_data_t *aai, uint32_t ipcm)
{
    return 0;
}

static int aai_hw_setdmacnt(struct card_data_t *aai, uint32_t ipcm, int nbdma)
{
    return 0;
}

static int aai_hwdma_prepare(aai_device_t *chan)
{
    struct card_data_t *aai = chan->pdrvdata;

    spin_lock(&aai->hwlock);

    /*
     * Set buffer sizes
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
    else if (DEV_PCM1(chan->ipcm))
    {
        /*
         * PCM 1
         */
        if ((AAI_PCM1_CFG_16K&aai_readreg(aai,AAI_PCM1_CFG)) != val)
        {
            if (!pcm1_rate_locked(aai->deven, chan->ipcm))
            {
                aai_setbit(aai, AAI_PCM1_CFG, AAI_PCM1_CFG_16K, val);
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
         * ERROR
         * not a PCMx device
         */
        aai_err(aai->dev,"%s(%d) - is not a pcm device\n",chan->name, chan->ipcm);
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
            err = aai_hw_set_multfixrate(chan, rate);
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
            aai_dbg(chan2dev(chan),"%s(%d) - unknown device\n",chan->name, chan->ipcm);
            err= -EINVAL;
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
 * TODO: factor
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
        case AAI_SPK_AUX:
            hw_getvolume(aai, chan, 0, AAI_LEFT_AUX_VOL_OUT0);
            hw_getvolume(aai, chan, 1, AAI_RIGHT_AUX_VOL_OUT0);
            hw_getvolume(aai, chan, 2, AAI_LEFT_AUX_VOL_OUT1);
            hw_getvolume(aai, chan, 3, AAI_RIGHT_AUX_VOL_OUT1);
            break;
        case AAI_SPK_16KHZ:
            hw_getvolume(aai, chan, 0, AAI_LEFT_16KHZ_VOL_OUT0);
            hw_getvolume(aai, chan, 1, AAI_RIGHT_16KHZ_VOL_OUT0);
            hw_getvolume(aai, chan, 2, AAI_LEFT_16KHZ_VOL_OUT1);
            hw_getvolume(aai, chan, 3, AAI_RIGHT_16KHZ_VOL_OUT1);
            break;
        case AAI_SPK_8KHZ:
            hw_getvolume(aai, chan, 0, AAI_LEFT_8KHZ_VOL_OUT0);
            hw_getvolume(aai, chan, 1, AAI_RIGHT_8KHZ_VOL_OUT0);
            hw_getvolume(aai, chan, 2, AAI_LEFT_8KHZ_VOL_OUT1);
            hw_getvolume(aai, chan, 3, AAI_RIGHT_8KHZ_VOL_OUT1);
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
        case AAI_SPK_AUX:
            hw_setvolume(aai,chan,0,AAI_LEFT_AUX_VOL_OUT0);
            hw_setvolume(aai,chan,1,AAI_RIGHT_AUX_VOL_OUT0);
            hw_setvolume(aai,chan,2,AAI_LEFT_AUX_VOL_OUT1);
            hw_setvolume(aai,chan,3,AAI_RIGHT_AUX_VOL_OUT1);
            break;
        case AAI_SPK_16KHZ:
            hw_setvolume(aai,chan,0,AAI_LEFT_16KHZ_VOL_OUT0);
            hw_setvolume(aai,chan,1,AAI_RIGHT_16KHZ_VOL_OUT0);
            hw_setvolume(aai,chan,2,AAI_LEFT_16KHZ_VOL_OUT1);
            hw_setvolume(aai,chan,3,AAI_RIGHT_16KHZ_VOL_OUT1);
            break;
        case AAI_SPK_8KHZ:
            hw_setvolume(aai,chan,0,AAI_LEFT_8KHZ_VOL_OUT0);
            hw_setvolume(aai,chan,1,AAI_RIGHT_8KHZ_VOL_OUT0);
            hw_setvolume(aai,chan,2,AAI_LEFT_8KHZ_VOL_OUT1);
            hw_setvolume(aai,chan,3,AAI_RIGHT_8KHZ_VOL_OUT1);
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
static int aai_hw_set_srconvdir_p6(struct card_data_t *aai, int val)
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

static int aai_hw_get_srconvrate_p6(struct card_data_t *aai)
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
static int aai_hw_set_srconvrate_p6(struct card_data_t *aai, int rate)
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
 * @brief AAI P6 initialization
 */
static int aai_hw_init_p6(struct card_data_t *aai)
{
    uint32_t cfg;

    cfg = AAI_CFG_COMPACT         |   /* set to 44,1kHz */
          AAI_CFG_MASTER_CLK_4    |   /* master clock is serial bit clock x 4 */
          (AAI_CFG_AUX_FIFO_32 << AAI_CFG_AUX_FIFO_SHIFT) | /* aux fifo depth is 32 */
          AAI_CFG_RUN_MULT        |   /* enable multimedia interfaces */
          AAI_CFG_USE_ICH2        |   /* enable music input2 interfaces */
          AAI_CFG_USE_ICH3        |   /* enable music input3 interfaces */
          AAI_CFG_FEEDBACK        |   /* enable feedback interfaces */
          (AAI_CFG_VOICE_SYNC_PCM0 << AAI_CFG_VOICE_SYNC_SHIFT); /* sync voice & pcm0 */

    aai_writereg(aai, cfg, AAI_CFG);

    /* JPL fix to enable ICH2 and ICH3 */
    aai_writereg(aai, 0x0, AAI_I2S_FORMAT);
    /* BUG:
     * ICH2 and ICH3 seems to need that something has been written in AAI_I2S_FORMAT
     * to be enabled.... to CHECK
     */

    aai_writereg(aai, AAI_PCM0_CFG_RUN, AAI_PCM0_CFG); /* enable PCM0 interface */
    aai_writereg(aai, AAI_PCM1_CFG_RUN, AAI_PCM1_CFG); /* enable PCM1 interface */

    /* Do not use DMA! */
    aai_writereg(aai, 0, AAI_DMACTL);
    aai_writereg(aai, 0,      AAI_VOICE_DMA_COUNT);
    aai_writereg(aai, 0,      AAI_AUX_DMA_COUNT);
    aai_writereg(aai, 0,      AAI_MUSIC_DMA_COUNT);
    aai_writereg(aai, 0,      AAI_LOUD_CTL);
    aai_writereg(aai, 0,      AAI_LOUDREF_OUT0);
    aai_writereg(aai, 0,      AAI_LOUDREF_OUT1);

    return 0;
}

static void aai_hw_enable_bitclk(void)
{
    parrot_select_pin(AC_CLKM);
}

static void aai_hw_enable_mstclk(void)
{
    parrot_select_pin(MCLK);
}

static void aai_hw_disable_bitclk(void)
{
    parrot_select_pin(AC_CLKM|1);
}

static void aai_hw_disable_mstclk(void)
{
    parrot_select_pin(MCLK|1);
}

struct spec_ops_t aai_spec_ops_p6 =
{
    .hw_init         = aai_hw_init_p6,

    .set_channelrate = aai_hw_set_channelrate,
    .get_volume      = aai_hw_getvolume,
    .set_volume      = aai_hw_setvolume,

    .get_dma_cnt     = aai_hw_getdmacnt,
    .set_dma_cnt     = aai_hw_setdmacnt,
    .dma_prepare     = aai_hwdma_prepare,
    .dma_close       = aai_hwdma_close,

    .set_srconvdir  = aai_hw_set_srconvdir_p6,
    .set_srconvrate = aai_hw_set_srconvrate_p6,
    .get_srconvrate = aai_hw_get_srconvrate_p6,

    .disable_i2s_bit_clock    = aai_hw_disable_bitclk,
    .enable_i2s_bit_clock     = aai_hw_enable_bitclk,
    .disable_i2s_master_clock = aai_hw_disable_mstclk,
    .enable_i2s_master_clock  = aai_hw_enable_mstclk,
};

int aai_init_card_p6(struct device *dev, struct card_data_t *aai, int dev_id)
{
    int32_t err = 0;
    struct snd_card *card = aai->card;

    aai->pcms_cnt = AAI_NB_CHANNELS;

    /*
     * Set appropriate channels structure
     */
    aai->chans = aai_channels_p6;

    aai->spec_ops = &aai_spec_ops_p6;

#if CONFIG_AAI_DBG_LEVEL > 0
    aai->vaai = vaai_p6;
#endif

    /*
     * Register IOCTL
     */
    aai_ioctl_hwdep_new_p6(aai, "AAIhwdep", 0);

    /*
     * Set IRQ handler
     */
    aai->irq_handler = aai_irq_p6;

    /*
     * Initialize Mixer
     */
    err = aai_mixer_new_p6(aai);
    if (err < 0)
    {
        aai_err(dev, "%s mixer init failed \n", __FUNCTION__);
        goto exit;
    }
    aai_dbg(dev, "mixer initialization ok\n");

    /*
     * Set Sound Card datas
     */
    strcpy(card->driver, "AAI P6");
    strcpy(card->shortname, "AAI");
    sprintf(card->longname, "P6 Advanced Audio Interface %i", dev_id + 1);

exit:
    return err;
}

