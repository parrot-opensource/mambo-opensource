/*
 * linux/drivers/io/aai_hw.c
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

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include "aai.h"
#include "aai_hw.h"


//! Audio Channels
aai_device_t aai_channels[AAI_NB_CHANNELS] = {
    [AAI_CHANNEL_SPK] = {
        .name     = "spk",
        .ipcm     = AAI_CHANNEL_SPK,
        .nbchans  = STEREO_STREAM,
        .ops      = &snd_pcm_multplay_ops,
        .cfgshift = 18,                /* configuration shift in ITEN register */
        .direction= AAI_TX,
        .intflag  = AAI_INT_SPK,
        .enflag   = AAI_FE_SPK,
        .fifo[0]  = { .hfifo    = _AAI_MULT_OUT,},
        .volume   = {0,0},
    },
    [AAI_CHANNEL_MIC12] = {
        .name     = "mic12",
        .ipcm     = AAI_CHANNEL_MIC12,
        .nbchans  = STEREO_STREAM,
        .ops      = &snd_pcm_multcap_ops,
        .cfgshift = 23,                /* configuration shift in ITEN register */
        .direction= AAI_RX,
        .intflag  = AAI_INT_MIC12,
        .enflag   = AAI_FE_MIC12,
        .fifo[0]  = { .hfifo    = _AAI_MULT_IN1,},
    },
    [AAI_CHANNEL_MIC34] = {
        .name     = "mic34",
        .ipcm     = AAI_CHANNEL_MIC34,
        .nbchans  = STEREO_STREAM,
        .ops      = &snd_pcm_multcap_ops,
        .cfgshift = 26,                /* configuration shift in ITEN register */
        .direction= AAI_RX,
        .intflag  = AAI_INT_MIC34,
        .enflag   = AAI_FE_MIC34,
        .fifo[0]  = { .hfifo    = _AAI_MULT_IN2,},
    },
    [AAI_CHANNEL_BT_OUT] = {
        .name     = "btout",
        .ipcm     = AAI_CHANNEL_BT_OUT,
        .nbchans  = MONO_STREAM,
        .ops      = &snd_pcm_multplay_ops,
        .cfgshift = 18,                /* configuration shift in ITEN register */
        .direction= AAI_TX,
        .intflag  = AAI_INT_BT_TX,
        .enflag   = AAI_FE_BT_TX,
        .fifo[0]  = { .hfifo    = _AAI_BT_OUT,},
    },
    [AAI_CHANNEL_BT_IN] = {
        .name     = "btin",
        .ipcm     = AAI_CHANNEL_BT_IN,
        .nbchans  = MONO_STREAM,
        .ops      = &snd_pcm_multcap_ops,
        .cfgshift = 18,                /* configuration shift in ITEN register */
        .direction= AAI_RX,
        .intflag  = AAI_INT_BT_RX,
        .enflag   = AAI_FE_BT_RX,
        .fifo[0]  = { .hfifo    = _AAI_BT_IN,},
    },
    [AAI_CHANNEL_EXT_OUT] = {
        .name     = "extout",
        .ipcm     = AAI_CHANNEL_EXT_OUT,
        .nbchans  = MONO_STREAM,
        .ops      = &snd_pcm_multplay_ops,
        .cfgshift = 20,                /* configuration shift in ITEN register */
        .direction= AAI_TX,
        .intflag  = AAI_INT_EXT_TX,
        .enflag   = AAI_FE_EXT_TX,
        .fifo[0]  = { .hfifo    = _AAI_EXT_OUT,},
    },
    [AAI_CHANNEL_EXT_IN] = {
        .name     = "extin",
        .ipcm     = AAI_CHANNEL_EXT_IN,
        .nbchans  = MONO_STREAM,
        .ops      = &snd_pcm_multcap_ops,
        .cfgshift = 20,                /* configuration shift in ITEN register */
        .direction= AAI_RX,
        .intflag  = AAI_INT_EXT_RX,
        .enflag   = AAI_FE_EXT_RX,
        .fifo[0]  = { .hfifo    = _AAI_EXT_IN,},
    },
};


/**
 * @param ipcm channel ident currently
 */
static inline int spk_is_44_48(uint32_t iten){
   return ( (iten&AAI_INT_SPK)
            &&(AAI_FE_SPK_FREQ(iten)>=AAI_STREAM_MODE_44_48KHZ_16W));
}

static inline int mic12_is_44_48(uint32_t iten){
   return ( (iten&AAI_INT_MIC12)
            &&(AAI_FE_MIC12_FREQ(iten)>=AAI_STREAM_MODE_44_48KHZ_16W) );
}

static inline int mic34_is_44_48(uint32_t iten){
   return ( (iten&AAI_INT_MIC34)
            &&(AAI_FE_MIC34_FREQ(iten)>=AAI_STREAM_MODE_44_48KHZ_16W));
}

static inline int chan_is_mult(aai_device_t *chan){
   return ( (chan->ipcm == AAI_CHANNEL_SPK)    ||
            (chan->ipcm == AAI_CHANNEL_MIC12) ||
            (chan->ipcm == AAI_CHANNEL_MIC34) );
}

static inline int chan_is_bt(aai_device_t *chan){
   return 0;
}



/******************************************************************************
 *
 * Public functions
 *
 *****************************************************************************/

/** Print AAI Registers
 *
 */
void aai_hw_status(struct card_data_t *aai)
{
   aai_print(aai->dev,"     AAI_ITEN : 0x%08x\n", readl(aai->iobase+_AAI_FE ));
   aai_print(aai->dev,"     AAI_CFG  : 0x%08x\n", readl(aai->iobase+_AAI_CFG));
   aai_print(aai->dev,"     AAI_INT  : 0x%08x\n", readl(aai->iobase+_AAI_INT));
   aai_print(aai->dev,"     AAI_FLATT: 0x%08x\n", readl(aai->iobase+_AAI_FLATT));
   aai_print(aai->dev,"     AAI_FRATT: 0x%08x\n", readl(aai->iobase+_AAI_FRATT));
}

#if 0
/** Print AAI Registers
 *
 */
void aai_hw_status(struct card_data_t *aai)
{
   int i=0;
   while (vaai[i].addr!=0){
      aai_dbg("      %s : 0x%08x\n", vaai[i].name, aai_readreg(aai,vaai[i].addr ));
      i++;
   }
}
#endif

aai_device_t *aai_hw_get_pchans( void )
{
   return aai_channels;
}

/** Initialize AAI driver and hardware.
 * This function set defaults values for AAI_CFG register.<br>
 * On P5 hardware, init value of AAI_CFG is 0xF401, following bits are enabled:
 * <p>
 * On P5+ hardware, init value of AAI_CFG is 0x3401, following bits are
 * enabled:
 * <ul>
 *    <li>@ref AAI_CFG_COMPACT    : Set default sampling frequency (44.1 kHz)
 *    <li>@ref AAI_CFG_RUN_MULT   : Multimedia state running
 *    <li>@ref AAI_CFG_RUN_BT     : Bluetooth state running
 *    <li>@ref AAI_CFG_RUN_EXT    : External state running
 *    <li>@ref AAI_CFG_MIC34_PAD  : External Mic 3&4 pad use
 * </ul>
 */
int aai_hw_init(struct card_data_t *drv_data)
{
   uint32_t cfg = AAI_CFG_COMPACT     |
                  AAI_CFG_RUN_MULT    |
                  AAI_CFG_RUN_BT      |
                  AAI_CFG_RUN_EXT     |
                  AAI_CFG_MIC34_PAD   |
                  AAI_CFG_RUN_DAC     |
                  AAI_CFG_MASTER_BLUE |
                  AAI_CFG_MASTER_EXT  |
                  AAI_CFG_RUN_ADC;

   // default PCM configuration
   writel(0x0, drv_data->iobase+_AAI_BT_PCM_SI1);
   writel(0x0, drv_data->iobase+_AAI_BT_PCM_SI2);
   writel(0x0, drv_data->iobase+_AAI_EXT_PCM_SI1);
   // disable all channels
   writel(0x0, drv_data->iobase+_AAI_FE);
   writel(cfg, drv_data->iobase+_AAI_CFG);

   return 0;
}


/** Start channel.
 *
 */
int aai_hw_start_channel(aai_device_t *chan)
{
   uint32_t reg;
   struct card_data_t *aai = chan->pdrvdata;

   if(chan->ipcm < AAI_NB_CHANNELS){
      spin_lock(aai->lock);
         reg = readl(aai->iobase+_AAI_FE );
         reg |= chan->enflag;
         writel(reg, aai->iobase+_AAI_FE);
      spin_unlock(aai->lock);
      return 0;
   }

   return -EINVAL;
}


/** Stop channel.
 *
 */
int aai_hw_stop_channel(aai_device_t *chan)
{
   uint32_t reg;
   struct card_data_t *aai = chan->pdrvdata;

   if(chan->ipcm < AAI_NB_CHANNELS){
      spin_lock(aai->lock);
         reg = readl(aai->iobase+_AAI_FE );
         reg &= ~(chan->enflag);
         writel( reg, aai->iobase+_AAI_FE);
      spin_unlock(aai->lock);
      return 0;
   }

   return -EINVAL;
}

/** Set channel rate.
 * Set channel rate if it's possible.
 *
 * @param chan channel
 * @param rate numerical value (8000,16000,44100,48000)
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
int aai_hw_set_btrate(aai_device_t *chan, int rate)
{
   uint32_t iten, mode;
   void __iomem *iobase = ((struct card_data_t*)chan->pdrvdata)->iobase;
   struct card_data_t *aai = chan->pdrvdata;

   /*------------------------------------------------
	 * Bluetooth channels
    *----------------------------------------------*/
   switch (rate){
      case 8000:  mode = 0;                break;
      case 16000: mode = AAI_CFG_BLUE_16K; break;
      default:    goto error;              break;
   }

   spin_lock(aai->lock);
   iten =  readl(iobase+_AAI_FE );
   iten &= ~(AAI_CFG_BLUE_16K);/* reset channel config */
   iten |= mode;               /* write new config */
   writel( iten, iobase+_AAI_FE);
   spin_unlock(aai->lock);
   return 0;

error:
   spin_unlock(aai->lock);
   return -EINVAL;
}


/** Set channel rate.
 * Set channel rate if it's possible.
 *
 * @param chan channel
 * @param rate numerical value (8000,16000,44100,48000)
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
int aai_hw_set_multrate(aai_device_t *chan, int rate)
{
   uint32_t iten, cfg, mode;
   void __iomem *iobase = ((struct card_data_t*)chan->pdrvdata)->iobase;
   struct card_data_t *aai = chan->pdrvdata;

   spin_lock(aai->lock);
   iten =  readl(iobase+_AAI_FE );

   /*------------------------------------------------
	 * Multimedia channels
    *----------------------------------------------*/
   switch (rate)
   {
   case 8000:
      mode = AAI_STREAM_MODE_8KHZ_16W << chan->cfgshift;
      break;
   case 16000:
      mode = AAI_STREAM_MODE_16KHZ_32W << chan->cfgshift;
      break;
   case 44100:
      mode = AAI_STREAM_MODE_44_48KHZ_32W << chan->cfgshift;
      cfg =  readl(iobase+_AAI_CFG );
      if((cfg&0x1)==0){
         /* AAI is configured in 48000Hz */
         if( (chan->ipcm != AAI_CHANNEL_SPK) && spk_is_44_48(iten) ){
            goto out;
         }
         if( (chan->ipcm != AAI_CHANNEL_MIC12) && mic12_is_44_48(iten) ){
            goto out;
         }
         if( (chan->ipcm != AAI_CHANNEL_MIC34) && mic34_is_44_48(iten) ){
            goto out;
         }
         writel( (cfg | 0x1 ), iobase+_AAI_CFG);
      }
      break;
   case 48000:
      mode = AAI_STREAM_MODE_44_48KHZ_32W << chan->cfgshift;
      cfg =  readl(iobase+_AAI_CFG );
      if((cfg&0x1)==1){
         /* AAI is configured in 48000Hz */
         if( (chan->ipcm != AAI_CHANNEL_SPK) && spk_is_44_48(iten) ){
            goto out;
         }
         if( (chan->ipcm != AAI_CHANNEL_MIC12) && mic12_is_44_48(iten) ){
            goto out;
         }
         if( (chan->ipcm != AAI_CHANNEL_MIC34) && mic34_is_44_48(iten) ){
            goto out;
         }
         writel( (cfg & ~(0x1)), iobase+_AAI_CFG);
      }
      break;
   default:
      goto out;
      break;
   }

   iten =  readl(iobase+_AAI_FE );
   /* reset channel config */
   iten &= ~(AAI_STREAM_MODE_44_48KHZ_32W << chan->cfgshift);
   iten |= mode;
   /* write new config */
   writel( iten, iobase+_AAI_FE);

   spin_unlock(aai->lock);
   return 0;

out:
   spin_unlock(aai->lock);
   return -EINVAL;
}


int aai_hw_set_channelrate(aai_device_t *chan, int rate)
{
   int err = 0;
   struct card_data_t *aai = chan->pdrvdata;

   if(chan_is_mult(chan))
      err = aai_hw_set_multrate(chan,rate);
   else if(chan_is_bt(chan))
      err = aai_hw_set_btrate(chan,rate);

   //aai_dbg("%s(%d) - Setting channel to %d Hz\n", chan->name, chan->ipcm, rate);
   //aai_hw_status(iobase);

   if (err != 0)
      aai_err(aai->dev,"%s(%d) - Setting channel to %d Hz failed\n", chan->name, chan->ipcm, rate);

   return err;
}




/** Get SPK volume.
 *
 */
int aai_hw_getvolume(aai_device_t *chan)
{
   int err=0;

   if(chan->ipcm == AAI_CHANNEL_SPK){
      struct card_data_t *aai = chan->pdrvdata;

      spin_lock(aai->lock);
      chan->volume[0] = readl(aai->iobase+_AAI_FLATT);
      chan->volume[1] = readl(aai->iobase+_AAI_FRATT);
      spin_unlock(aai->lock);
      err = 0;
   }else{
      err = -EINVAL;
   }

   return err;
}

/** Set SPK volume.
 *
 */
int aai_hw_setvolume(aai_device_t *chan)
{
   int err=0;

   if(chan->ipcm == AAI_CHANNEL_SPK){
      struct card_data_t *aai = chan->pdrvdata;

      spin_lock(aai->lock);
      writel( chan->volume[0], aai->iobase+_AAI_FLATT);
      writel( chan->volume[1], aai->iobase+_AAI_FRATT);
      spin_unlock(aai->lock);
      err = 0;
   }else{
      err = -EINVAL;
   }

   return err;
}





