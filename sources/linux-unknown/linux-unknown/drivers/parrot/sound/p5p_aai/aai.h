#ifndef INCONCE_AAI_H
#define INCONCE_AAI_H


/*
 * linux/drivers/io/aai.h
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
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/moduleparam.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include <sound/hwdep.h>


#define aai_printk(dev,...) /*printk("[  ] AAI "__VA_ARGS__)*/
#define aai_warn(dev,...)   dev_warn(dev,"[WW] "__VA_ARGS__)
#define aai_info(dev,...)   dev_info(dev,"[II] "__VA_ARGS__)
#define aai_print(dev,...)  dev_info(dev,__VA_ARGS__)
#define aai_dbg(dev,...)    dev_dbg(dev,"[  ] "__VA_ARGS__)

#define aai_err(dev,...)    { \
   dev_err(dev, "[EE]"__VA_ARGS__);  \
   dev_err(dev, "\tf:%s, l:%d\n",__FILE__, __LINE__);  \
};

/*
#define aai_printk(...) printk("[  ] AAI "__VA_ARGS__)
#define aai_warn(...)   printk("[WW] AAI "__VA_ARGS__)
#define aai_info(...)   printk("[II] AAI "__VA_ARGS__)
#define aai_print(...)  printk("\t"__VA_ARGS__)
#define aai_dbg(...)    printk("[  ] AAI "__VA_ARGS__)

#define aai_err(...)    { \
   printk("[EE] AAI "__VA_ARGS__);  \
   printk("[EE] AAI f:%s, l:%d\n",__FILE__, __LINE__);  \
};
*/



#define MONO_STREAM		1
#define STEREO_STREAM	2


//! Audio channels identifiers
typedef enum {
    AAI_CHANNEL_SPK = 0,   //!< Loudspeaker output
    AAI_CHANNEL_MIC12,     //!< Micro 1 & 2 input
    AAI_CHANNEL_MIC34,     //!< Micro 3 & 4 input
    AAI_CHANNEL_BT_OUT,    //!< Bluetooth output
    AAI_CHANNEL_BT_IN,     //!< Bluetooth input
    AAI_CHANNEL_EXT_OUT,   //!< External/GSM output
    AAI_CHANNEL_EXT_IN,    //!< External/GSM input
    AAI_NB_CHANNELS
} aai_pcm_ident_t;
#define LAST_PCM  AAI_NB_CHANNELS


typedef enum {
    AAI_RX   = 0x1,
    AAI_TX   = 0x2,
    AAI_RXTX = 0x3,
} aai_dir_t;


/**
 * AAI pcm substream
 */
typedef struct{
	int 			         mute;
   uint32_t             hfifo;      //!< HW fifo in for this channel
   uint32_t             bufsize;    //!<
   char                 *pstart;    //!< Ptr to start of transfer buffer
   char                 *pcurrent;  //!< Ptr to next chunk to transfer
   char                 *pend;      //!< Ptr to end of transfer buffer

}  aai_hwchan_t;


/**
 * AAI pcm stream
 */
typedef struct _aai_device_t {
   /* Misc informations */
   void        			*pdrvdata;  //!< Ptr to driver data
   const char  			name[16];   //!< Device name ("spk", "mic12", etc)
   aai_pcm_ident_t 	   ipcm; 		//!< pcm identifier number
   int 	   				nbchans;		//!< nbr of chans (mono, stereo, ...)
	aai_dir_t				direction;	//!< RX/TX
   struct snd_pcm_ops   *ops;       //!< PCM callbacks operators
   /* Hardware informations */
   uint32_t             intflag;    //!< HW interrupt flag for this channel
   uint32_t             enflag;     //!< HW enable flag for this channel
   aai_hwchan_t   		fifo[2];    //!< HW fifo informations
   uint32_t       		cfgshift;   //!< Rate/fifo depth config shift(AAI_CFG regsister)
   uint32_t       		rate;       //!< Rate value
   uint32_t       		format;     //!< Sample format (bytes)
   uint32_t       		period_frames; //!< alsa period size in frame
   uint32_t       		period_bytes;  //!< alsa period size in bytes
   uint32_t       		bytes_count;   //!< amount of bytes read or written
   int            		volume[2];  //!< Channel vol [-63dB;+18 dB]
} aai_device_t;



struct card_data_t {
   /* alsa structures      */
	struct snd_card		*card;
	struct snd_pcm			*pcms[AAI_NB_CHANNELS];
   aai_device_t       	*chans;
   uint32_t             deven;
	int			        	irq;
	//struct mutex		   sem;
   /* memory ressource     */
	void __iomem        	*iobase;
	struct resource		*ioarea;
	struct device			*dev;
	/* hardware dep         */
	struct snd_hwdep		*hwdep;

	spinlock_t 				mixer_lock;
	spinlock_t 				lock;
};


/*---------------------------------------------------------------------------*/
/* Exported prototypes of aai_alsa.c                                         */
/*---------------------------------------------------------------------------*/
extern struct card_data_t * get_datafromcard(struct snd_card *card);
extern struct snd_card * __devinit aai_alsa_probe(struct platform_device *pdev);

extern int __devinit aai_alsa_init_pcm(struct card_data_t *drv_data);
extern int __devexit aai_alsa_remove(struct snd_card *card);

/*---------------------------------------------------------------------------*/
/* Exported prototypes of aai_alsa_ops.c                                     */
/*---------------------------------------------------------------------------*/
extern struct snd_pcm_ops snd_pcm_multcap_ops;
extern struct snd_pcm_ops snd_pcm_multplay_ops;
extern struct snd_pcm_ops snd_pcm_btcap_ops;
extern struct snd_pcm_ops snd_pcm_btplay_ops;


/*---------------------------------------------------------------------------*/
/* Inlined functions                                                         */
/*---------------------------------------------------------------------------*/


/** Get pointer to playback substream
 * We consider that there is only one substream per pcm
 *
 * @param aai private driver data
 * @param ipcm PCM index
 * @return pointer to capture substream
 */
static inline struct snd_pcm_substream * chan2pbacksubstream(struct card_data_t * aai, int ipcm)
{
   return (struct snd_pcm_substream*)((struct snd_pcm*)(aai->pcms)[ipcm]->streams[0].substream);
}

/** Get pointer to capture substream
 * We consider that there is only one substream per pcm
 *
 * @param aai private driver data
 * @param ipcm PCM index
 * @return pointer to capture substream
 */
static inline struct snd_pcm_substream * chan2captsubstream(struct card_data_t * aai, int ipcm)
{
   return (struct snd_pcm_substream*)((struct snd_pcm*)(aai->pcms)[ipcm]->streams[1].substream);
}

/** Get pointer to substream's aai-pcm data
 *
 * @param substream
 * @return pointer to aai-pcm data
 */
static inline aai_device_t* aai_pchan_from_substream(struct snd_pcm_substream * substream)
{
   return (aai_device_t*)(substream->pcm->private_data);
}

#define chan2dev(_chan_) (((struct card_data_t*)chan->pdrvdata)->dev)



#endif //INCONCE_AAI_H
