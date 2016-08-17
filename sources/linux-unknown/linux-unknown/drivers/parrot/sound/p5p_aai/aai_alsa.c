/**
 * @file   aai_alsa.c
 * @brief  AAI alsa layer
 *
 * @author gregoire.etienne@parrot.com
 * @date   2008-10-02
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
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "aai.h"
#include "aai_hw.h"


extern int __init aai_mixer_new	      (struct card_data_t *aai);
extern int __init aai_ioctl_hwdep_new	(struct card_data_t *aai, char * id, int device);

static int __init aai_alsa_pcm_new	(struct device *dev, struct card_data_t *aai, int ipcm);

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;	// Index 0-MAX
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;	// ID for this card

/******************************************************************************
 *
 * Public functions
 *
 *****************************************************************************/

/** Create an ALSA card instance
 *
 */
struct snd_card * __devinit aai_alsa_probe(struct platform_device *pdev)
{
	struct snd_card *card;
	struct card_data_t *aai;
	int idx, err;
	int dev = pdev->id;

   /*------------------------------------------------
    * Initialize Card
    *----------------------------------------------*/
	card = snd_card_new( index[dev] , id[dev],
                        THIS_MODULE, sizeof(struct card_data_t));

	if (card == NULL){
      aai_err(&pdev->dev,"card creation failed \n");
		return NULL;
   }

	aai = card->private_data;
	aai->card = card;
	aai->dev = &pdev->dev;

   /*------------------------------------------------
	 * AAI Hardware Init
    *----------------------------------------------*/
   aai->chans = aai_hw_get_pchans( );

   /*------------------------------------------------
    * Initialize PCM streams
    *----------------------------------------------*/
	for (idx = 0; idx < LAST_PCM ; idx++)
   {
		if ((err = aai_alsa_pcm_new(&pdev->dev,aai, idx )) < 0){
         aai_err(&pdev->dev,"%s pcm %d init failed\n", __FUNCTION__, idx);
			goto __nodev;
      }
	}
   aai_dbg(&pdev->dev,"pcm streams initialization ok - %d streams\n", idx);

   /*------------------------------------------------
    * Initialize Mixer
    *----------------------------------------------*/
   err = aai_mixer_new(aai);
	if (err < 0){
      aai_err(&pdev->dev,"%s mixer init failed \n", __FUNCTION__);
		goto __nodev;
   }
   aai_dbg(&pdev->dev,"mixer initialization ok\n");

   /*------------------------------------------------
    * Set Sound Card datas
    *----------------------------------------------*/
	strcpy(card->driver, "AAI P5p");
	strcpy(card->shortname, "AAI");
	sprintf(card->longname, "P5P Advanced Audio Interface %i", dev + 1);

	aai_ioctl_hwdep_new(aai, "AAIhwdep", 0);

	snd_card_set_dev(card, &pdev->dev);
   err = snd_card_register(card);
	if (err != 0) {
      aai_err(&pdev->dev,"card registration failed: err=%d\n", err);
		goto __nodev;
   }

   aai_info(&pdev->dev,"%s card initialized\n",card->longname);
   return card;

__nodev:
   aai_err(&pdev->dev,"AAI card initialization failed \n");
	snd_card_free(card);
	return NULL;
}


/******************************************************************************
 *
 * Private functions
 *
 *****************************************************************************/

/** Create a PCM stream
 *
 */
static int aai_alsa_pcm_new(struct device *dev, struct card_data_t  *aai, int ipcm)
{
	int err;
   aai_device_t * chan = NULL;

   /* Link PCM stream to AAI channel */
   chan = &(aai->chans[ipcm]);
   chan->pdrvdata = aai;

   /*------------------------------------------------
    * Install device
    *----------------------------------------------*/
	err = snd_pcm_new(aai->card, &(chan->name[0]),
                     (char)ipcm,                       	/* device index */
                     ((chan->direction&AAI_TX)==AAI_TX),/* number of substreams for playback */
                     ((chan->direction&AAI_RX)==AAI_RX),/* number of substreams for capture  */
                     &(aai->pcms[ipcm]) );   		/* the pointer to store the new pcm instance */

	if (err < 0){
      aai_err(dev,"%s init failed\n", chan->name);
		return err;
   }

	aai->pcms[ipcm]->private_data = chan;
	aai->pcms[ipcm]->info_flags = 0;
	strcpy(aai->pcms[ipcm]->name, chan->name);

   /*------------------------------------------------
    * Set Operators
    *----------------------------------------------*/
   if(chan->direction&AAI_TX){
      snd_pcm_set_ops(aai->pcms[ipcm], SNDRV_PCM_STREAM_PLAYBACK, chan->ops);
   }else if(chan->direction&AAI_RX){
	   snd_pcm_set_ops(aai->pcms[ipcm], SNDRV_PCM_STREAM_CAPTURE, chan->ops);
   }else{
      aai_err(dev,"%s init failed\n", chan->name);
		return -EINVAL;
   }

	snd_pcm_lib_preallocate_pages_for_all( aai->pcms[ipcm], SNDRV_DMA_TYPE_CONTINUOUS,
					                           snd_dma_continuous_data(GFP_KERNEL),
					                           0, 64*1024);/* XXX where is it freed ? */

	return 0;
}



/** Remove Alsa
 *
 */
int __devexit aai_alsa_remove(struct snd_card *card)
{
	if (card) {
		snd_card_free(card);
   }

	return 0;
}


