/*
 * linux/drivers/io/aai_mixer.c
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

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>


#include "aai.h"
#include "aai_hw.h"


#define AAI_DB_MIN (-63)
#define AAI_DB_MAX (18)

static int chan_volume_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = STEREO_STREAM;                      /* stream number : stereo */
	uinfo->value.integer.min = AAI_DB_MIN; /* min value */
	uinfo->value.integer.max = AAI_DB_MAX; /* max value */
	return 0;
}

static int chan_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	aai_device_t *chan = &aai->chans[addr];
   int err = 0;

	spin_lock_irq(&aai->mixer_lock);
   err = aai_hw_getvolume(chan);

   /* changing value sign because aai is talking about
    * attenuation while alsa is talking about gain */
   chan->volume[0] = -chan->volume[0];
   chan->volume[1] = -chan->volume[1];

	ucontrol->value.integer.value[0] = chan->volume[0];
	ucontrol->value.integer.value[1] = chan->volume[1];
	spin_unlock_irq(&aai->mixer_lock);
	return err;
}

static int chan_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int err, left, right;
	struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
	int change, ipcm = kcontrol->private_value;
	aai_device_t *chan = &(aai->chans[ipcm]);

	left = ucontrol->value.integer.value[0];
	if (left < AAI_DB_MIN)   left = AAI_DB_MIN;
	if (left > AAI_DB_MAX)   left = AAI_DB_MAX;
	right = ucontrol->value.integer.value[1];
	if (right < AAI_DB_MIN)  right = AAI_DB_MIN;
	if (right > AAI_DB_MAX)  right = AAI_DB_MAX;

   /* changing value sign because aai is talking about
    * attenuation while alsa is talking about gain */
   left = -left;
   right = -right;

	spin_lock_irq(&aai->mixer_lock);

	change = chan->volume[0] != left ||
	         chan->volume[1] != right;

   if(change){
	   chan->volume[0] = left;
	   chan->volume[1] = right;
      err = aai_hw_setvolume(chan);
      if(err!=0) change = 0;
   }

	spin_unlock_irq(&aai->mixer_lock);
	return change;
}

/*---------------------------------------------------------------------*
 * Info mute channel value(s)
 *---------------------------------------------------------------------*/
/** Info mute channel value(s)
 *
 */
static int chan_mute_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo, int nbchans)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = nbchans;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}
/** Info mute mono value
 *
 */
static int chan_mute_info_mono(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	return chan_mute_info(kcontrol,uinfo,MONO_STREAM);
}

/** Info mute stereo values
 *
 */
static int chan_mute_info_stereo(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	return chan_mute_info(kcontrol,uinfo,STEREO_STREAM);
}


/*---------------------------------------------------------------------*
 * Get mute channel value(s)
 *---------------------------------------------------------------------*/
/** Get mute channel value(s)
 *
 */
static int chan_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol, int nbchans)
{
	struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
	int ichan, ipcm = kcontrol->private_value;
	aai_device_t *chan = &aai->chans[ipcm];

	spin_lock(&aai->mixer_lock);
   /* if mute = 1 mute func is ON whereas alsa mixer
    * put 1 when mute func is OFF */
	for(ichan=0; ichan<nbchans; ichan++){
		ucontrol->value.integer.value[ichan] = !chan->fifo[ichan].mute;
	}
	spin_unlock(&aai->mixer_lock);
	return 0;
}
/** Get mute mono value
 *
 */
static int chan_mute_get_mono(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return chan_mute_get(kcontrol,ucontrol,MONO_STREAM);
}

/** Get mute stereo values
 *
 */
static int chan_mute_get_stereo(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return chan_mute_get(kcontrol,ucontrol,STEREO_STREAM);
}


/*---------------------------------------------------------------------*
 * Put mute channel value(s)
 *---------------------------------------------------------------------*/
/** Put mute channel value(s)
 *
 */
static int chan_mute_put(struct snd_kcontrol *kcontrol,
				             struct snd_ctl_elem_value *ucontrol,
                         int nbchans)
{
	int mute, ichan;
	struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
	int change = 0, ipcm = kcontrol->private_value;
	aai_device_t *chan = &(aai->chans[ipcm]);

   /* if mute = 1 mute func is ON whereas alsa mixer
    * put 1 when mute func is OFF */
	spin_lock(&aai->mixer_lock);
	for(ichan=0; ichan<nbchans; ichan++){
   	mute = !ucontrol->value.integer.value[ichan];
		if(mute != chan->fifo[ichan].mute){
			chan->fifo[ichan].mute = mute;
			change = 1;
		}
	}
	spin_unlock(&aai->mixer_lock);
	return change;
}

/** Put mute mono value
 *
 */
static int chan_mute_put_mono(struct snd_kcontrol *kcontrol,
				                  struct snd_ctl_elem_value *ucontrol)
{
	return chan_mute_put(kcontrol,ucontrol,MONO_STREAM);
}

/** Put mute stereo values
 *
 */
static int chan_mute_put_stereo(struct snd_kcontrol *kcontrol,
				                    struct snd_ctl_elem_value *ucontrol)
{
	return chan_mute_put(kcontrol,ucontrol,STEREO_STREAM);
}



/*---------------------------------------------------------------------*
 * Define controls
 *---------------------------------------------------------------------*/

#define CTL_SWITCH_STEREO(_name_, _ipcm_) \
{	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER, \
   .name   = _name_, \
   .index  = _ipcm_, \
   .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
   .info   = chan_mute_info_stereo, \
   .get    = chan_mute_get_stereo, \
   .put    = chan_mute_put_stereo, \
   .private_value = _ipcm_, \
}

#define CTL_SWITCH_MONO(_name_, _ipcm_) \
{	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER, \
   .name   = _name_, \
   .index  = _ipcm_, \
   .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
   .info   = chan_mute_info_mono, \
   .get    = chan_mute_get_mono, \
   .put    = chan_mute_put_mono, \
   .private_value = _ipcm_, \
}

#define CTL_VOLUME(_name_, _ipcm_) \
{	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER, \
   .name   = _name_, \
   .index  = _ipcm_, \
   .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
   .info   = chan_volume_info, \
   .get    = chan_volume_get, \
   .put    = chan_volume_put, \
   .private_value = _ipcm_, \
   .tlv.p = db_scale_my_control, \
}

/*step size, in units of 0.01 dB*/
static DECLARE_TLV_DB_SCALE(db_scale_my_control, -6300, 1800, 0);


static struct snd_kcontrol_new aai_alsa_controls[] = {
	CTL_VOLUME("Speaker Playback Volume",AAI_CHANNEL_SPK),
	CTL_SWITCH_MONO("Speaker Playback Switch",AAI_CHANNEL_SPK),
};



/** Create a Mixer
 *
 */
int aai_mixer_new(struct card_data_t *aai)
{
	struct snd_card *card = aai->card;
	unsigned int idx;
	int err;

	snd_assert(aai != NULL, return -EINVAL);
	spin_lock_init(&aai->mixer_lock);
	strcpy(card->mixername, "AAI Mixer");

	for (idx = 0; idx < ARRAY_SIZE(aai_alsa_controls); idx++) {
      err = snd_ctl_add(card, snd_ctl_new1(&aai_alsa_controls[idx], aai));
		if (err < 0){
         aai_err(aai->dev,"... adding control failed \n");
			return err;
      }
	}

	return 0;
}




