/**
 * @file   aai_mixer.c
 * @brief  AAI mixer handling
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

#include "aai.h"
#include "aai_hw.h"
#include "aai_mixer.h"

#define AAI_DB_MIN         (-63)
#define AAI_DB_MAX_VOICE   (0)
#define AAI_DB_MAX_MUSIC   (15)
/* BUG:
 * Due to a bug in the AAI, AAI_DB_MAX_MUSIC is not 17
 * When writting a value >= 0x40 in volume registers, it will set the volume to 0x47
 * so to avoid volume gaps of about 2 dB, maximum volume level is the last
 * correct value : 0x3c -> 15 dB.
 * Moreover, it seems that odd values can trigger the same bug.
 */

/*
 * Info volume channel value(s)
 */
static int chan_volume_info(struct snd_kcontrol      *kcontrol,
                            struct snd_ctl_elem_info *uinfo,
                            int                      nbchans,
                            int                      max)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = nbchans;                 /* stream number */
    uinfo->value.integer.min = AAI_DB_MIN;  /* min value */
    uinfo->value.integer.max = max;         /* max value */

    return 0;
}

int chan_volume_info_stereo(struct snd_kcontrol      *kcontrol,
                            struct snd_ctl_elem_info *uinfo)
{
    return chan_volume_info(kcontrol, uinfo, STEREO_STREAM, AAI_DB_MAX_MUSIC);
}

int chan_volume_info_quad(struct snd_kcontrol      *kcontrol,
                          struct snd_ctl_elem_info *uinfo)
{
    return chan_volume_info(kcontrol, uinfo, QUAD_STREAM, AAI_DB_MAX_VOICE);
}

/*
 * Get volume channel value(s)
 */
static int chan_volume_get(struct snd_kcontrol       *kcontrol,
                           struct snd_ctl_elem_value *ucontrol,
                           int                       nbchans)
{
    struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
    int addr = kcontrol->private_value;
    aai_device_t *chan = &aai->chans[addr];
    int ichan;
    int err = 0;

    spin_lock_irq(&aai->mixer_lock);
    err = aai->spec_ops->get_volume(chan);

    /*
     * changing value sign because aai is talking about
     * attenuation while alsa is talking about gain
     */
    for(ichan = 0; ichan < nbchans; ichan++)
    {
        ucontrol->value.integer.value[ichan] = chan->volume[ichan];
    }

    spin_unlock_irq(&aai->mixer_lock);
    return err;
}

int chan_volume_get_stereo(struct snd_kcontrol       *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
    return chan_volume_get(kcontrol, ucontrol, STEREO_STREAM);
}

int chan_volume_get_quad(struct snd_kcontrol       *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
    return chan_volume_get(kcontrol, ucontrol, QUAD_STREAM);
}

/*
 * Put volume channel value(s)
 */
static int chan_volume_put(struct snd_kcontrol       *kcontrol,
                           struct snd_ctl_elem_value *ucontrol,
                           int                       nbchans,
                           int                       max)
{
    int ichan, err, volume[QUAD_STREAM];
    struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
    int change = 0, ipcm = kcontrol->private_value;
    aai_device_t *chan = &(aai->chans[ipcm]);

    for (ichan = 0; ichan < nbchans; ichan++)
    {
        volume[ichan] = ucontrol->value.integer.value[ichan];

        if (volume[ichan] < AAI_DB_MIN)
        {
            volume[ichan] = AAI_DB_MIN;
        }
        else if (volume[ichan] > max)
        {
            volume[ichan] = max;
        }
    }

    spin_lock_irq(&aai->mixer_lock);

    /*
     * check if values need to be changed
     */
    for (ichan = 0; ichan < nbchans; ichan++)
    {
        if (chan->volume[ichan] != volume[ichan])
        {
            change = 1;
            break;
        }
    }

    /*
     * if values changed, memorize them and set them
     */
    if (change)
    {
        for (ichan = 0; ichan < nbchans; ichan++)
        {
            chan->volume[ichan] = volume[ichan];
        }
        err = aai->spec_ops->set_volume(chan);
        if (err != 0)
            change = 0;
    }

    spin_unlock_irq(&aai->mixer_lock);
    return change;
}

/*
 * Put volume channel value(s) for linked left/right channels
 */
static int chan_volume_put_linked(struct snd_kcontrol       *kcontrol,
                                  struct snd_ctl_elem_value *ucontrol,
                                  int                       nbchans,
                                  int                       max)
{
    int ichan, err, volume[QUAD_STREAM];
    struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
    int change = 0, ipcm = kcontrol->private_value;
    aai_device_t *chan = &(aai->chans[ipcm]);

    for (ichan = 0; ichan < nbchans; ichan++)
    {
        volume[ichan] = ucontrol->value.integer.value[ichan];

        if (volume[ichan] < AAI_DB_MIN)
        {
            volume[ichan] = AAI_DB_MIN;
        }
        else if (volume[ichan] > max)
        {
            volume[ichan] = max;
        }
    }

    spin_lock_irq(&aai->mixer_lock);

    /*
     * check if values need to be changed
     */
    for (ichan = 0; ichan < nbchans; ichan++)
    {
        if (chan->volume[ichan] != volume[ichan])
        {
            change = 1;
            volume[ichan^1] = volume[ichan];
            break;
        }
    }

    /*
     * if values changed, memorize them and set them for both left and righ channels
     */
    if (change)
    {
        for (ichan = 0; ichan < nbchans; ichan++)
        {
            chan->volume[ichan] = volume[ichan];
            chan->volume[ichan^1] = volume[ichan];
        }
        err = aai->spec_ops->set_volume(chan);
        if (err != 0)
            change = 0;
    }

    /*
     * check again set values to update ucontrol
     * (some channels may have linked left and right routes)
     */
    for (ichan = 0; ichan < nbchans; ichan++)
    {
        ucontrol->value.integer.value[ichan] = chan->volume[ichan];
    }

    spin_unlock_irq(&aai->mixer_lock);
    return change;
}

int chan_volume_put_stereo(struct snd_kcontrol *kcontrol,
                           struct snd_ctl_elem_value *ucontrol)
{
    return chan_volume_put(kcontrol, ucontrol, STEREO_STREAM, AAI_DB_MAX_MUSIC);
}

int chan_volume_put_quad(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
    return chan_volume_put(kcontrol, ucontrol, QUAD_STREAM, AAI_DB_MAX_VOICE);
}

int chan_volume_put_quad_linked(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
    return chan_volume_put_linked(kcontrol, ucontrol, QUAD_STREAM, AAI_DB_MAX_VOICE);
}

/*---------------------------------------------------------------------*
 * Info mute channel value(s)
 *---------------------------------------------------------------------*/
/**
 * Info mute channel value(s)
 */
static int chan_mute_info(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_info *uinfo,
                          int nbchans)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = nbchans;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}

/**
 * Info mute mono value
 */
int chan_mute_info_quad(struct snd_kcontrol *kcontrol,
                        struct snd_ctl_elem_info *uinfo)
{
    return chan_mute_info(kcontrol, uinfo, QUAD_STREAM);
}

/**
 * Info mute stereo values
 */
int chan_mute_info_stereo(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_info *uinfo)
{
    return chan_mute_info(kcontrol, uinfo, STEREO_STREAM);
}

/*---------------------------------------------------------------------*
 * Get mute channel value(s)
 *---------------------------------------------------------------------*/
/**
 * Get mute channel value(s)
 */
static int chan_mute_get(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol,
                         int nbchans)
{
    struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
    int err __maybe_unused, ichan, ipcm = kcontrol->private_value;
    aai_device_t *chan = &aai->chans[ipcm];

    spin_lock(&aai->mixer_lock);
    err = aai->spec_ops->get_volume(chan);
    /*
     * if mute = 1 mute func is ON whereas alsa mixer
     * put 1 when mute func is OFF
     */
    for (ichan = 0; ichan < nbchans; ichan++)
    {
        ucontrol->value.integer.value[ichan] = !chan->mute[ichan];
    }
    spin_unlock(&aai->mixer_lock);
    return 0;
}

int chan_mute_get_quad(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
    return chan_mute_get(kcontrol, ucontrol, QUAD_STREAM);
}

int chan_mute_get_stereo(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
    return chan_mute_get(kcontrol, ucontrol, STEREO_STREAM);
}

/*---------------------------------------------------------------------*
 * Put mute channel value(s)
 *---------------------------------------------------------------------*/
/**
 * Put mute channel value(s)
 */
static int chan_mute_put(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol,
                         int nbchans)
{
    int err, mute, ichan;
    struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
    int change = 0, ipcm = kcontrol->private_value;
    aai_device_t *chan = &(aai->chans[ipcm]);

    /*
     * if mute = 1 mute func is ON whereas alsa mixer
     * put 1 when mute func is OFF
     */
    spin_lock(&aai->mixer_lock);
    for (ichan = 0; ichan < nbchans; ichan++)
    {
        mute = !ucontrol->value.integer.value[ichan];
        if(mute != chan->mute[ichan])
        {
            chan->mute[ichan] = mute;
            change = 1;
        }
    }

    if (change)
    {
        err = aai->spec_ops->set_volume(chan);
        if (err != 0)
        {
            change = 0;
        }
    }

    spin_unlock(&aai->mixer_lock);
    return change;
}

static int chan_mute_put_linked(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol,
                                int nbchans)
{
    int err, mute, ichan;
    struct card_data_t *aai = snd_kcontrol_chip(kcontrol);
    int change = 0, ipcm = kcontrol->private_value;
    aai_device_t *chan = &(aai->chans[ipcm]);

    /*
     * if mute = 1 mute func is ON whereas alsa mixer
     * put 1 when mute func is OFF
     */
    spin_lock(&aai->mixer_lock);
    for (ichan = 0; ichan < nbchans; ichan++)
    {
        mute = !ucontrol->value.integer.value[ichan];

        if(mute != chan->mute[ichan])
        {
            chan->mute[ichan] = mute;
            chan->mute[ichan^1] = mute;
            change = 1;
        }
    }

    if (change)
    {
        err = aai->spec_ops->set_volume(chan);
        if (err != 0)
            change = 0;
    }

    /*
     * check again set values to update ucontrol
     * (some channels may have linked left and right routes)
     */
    for (ichan = 0; ichan < nbchans; ichan++)
    {
        ucontrol->value.integer.value[ichan] = !chan->mute[ichan];
    }

    spin_unlock(&aai->mixer_lock);
    return change;
}

int chan_mute_put_quad_linked(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
    return chan_mute_put_linked(kcontrol,ucontrol,QUAD_STREAM);
}

int chan_mute_put_quad(struct snd_kcontrol *kcontrol,
                       struct snd_ctl_elem_value *ucontrol)
{
    return chan_mute_put(kcontrol,ucontrol,QUAD_STREAM);
}

int chan_mute_put_stereo(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
    return chan_mute_put(kcontrol,ucontrol,STEREO_STREAM);
}

