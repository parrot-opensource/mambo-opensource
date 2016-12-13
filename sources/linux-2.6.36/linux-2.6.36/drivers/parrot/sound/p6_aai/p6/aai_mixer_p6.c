#include "aai.h"
#include "aai_hw.h"
#include "aai_hw_p6.h"
#include "aai_mixer.h"

/*
 * step size, in units of 0.01 dB
 */
static DECLARE_TLV_DB_SCALE(db_scale_my_control, -6300, 1800, 0);

static struct snd_kcontrol_new aai_alsa_controls[] =
{
    CTL_VOLUME_STEREO("spk-out0 Playback Volume", AAI_SPK_OUT0),
    CTL_SWITCH_STEREO("spk-out0 Playback Switch", AAI_SPK_OUT0),
    CTL_VOLUME_STEREO("spk-out1 Playback Volume", AAI_SPK_OUT1),
    CTL_SWITCH_STEREO("spk-out1 Playback Switch", AAI_SPK_OUT1),
    CTL_VOLUME_QUAD("spk-8khz Playback Volume",   AAI_SPK_8KHZ),
    CTL_SWITCH_QUAD("spk-8khz Playback Switch",   AAI_SPK_8KHZ),
    CTL_VOLUME_QUAD("spk-16khz Playback Volume",  AAI_SPK_16KHZ),
    CTL_SWITCH_QUAD("spk-16khz Playback Switch",  AAI_SPK_16KHZ),
    CTL_VOLUME_QUAD("spk-aux Playback Volume",    AAI_SPK_AUX),
    CTL_SWITCH_QUAD("spk-aux Playback Switch",    AAI_SPK_AUX)
};

/**
 * @brief Create a mixer
 *
 * @param aai  AAI card instance
 *
 * @return error code
 */
int aai_mixer_new_p6(struct card_data_t *aai)
{
    struct snd_card *card = aai->card;
    unsigned int idx;
    int err;

    //snd_assert(aai != NULL, return -EINVAL);
    spin_lock_init(&aai->mixer_lock);
    strcpy(card->mixername, "AAI Mixer");

    for (idx = 0; idx < ARRAY_SIZE(aai_alsa_controls); idx++)
    {
        err = snd_ctl_add(card, snd_ctl_new1(&aai_alsa_controls[idx], aai));
        if (err < 0)
        {
            aai_err(aai->dev, "... adding control failed \n");
            return err;
        }
    }

    return 0;
}

