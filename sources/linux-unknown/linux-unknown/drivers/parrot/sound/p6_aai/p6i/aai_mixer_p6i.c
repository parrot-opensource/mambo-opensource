#include "aai.h"
#include "aai_hw.h"
#include "aai_hw_p6i.h"
#include "aai_mixer.h"
#include "aai_codec_p6i.h"

#define CODEC_PGA_INPUT_MIN     (-12)
#define CODEC_PGA_INPUT_MAX     40
#define CODEC_RECORD_MIN        (-96)
#define CODEC_RECORD_MAX        30

enum
{
    CODEC_PGA_INPUT = AAI_SPK_16KHZ + 1,
    CODEC_RECORD,
    CODEC_PLAYBACK
};

static int chan_volume_info_codec(struct snd_kcontrol      *kcontrol,
                                  struct snd_ctl_elem_info *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 2;

    switch (kcontrol->private_value)
    {
        case CODEC_PGA_INPUT:
            uinfo->value.integer.min = CODEC_PGA_INPUT_MIN;
            uinfo->value.integer.max = CODEC_PGA_INPUT_MAX;
            break;

        case CODEC_RECORD:
            uinfo->value.integer.min = CODEC_RECORD_MIN;
            uinfo->value.integer.max = CODEC_RECORD_MAX;
            break;

        default:
            break;
    }

    return 0;
}

static int chan_volume_get_codec(struct snd_kcontrol       *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
    int left;
    int right;
    uint32_t reg;

    reg = readl(PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL2);

    switch (kcontrol->private_value)
    {
        case CODEC_PGA_INPUT:
            left  =  reg       & 0xff;
            right = (reg >> 8) & 0xff;
            ucontrol->value.integer.value[0] = left  + CODEC_PGA_INPUT_MIN;
            ucontrol->value.integer.value[1] = right + CODEC_PGA_INPUT_MIN;
            break;

        case CODEC_RECORD:
            left  = (reg >> 16) & 0xff;
            right = (reg >> 24) & 0xff;
            ucontrol->value.integer.value[0] = CODEC_RECORD_MAX - left  * 3 / 2;
            ucontrol->value.integer.value[1] = CODEC_RECORD_MAX - right * 3 / 2;
            break;

        default:
            break;
    }

    return 0;
}

static int chan_volume_put_codec(struct snd_kcontrol       *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
    int left;
    int right;
    uint32_t reg;

    reg = readl(PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL2);

    switch (kcontrol->private_value)
    {
        case CODEC_PGA_INPUT:
            left  = ucontrol->value.integer.value[0] - CODEC_PGA_INPUT_MIN;
            right = ucontrol->value.integer.value[1] - CODEC_PGA_INPUT_MIN;
            reg &= ~P6I_SYS_CODEC_MICVOL_MASK;
            reg |= ((right & 0xff) << 8) | (left & 0xff);
            break;

        case CODEC_RECORD:
            left  = (CODEC_RECORD_MAX - ucontrol->value.integer.value[0]) * 2 / 3;
            right = (CODEC_RECORD_MAX - ucontrol->value.integer.value[1]) * 2 / 3;
            reg &= ~P6I_SYS_CODEC_RECVOL_MASK;
            reg |= ((right & 0xff) << 24) | ((left & 0xff) << 16);
            break;

        default:
            break;
    }

    writel(reg, PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL2);
    aai_codec_latch(0);

    return 0;
}

static int chan_mute_info_codec(struct snd_kcontrol      *kcontrol,
                                struct snd_ctl_elem_info *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    uinfo->count = 2;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}

static int chan_mute_get_codec(struct snd_kcontrol       *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
    uint32_t reg;

    reg = readl(PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL3);

    switch (kcontrol->private_value)
    {
        case CODEC_PGA_INPUT:
            ucontrol->value.integer.value[0] = !(reg & (1 << 2));
            ucontrol->value.integer.value[1] = !(reg & (1 << 3));
            break;

        case CODEC_RECORD:
            ucontrol->value.integer.value[0] = !(reg & (1 << 4));
            ucontrol->value.integer.value[1] = !(reg & (1 << 5));
            break;

        case CODEC_PLAYBACK:
            ucontrol->value.integer.value[0] = !(reg & (1 << 0));
            ucontrol->value.integer.value[1] = !(reg & (1 << 1));
            break;

        default:
            break;
    }

    return 0;
}

static int chan_mute_put_codec(struct snd_kcontrol       *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
    uint32_t reg;

    reg = readl(PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL3);

    switch (kcontrol->private_value)
    {
        case CODEC_PGA_INPUT:
            reg &= ~P6I_SYS_CODEC_MICMUTE_MASK;
            reg |= !ucontrol->value.integer.value[0] << 2;
            reg |= !ucontrol->value.integer.value[1] << 3;
            break;

        case CODEC_RECORD:
            reg &= ~P6I_SYS_CODEC_RECMUTE_MASK;
            reg |= !ucontrol->value.integer.value[0] << 4;
            reg |= !ucontrol->value.integer.value[1] << 5;
            break;

        case CODEC_PLAYBACK:
            reg &= ~P6I_SYS_CODEC_LMMUTE_MASK;
            reg |= !ucontrol->value.integer.value[0] << 0;
            reg |= !ucontrol->value.integer.value[1] << 1;
            break;

        default:
            break;
    }

    writel(reg, PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL3);
    aai_codec_latch(0);

    return 0;
}

/*
 * step size, in units of 0.01 dB
 */
static DECLARE_TLV_DB_SCALE(db_scale_my_control, -6300, 1800, 0);

static struct snd_kcontrol_new aai_alsa_controls[] =
{
    CTL_VOLUME_STEREO("spk-out0 Playback Volume",       AAI_SPK_OUT0),
    CTL_SWITCH_STEREO("spk-out0 Playback Switch",       AAI_SPK_OUT0),
    CTL_VOLUME_STEREO("spk-out1 Playback Volume",       AAI_SPK_OUT1),
    CTL_SWITCH_STEREO("spk-out1 Playback Switch",       AAI_SPK_OUT1),
    CTL_VOLUME_QUAD_LINKED("spk-8khz Playback Volume",  AAI_SPK_8KHZ),
    CTL_SWITCH_QUAD_LINKED("spk-8khz Playback Switch",  AAI_SPK_8KHZ),
    CTL_VOLUME_QUAD_LINKED("spk-16khz Playback Volume", AAI_SPK_16KHZ),
    CTL_SWITCH_QUAD_LINKED("spk-16khz Playback Switch", AAI_SPK_16KHZ),
    CTL_VOLUME_CODEC("Codec PGA Input Volume",          CODEC_PGA_INPUT),
    CTL_VOLUME_CODEC("Codec Record Volume",             CODEC_RECORD),
    CTL_SWITCH_CODEC("Codec PGA Input Switch",          CODEC_PGA_INPUT),
    CTL_SWITCH_CODEC("Codec Record Switch",             CODEC_RECORD),
    CTL_SWITCH_CODEC("Codec Output Playback Switch",    CODEC_PLAYBACK)
};

/**
 * Create a Mixer
 */
int aai_mixer_new_p6i(struct card_data_t *aai)
{
    struct snd_card *card = aai->card;
    unsigned int idx;
    int err;

    //snd_assert(aai != NULL, return -EINVAL);
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

