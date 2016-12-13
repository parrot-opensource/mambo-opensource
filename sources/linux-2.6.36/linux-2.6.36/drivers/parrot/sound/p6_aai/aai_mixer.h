#ifndef INCONCE_AAI_MIXER_H
#define INCONCE_AAI_MIXER_H

/*
 * Common mixer functions
 */
int chan_volume_info_stereo(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
int chan_volume_info_quad(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);

int chan_volume_get_stereo(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int chan_volume_get_quad(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);

int chan_volume_put_stereo(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int chan_volume_put_quad(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int chan_volume_put_quad_linked(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);

int chan_mute_info_quad(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);
int chan_mute_info_stereo(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo);

int chan_mute_get_quad(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int chan_mute_get_stereo(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);

int chan_mute_put_quad_linked(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int chan_mute_put_quad(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int chan_mute_put_stereo(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);

/*
 * Define controls
 */
#define CTL_SWITCH_STEREO(_name_, _ipcm_) \
{                                               \
    .iface  = SNDRV_CTL_ELEM_IFACE_MIXER,       \
    .name   = _name_,                           \
    .index  = _ipcm_,                           \
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,  \
    .info   = chan_mute_info_stereo,            \
    .get    = chan_mute_get_stereo,             \
    .put    = chan_mute_put_stereo,             \
    .private_value = _ipcm_,                    \
}

#define CTL_SWITCH_QUAD(_name_, _ipcm_) \
{                                               \
    .iface  = SNDRV_CTL_ELEM_IFACE_MIXER,       \
    .name   = _name_,                           \
    .index  = _ipcm_,                           \
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,  \
    .info   = chan_mute_info_quad,              \
    .get    = chan_mute_get_quad,               \
    .put    = chan_mute_put_quad,               \
    .private_value = _ipcm_,                    \
}

#define CTL_SWITCH_QUAD_LINKED(_name_, _ipcm_) \
{                                               \
    .iface  = SNDRV_CTL_ELEM_IFACE_MIXER,       \
    .name   = _name_,                           \
    .index  = _ipcm_,                           \
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,  \
    .info   = chan_mute_info_quad,              \
    .get    = chan_mute_get_quad,               \
    .put    = chan_mute_put_quad_linked,        \
    .private_value = _ipcm_,                    \
}

#define CTL_VOLUME_STEREO(_name_, _ipcm_) \
{                                               \
    .iface  = SNDRV_CTL_ELEM_IFACE_MIXER,       \
    .name   = _name_,                           \
    .index  = _ipcm_,                           \
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,  \
    .info   = chan_volume_info_stereo,          \
    .get    = chan_volume_get_stereo,           \
    .put    = chan_volume_put_stereo,           \
    .private_value = _ipcm_,                    \
    .tlv.p = db_scale_my_control,               \
}

#define CTL_VOLUME_QUAD(_name_, _ipcm_) \
{                                               \
    .iface  = SNDRV_CTL_ELEM_IFACE_MIXER,       \
    .name   = _name_,                           \
    .index  = _ipcm_,                           \
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,  \
    .info   = chan_volume_info_quad,            \
    .get    = chan_volume_get_quad,             \
    .put    = chan_volume_put_quad,             \
    .private_value = _ipcm_,                    \
    .tlv.p = db_scale_my_control,               \
}

#define CTL_VOLUME_QUAD_LINKED(_name_, _ipcm_) \
{                                               \
    .iface  = SNDRV_CTL_ELEM_IFACE_MIXER,       \
    .name   = _name_,                           \
    .index  = _ipcm_,                           \
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,  \
    .info   = chan_volume_info_quad,            \
    .get    = chan_volume_get_quad,             \
    .put    = chan_volume_put_quad_linked,      \
    .private_value = _ipcm_,                    \
    .tlv.p = db_scale_my_control,               \
}

#define CTL_VOLUME_CODEC(_name_, _ipcm_) \
{                                               \
    .iface  = SNDRV_CTL_ELEM_IFACE_MIXER,       \
    .name   = _name_,                           \
    .index  = _ipcm_,                           \
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,  \
    .info   = chan_volume_info_codec,           \
    .get    = chan_volume_get_codec,            \
    .put    = chan_volume_put_codec,            \
    .private_value = _ipcm_,                    \
    .tlv.p = db_scale_my_control,               \
}

#define CTL_SWITCH_CODEC(_name_, _ipcm_) \
{                                               \
    .iface  = SNDRV_CTL_ELEM_IFACE_MIXER,       \
    .name   = _name_,                           \
    .index  = _ipcm_,                           \
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,  \
    .info   = chan_mute_info_codec,             \
    .get    = chan_mute_get_codec,              \
    .put    = chan_mute_put_codec,              \
    .private_value = _ipcm_,                    \
}

#endif

