/**
 * @file   aai.h
 * @brief  AAI driver P6 specific header
 *
 *  Written by Adrien Charruel <adrien.charruel@parrot.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *      Parrot Advanced Audio Interface Driver
 *
 */

#ifndef INCONCE_AAI_P6_H
#define INCONCE_AAI_P6_H

/**
 * P6 Audio Channels Identifiers
 */
typedef enum
{
    AAI_SPK_OUT0 = 0,      /*!< Loudspeaker output0   */
    AAI_SPK_OUT1,          /*!< Loudspeaker output1   */
    AAI_SPK_8KHZ,          /*!< Loudspeaker 8Khz      */
    AAI_SPK_16KHZ,         /*!< Loudspeaker 16Khz     */
    AAI_SPK_AUX,           /*!< Loudspeaker Auxiliary */
    AAI_MIC0_8KHZ,
    AAI_MIC0_16KHZ,
    AAI_MIC0_MUSIC,
    AAI_MIC1_MUSIC,
    AAI_MIC2_8KHZ,
    AAI_MIC2_16KHZ,
    AAI_MIC2_MUSIC,
    AAI_MIC3_MUSIC,
    AAI_FBACK_8KHZ,
    AAI_FBACK_16KHZ,
    AAI_FBACK_MUSIC,
    AAI_PCM0_SO1,
    AAI_PCM0_SI1,
    AAI_PCM0_SO2,
    AAI_PCM0_SI2,
    AAI_PCM0_SO3,
    AAI_PCM0_SI3,
    AAI_PCM1_SO1,
    AAI_PCM1_SI1,
    AAI_PCM1_SO2,
    AAI_PCM1_SI2,
    /*AAI_BINARY,*/

    AAI_NB_CHANNELS
} aai_pcm_ident_t;

#define DEV_MULTIMEDIA(_ipcm_) \
    ((_ipcm_ == AAI_SPK_OUT0   ) || \
     (_ipcm_ == AAI_SPK_OUT1   ) || \
     (_ipcm_ == AAI_SPK_AUX    ) || \
     (_ipcm_ == AAI_MIC0_MUSIC ) || \
     (_ipcm_ == AAI_MIC1_MUSIC ) || \
     (_ipcm_ == AAI_MIC2_MUSIC ) || \
     (_ipcm_ == AAI_MIC3_MUSIC ) || \
     (_ipcm_ == AAI_FBACK_MUSIC))

#define DEV_MUSIC(_ipcm_) \
    ((_ipcm_ == AAI_SPK_OUT0   ) || \
     (_ipcm_ == AAI_SPK_OUT1   ) || \
     (_ipcm_ == AAI_MIC0_MUSIC ) || \
     (_ipcm_ == AAI_MIC2_MUSIC ) || \
     (_ipcm_ == AAI_MIC3_MUSIC ) || \
     (_ipcm_ == AAI_FBACK_MUSIC))

#define DEV_VOICE_8K(_ipcm_) \
    ((_ipcm_ == AAI_SPK_8KHZ  ) || \
     (_ipcm_ == AAI_MIC0_8KHZ ) || \
     (_ipcm_ == AAI_MIC2_8KHZ ) || \
     (_ipcm_ == AAI_FBACK_8KHZ) || \
     (_ipcm_ == AAI_FBACK_8KHZ))

#define DEV_VOICE_16K(_ipcm_) \
    ((_ipcm_ == AAI_SPK_16KHZ  ) || \
     (_ipcm_ == AAI_MIC0_16KHZ ) || \
     (_ipcm_ == AAI_MIC2_16KHZ ) || \
     (_ipcm_ == AAI_FBACK_16KHZ) || \
     (_ipcm_ == AAI_FBACK_16KHZ))

#define DEV_SRCONV_OUT(_ipcm_) \
    ((_ipcm_ == AAI_SPK_OUT0) || (_ipcm_ == AAI_SPK_OUT1))

#define DEV_SRCONV_IN(_ipcm_) \
    ((_ipcm_ == AAI_MIC2_MUSIC) || \
     (_ipcm_ == AAI_MIC3_MUSIC))

#define DEV_AUX(_ipcm_) \
    ((_ipcm_ == AAI_SPK_AUX   ) || \
     (_ipcm_ == AAI_MIC1_MUSIC))

#define DEV_PCM(_ipcm_) \
    ((_ipcm_ == AAI_PCM0_SO1) || \
     (_ipcm_ == AAI_PCM0_SI1) || \
     (_ipcm_ == AAI_PCM0_SO2) || \
     (_ipcm_ == AAI_PCM0_SI2) || \
     (_ipcm_ == AAI_PCM0_SO3) || \
     (_ipcm_ == AAI_PCM0_SI3) || \
     (_ipcm_ == AAI_PCM1_SO1) || \
     (_ipcm_ == AAI_PCM1_SI1) || \
     (_ipcm_ == AAI_PCM1_SO2) || \
     (_ipcm_ == AAI_PCM1_SI2))

#define DEV_PCM0(_ipcm_) \
    ((_ipcm_ == AAI_PCM0_SO1) || \
     (_ipcm_ == AAI_PCM0_SI1) || \
     (_ipcm_ == AAI_PCM0_SO2) || \
     (_ipcm_ == AAI_PCM0_SI2) || \
     (_ipcm_ == AAI_PCM0_SO3) || \
     (_ipcm_ == AAI_PCM0_SI3))

#define DEV_PCM1(_ipcm_) \
    ((_ipcm_ == AAI_PCM1_SO1) || \
     (_ipcm_ == AAI_PCM1_SI1) || \
     (_ipcm_ == AAI_PCM1_SO2) || \
     (_ipcm_ == AAI_PCM1_SI2))

#define SPK_OUT0_ON     (1 << AAI_SPK_OUT0)
#define SPK_OUT1_ON     (1 << AAI_SPK_OUT1)
#define SPK_8K_ON       (1 << AAI_SPK_8KHZ)
#define SPK_16K_ON      (1 << AAI_SPK_16KHZ)
#define SPK_AUX_ON      (1 << AAI_SPK_AUX)

#define MIC0_8K_ON      (1 << AAI_MIC0_8KHZ)
#define MIC0_16K_ON     (1 << AAI_MIC0_16KHZ)
#define MIC0_MUSIC_ON   (1 << AAI_MIC0_MUSIC)
#define MIC1_MUSIC_ON   (1 << AAI_MIC1_MUSIC)
#define MIC2_8K_ON      (1 << AAI_MIC2_8KHZ)
#define MIC2_16K_ON     (1 << AAI_MIC2_16KHZ)
#define MIC2_MUSIC_ON   (1 << AAI_MIC2_MUSIC)
#define MIC3_MUSIC_ON   (1 << AAI_MIC3_MUSIC)
#define BACK_8K_ON      (1 << AAI_FBACK_8KHZ)
#define BACK_16K_ON     (1 << AAI_FBACK_16KHZ)
#define BACK_MUSIC_ON   (1 << AAI_FBACK_MUSIC)

#define PCM0_SO1_ON     (1 << AAI_PCM0_SO1)
#define PCM0_SO2_ON     (1 << AAI_PCM0_SO2)
#define PCM0_SO3_ON     (1 << AAI_PCM0_SO3)
#define PCM1_SO1_ON     (1 << AAI_PCM1_SO1)
#define PCM1_SO2_ON     (1 << AAI_PCM1_SO2)
#define PCM0_SI1_ON     (1 << AAI_PCM0_SI1)
#define PCM0_SI2_ON     (1 << AAI_PCM0_SI2)
#define PCM0_SI3_ON     (1 << AAI_PCM0_SI3)
#define PCM1_SI1_ON     (1 << AAI_PCM1_SI1)
#define PCM1_SI2_ON     (1 << AAI_PCM1_SI2)

/*
 * P6 internal data
 * These are defined in aai_hw_struct_xxx.c
 */
#if CONFIG_AAI_DBG_LEVEL > 0
extern struct virtual_reg vaai_p6[];
#endif
extern aai_device_t aai_channels_p6[];

/*
 * Exported P6 specific funtctions
 */
int aai_ioctl_hwdep_new_p6(struct card_data_t  *aai, char * id, int device);
irqreturn_t aai_irq_p6(int irq, void *dev);
int aai_mixer_new_p6(struct card_data_t *aai);

/**
 * Is spk-outt0 channel enabled.
 *
 * @param iten ITEN register value
 * @param dmactl DMACTL register value
 *
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_SRCONV_ON   (MIC0_8K_ON    | \
                         MIC0_16K_ON   | \
                         MIC0_MUSIC_ON | \
                         MIC2_8K_ON    | \
                         MIC2_16K_ON   | \
                         MIC2_MUSIC_ON | \
                         SPK_OUT0_ON   | \
                         SPK_OUT1_ON   | \
                         MIC3_MUSIC_ON)
static inline int srconv_is_locked(uint32_t deven)
{
   if (deven & DEV_SRCONV_ON)
      return 1;

   return 0;
}

/**
 * Is any Multimedia channels(except this one) enabled.
 * Multimedia devices are : AAI_MIC0_MUSIC, AAI_MIC1_MUSIC, AAI_MIC2_MUSIC,
 *                          AAI_MIC3_MUSIC, AAI_FBACK_MUSIC,
 *                          AAI_SPK_OUT0, AAI_SPK_OUT1, AAI_SPK_AUX
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_MULTIMEDIA_ON  (SPK_OUT0_ON    | \
                            SPK_OUT1_ON    | \
                            BACK_MUSIC_ON  | \
                            SPK_AUX_ON     | \
                            MIC0_MUSIC_ON  | \
                            MIC1_MUSIC_ON  | \
                            MIC2_MUSIC_ON  | \
                            MIC3_MUSIC_ON)

static inline int compactbit_locked(uint32_t deven, uint32_t ipcm)
{
   if (deven & (DEV_MULTIMEDIA_ON & (~(1 << ipcm))))
      return 1;

   return 0;
}

/**
 * Is any Music channels(except this one) enabled.
 * Multimedia devices are : AAI_MIC0_MUSIC, AAI_MIC2_MUSIC,
 *                          AAI_MIC3_MUSIC, AAI_FBACK_MUSIC,
 *                          AAI_SPK_OUT0, AAI_SPK_OUT1
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_MUSIC_ON    (SPK_OUT0_ON    | \
                         SPK_OUT1_ON    | \
                         BACK_MUSIC_ON  | \
                         MIC0_MUSIC_ON  | \
                         MIC2_MUSIC_ON  | \
                         MIC3_MUSIC_ON)
static inline int dmacnt_music_locked(uint32_t deven)
{
   if (deven & (DEV_MUSIC_ON))
      return 1;

   return 0;
}
 
/**
 * Are auxiliary channels enabled.
 * Auxiliary devices are : AAI_MIC1_MUSIC, AAI_SPK_AUX
 *
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_AUX_ON  (SPK_AUX_ON     | \
                     MIC1_MUSIC_ON)
static inline int dmacnt_aux_locked(uint32_t deven)
{
   if (deven & (DEV_AUX_ON))
      return 1;

   return 0;
}

/** Does any Voice channels(except this one) is enabled.
 * Voice devices are : AAI_SPK_8KHZ,AAI_SPK_16KHZ,AAI_MIC0_8KHZ,AAI_MIC0_16KHZ,
 *                     AAI_MIC2_8KHZ,AAI_MIC2_16KHZ,AAI_FBACK_8KHZ,AAI_FBACK_16KHZ
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_VOICE_ON    (AAI_SPK_8KHZ   | \
                         AAI_SPK_16KHZ  | \
                         AAI_MIC0_8KHZ  | \
                         AAI_MIC0_16KHZ | \
                         AAI_MIC2_8KHZ  | \
                         AAI_MIC2_16KHZ | \
                         AAI_FBACK_8KHZ | \
                         AAI_FBACK_16KHZ| \
                         PCM0_SO1_ON    | \
                         PCM0_SO2_ON    | \
                         PCM0_SO3_ON    | \
                         PCM1_SO1_ON    | \
                         PCM1_SO2_ON    | \
                         PCM0_SI1_ON    | \
                         PCM0_SI2_ON    | \
                         PCM0_SI3_ON    | \
                         PCM1_SI1_ON    | \
                         PCM1_SI2_ON)
static inline int dmacnt_voice_locked(uint32_t deven)
{
    if (deven & (DEV_VOICE_ON))
        return 1;

   return 0;
}

/**
 * Is any PCM0 device(except this one) enabled.
 *
 * @param deven device enable mask
 * @param ipcm device id
 *
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_PCM0_ON     (PCM0_SO1_ON | \
                         PCM0_SO2_ON | \
                         PCM0_SO3_ON | \
                         PCM0_SI1_ON | \
                         PCM0_SI2_ON | \
                         PCM0_SI3_ON)
static inline int pcm0_rate_locked(uint32_t deven, int ipcm)
{
   if (deven & (DEV_PCM0_ON&(~(1 << ipcm))))
       return 1;

   return 0;
}

/** Is any PCM1 device(except this one) enabled.
 *
 * @param deven device enable mask
 * @param ipcm device id
 *
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_PCM1_ON     (PCM1_SO1_ON    | \
                         PCM1_SO2_ON    | \
                         PCM1_SI1_ON    | \
                         PCM1_SI2_ON)
static inline int pcm1_rate_locked(uint32_t deven, int ipcm)
{
    if (deven & (DEV_PCM1_ON&(~(1 << ipcm))))
        return 1;

    return 0;
}

#endif

