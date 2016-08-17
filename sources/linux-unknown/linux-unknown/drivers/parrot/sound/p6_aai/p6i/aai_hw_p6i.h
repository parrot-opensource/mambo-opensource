/**
 * @file   aai_p6i.h
 * @brief  AAI driver P6i specific header
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

#ifndef INCONCE_AAI_P6I_H
#define INCONCE_AAI_P6I_H

/**
 * P6i Audio Channels Identifiers
 */
typedef enum
{
    AAI_SPK_OUT0 = 0,      /*!< Loudspeaker output0 */
    AAI_SPK_OUT1,          /*!< Loudspeaker output1 */
    AAI_SPK_8KHZ,          /*!< Loudspeaker 8Khz */
    AAI_SPK_16KHZ,         /*!< Loudspeaker 16Khz */
    AAI_MIC0_8KHZ,
    AAI_MIC0_16KHZ,
    AAI_MIC0_MUSIC,
    AAI_MIC2_MUSIC,
    AAI_FBACK_8KHZ,
    AAI_FBACK_16KHZ,
    AAI_FBACK_MUSIC,
    AAI_PCM0_SO1,
    AAI_PCM0_SI1,
    AAI_PCM0_SO2,
    AAI_PCM0_SI2,

    AAI_NB_CHANNELS
} aai_pcm_ident_t;

#define DEV_MULTIMEDIA(_ipcm_) \
    ((_ipcm_ == AAI_SPK_OUT0   ) || \
     (_ipcm_ == AAI_SPK_OUT1   ) || \
     (_ipcm_ == AAI_MIC0_MUSIC ) || \
     (_ipcm_ == AAI_MIC2_MUSIC ) || \
     (_ipcm_ == AAI_FBACK_MUSIC))

#define DEV_MUSIC(_ipcm_) \
    ((_ipcm_ == AAI_SPK_OUT0   ) || \
     (_ipcm_ == AAI_SPK_OUT1   ) || \
     (_ipcm_ == AAI_MIC0_MUSIC ) || \
     (_ipcm_ == AAI_MIC2_MUSIC ) || \
     (_ipcm_ == AAI_FBACK_MUSIC))

#define DEV_VOICE_8K(_ipcm_) \
    ((_ipcm_ == AAI_SPK_8KHZ  ) || \
     (_ipcm_ == AAI_MIC0_8KHZ ) || \
     (_ipcm_ == AAI_FBACK_8KHZ))

#define DEV_VOICE_16K(_ipcm_) \
    ((_ipcm_ == AAI_SPK_16KHZ  ) || \
     (_ipcm_ == AAI_MIC0_16KHZ ) || \
     (_ipcm_ == AAI_FBACK_16KHZ))

#define DEV_SRCONV_OUT(_ipcm_) \
    ((_ipcm_ == AAI_SPK_OUT0) || \
     (_ipcm_ == AAI_SPK_OUT1))

#define DEV_SRCONV_IN(_ipcm_) \
    ((_ipcm_ == AAI_MIC2_MUSIC))

#define DEV_PCM(_ipcm_) \
    ((_ipcm_ == AAI_PCM0_SO1) || \
     (_ipcm_ == AAI_PCM0_SI1) || \
     (_ipcm_ == AAI_PCM0_SO2) || \
     (_ipcm_ == AAI_PCM0_SI2))

#define DEV_PCM0(_ipcm_) \
    ((_ipcm_ == AAI_PCM0_SO1) || \
     (_ipcm_ == AAI_PCM0_SI1) || \
     (_ipcm_ == AAI_PCM0_SO2) || \
     (_ipcm_ == AAI_PCM0_SI2))

#define SPK_OUT0_ON        (1 << AAI_SPK_OUT0)
#define SPK_OUT1_ON        (1 << AAI_SPK_OUT1)
#define SPK_8K_ON          (1 << AAI_SPK_8KHZ)
#define SPK_16K_ON         (1 << AAI_SPK_16KHZ)

#define MIC0_8K_ON         (1 << AAI_MIC0_8KHZ)
#define MIC0_16K_ON        (1 << AAI_MIC0_16KHZ)
#define MIC0_MUSIC_ON      (1 << AAI_MIC0_MUSIC)
#define MIC2_MUSIC_ON      (1 << AAI_MIC2_MUSIC)
#define BACK_8K_ON         (1 << AAI_FBACK_8KHZ)
#define BACK_16K_ON        (1 << AAI_FBACK_16KHZ)
#define BACK_MUSIC_ON      (1 << AAI_FBACK_MUSIC)

#define PCM0_SO1_ON        (1 << AAI_PCM0_SO1)
#define PCM0_SO2_ON        (1 << AAI_PCM0_SO2)
#define PCM0_SI1_ON        (1 << AAI_PCM0_SI1)
#define PCM0_SI2_ON        (1 << AAI_PCM0_SI2)

/*
 * P6i internal data
 * These are defined in aai_hw_struct_xxx.c
 */
#if CONFIG_AAI_DBG_LEVEL > 2
extern struct virtual_reg vaai_p6i[];
#endif
extern aai_device_t aai_channels_p6i[];

/*
 * Exported P6i specific funtctions
 */
uint32_t* aai_adc_getdata(void);
int aai_ioctl_hwdep_new_p6i(struct card_data_t  *aai, char * id, int device);
irqreturn_t aai_irq_p6i(int irq, void *dev);
int aai_mixer_new_p6i(struct card_data_t *aai);

/*
 * Constraints definitions and checking functions
 */

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
                         MIC2_MUSIC_ON | \
                         SPK_OUT0_ON   | \
                         SPK_OUT1_ON)
static inline int srconv_is_locked(uint32_t deven)
{
   if (deven & DEV_SRCONV_ON)
      return 1;

   return 0;
}

/**
 * Is any Multimedia channels(except this one) enabled.
 * Multimedia devices are : AAI_MIC0_MUSIC, AAI_MIC2_MUSIC, AAI_FBACK_MUSIC,
 *                          AAI_SPK_OUT0, AAI_SPK_OUT1
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_MULTIMEDIA_ON  (SPK_OUT0_ON    | \
                            SPK_OUT1_ON    | \
                            BACK_MUSIC_ON  | \
                            MIC0_MUSIC_ON  | \
                            MIC2_MUSIC_ON)
static inline int compactbit_locked(uint32_t deven, uint32_t ipcm)
{
   if (deven & (DEV_MULTIMEDIA_ON & (~(1 << ipcm))))
      return 1;

   return 0;
}

/**
 * Is any Music channels(except this one) enabled.
 * Multimedia devices are : AAI_MIC0_MUSIC, AAI_MIC2_MUSIC, AAI_FBACK_MUSIC,
 *                          AAI_SPK_OUT0, AAI_SPK_OUT1
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_MUSIC_ON    (SPK_OUT0_ON    | \
                         SPK_OUT1_ON    | \
                         BACK_MUSIC_ON  | \
                         MIC0_MUSIC_ON  | \
                         MIC2_MUSIC_ON)
static inline int dmacnt_music_locked(uint32_t deven)
{
   if (deven & (DEV_MUSIC_ON))
      return 1;

   return 0;
}
 
/** Does any Voice channels(except this one) is enabled.
 * Voice devices are: AAI_SPK_8KHZ, AAI_SPK_16KHZ, AAI_MIC0_8KHZ,
 *                    AAI_MIC0_16KHZ, AAI_FBACK_8KHZ,AAI_FBACK_16KHZ,
 *                    AAI_PCM0_SO1, AAI_PCM0_SO2, AAI_PCM0_SI1,
 *                    AAI_PCM0_SI2
 * @return boolean (0:disabled / 1:enabled)
 */
#define DEV_VOICE_ON    (AAI_SPK_8KHZ   | \
                         AAI_SPK_16KHZ  | \
                         AAI_MIC0_8KHZ  | \
                         AAI_MIC0_16KHZ | \
                         AAI_FBACK_8KHZ | \
                         AAI_FBACK_16KHZ| \
                         PCM0_SO1_ON    | \
                         PCM0_SO2_ON    | \
                         PCM0_SI1_ON    | \
                         PCM0_SI2_ON)
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
                         PCM0_SI1_ON | \
                         PCM0_SI2_ON)
static inline int pcm0_rate_locked(uint32_t deven, int ipcm)
{
   if (deven & (DEV_PCM0_ON & (~(1 << ipcm))))
       return 1;

   return 0;
}

#endif

