/*
 * @file   aai.h
 * @brief  AAI driver header
 *
 *  Written by Gregoire ETIENNE <gregoire.etienne@parrot.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *      Parrot Advanced Audio Interface Driver
 *
 */

#ifndef INCONCE_AAI_H
#define INCONCE_AAI_H

#include <mach/parrot.h>

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
#include <sound/pcm_params.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

#include <sound/hwdep.h>

/*
 * Set this to use fifo on feedback & input channels
 */
#define NODMA 1

/*
 * Debug macros
 */
#define aai_print(dev,...)  dev_info(dev,__VA_ARGS__)

#if CONFIG_AAI_DBG_LEVEL > 0
# define aai_err(dev,...) \
    {                                                      \
        dev_err(dev, "[EE]"__VA_ARGS__);                   \
        dev_err(dev, "\tf:%s, l:%d\n",__FILE__, __LINE__); \
    };
# define aai_warn(dev,...)   dev_warn(dev,"[WW] "__VA_ARGS__)

#else
# define aai_err(dev,...)
# define aai_warn(dev,...)
#endif

#if CONFIG_AAI_DBG_LEVEL > 1
# define aai_info(dev,...) dev_info(dev,"[II] "__VA_ARGS__)
#else
# define aai_info(dev,...)
#endif

#if CONFIG_AAI_DBG_LEVEL > 2
# define aai_dbg(dev,...) dev_dbg(dev,"[  ] "__VA_ARGS__)
#else
# define aai_dbg(dev,...)
#endif


#if CONFIG_AAI_DBG_LEVEL > 2
# define aai_api(dev,...) dev_dbg(dev,"[IF] "__VA_ARGS__)
#else
# define aai_api(dev,...)
#endif

#define MONO_STREAM         1
#define STEREO_STREAM       2
#define QUAD_STREAM         4

#define AAI_FIFO_2W_32b     (2)  /* 2  words(u32) = 2  stereo samples(u16) =  8 bytes */
#define AAI_FIFO_4W_32b     (4)  /* 4  words(u32) = 4  stereo samples(u16) = 16 bytes */
#define AAI_FIFO_8W_32b     (8)  /* 8  words(u32) = 8  stereo samples(u16) = 32 bytes */
#define AAI_FIFO_16W_32b    (16) /* 16 words(u32) = 16 stereo samples(u16) = 64 bytes */
#define AAI_FIFO_16W_64b    (32) /* 16 dwords(u64) = 16 stereo samples(u32) = 128 bytes */
#define AAI_DMA_XFER        (0xffffffff) /* DMA transfer, FIFO not used */

/**
 * AAI directions
 */
typedef enum
{
    AAI_RX   = 0x1,
    AAI_TX   = 0x2,
} aai_dir_t;

/**
 * Define whether a channel is interleaved
 */
typedef enum
{
    AAI_INTERLEAVED    = 0x1,
    AAI_NONINTERLEAVED = 0x2,
} aai_access_t;

/**
 * AAI register rules
 */
typedef struct
{
   uint32_t             idx;    /*!< Resgister index */
   uint32_t             addr;   /*!< Resgister address */
   uint32_t             mask;   /*!< bit mask */
   uint32_t             value;  /*!< bit values */
}  aai_regrules_t;

enum
{
    RULE_CFG,
    RULE_ITEN,
};

#define END_OF_RULES {.addr=0,.addr=0, .mask=0, .value=0}
#define IS_LAST_RULE(_rule_) ((_rule_).addr==0)

#define ICH0_MUSIC  {.idx=RULE_CFG, .addr=AAI_CFG,  .mask=AAI_CFG_MUSIC_ICH0,   .value=AAI_CFG_MUSIC_ICH0}
#define ICH0_VOICE  {.idx=RULE_CFG, .addr=AAI_CFG,  .mask=AAI_CFG_MUSIC_ICH0,   .value=0}
#define ICH2_MUSIC  {.idx=RULE_CFG, .addr=AAI_CFG,  .mask=AAI_CFG_MUSIC_ICH2,   .value=AAI_CFG_MUSIC_ICH2}
#define ICH2_VOICE  {.idx=RULE_CFG, .addr=AAI_CFG,  .mask=AAI_CFG_MUSIC_ICH2,   .value=0}
#define BACK_MUSIC  {.idx=RULE_CFG, .addr=AAI_CFG,  .mask=AAI_CFG_MUSIC_BACK,   .value=AAI_CFG_MUSIC_BACK}
#define BACK_VOICE  {.idx=RULE_CFG, .addr=AAI_CFG,  .mask=AAI_CFG_MUSIC_BACK,   .value=0}
#define BUG_SRCOUT  {.idx=RULE_ITEN,.addr=AAI_ITEN, .mask=AAI_ITEN_SRC_OUT,     .value=0}
#define BINARY_SPDIF {.idx=RULE_CFG, .addr=AAI_CFG, .mask=AAI_CFG_BINARY_RUN,   .value=0}

#define UNIT_PER_DB 4        /*<! number of AAI volume unit(ie jplevan) per dB */
#define MUTE_VOLUME  (-256)  /*<! mute value */

/**
 * AAI channel definition
 */
typedef struct
{
    uint32_t             hfifo;         /*!< HW fifo in for this channel */
    char                 *pstart;       /*!< Ptr to start of transfer buffer */
    char                 *pcurrent;     /*!< Ptr to next chunk to transfer */
    char                 *pend;         /*!< Ptr to end of transfer buffer */
    char                 *pfollow;      /*!< Ptr to following chunk to transfer (DMA only) */
    uint32_t             dmasa;         /*!< DMA start address */
    uint32_t             dmafa;         /*!< DMA following address */
}  aai_hwchan_t;


/**
 * AAI pcm stream
 */
#define NB_RULES    4
typedef struct _aai_device_t
{
    /* Misc informations */
    void                    *pdrvdata;           /*!< Ptr to driver data */
    char                    name[12];            /*!< Device name ("spk", ... etc) */
    int                     ipcm;                /*!< pcm identifier number */
    aai_dir_t               direction;           /*!< RX/TX */
    int                     mode;                /*!< Size of FIFO tranfer */

    /* Alsa informations */
    struct snd_pcm_ops      *ops;                /*!< PCM callbacks operators */
    struct snd_pcm_hardware *hw;                 /*!< alsa hardware params pointer */
    uint32_t                devexclusion;        /*!< Device exclusion bit field */
    uint32_t                rate;                /*!< Rate value */
    uint32_t                period_frames;       /*!< alsa period size in frame */
    uint32_t                period_bytes;        /*!< alsa period size in bytes */
    uint32_t                bytes_count;         /*!< amount of bytes read or written */
    int                     access;              /*!< interleaving mode */
    int                     volume[QUAD_STREAM]; /*!< Volume value (OUT0L,OUT0R,OUT1L,OUT1R) */
    int                     mute[QUAD_STREAM];   /*!< Mute switch value */

    /* Hardware informations */
    aai_regrules_t          regrules[NB_RULES];  /*!< registers constraints */
    uint32_t                intflag;             /*!< HW interrupt flag for this channel */
    uint32_t                enflag;              /*!< HW enable flag for this channel */
    aai_hwchan_t            fifo[2];             /*!< HW fifo informations */

    /* DMA informations      */
    uint32_t                dmaflag;             /*!< DMA enable flag */
    uint32_t                dmaxfersz;           /*!< DMA transfer size */
    unsigned char *         dma_area;            /*!< DMA area for FIFO transfer */

    /* Buffer informations */
    uint32_t                bufsize;             /*!< Buffer size (for each fifo) */
    uint32_t                nbfiq;               /*!< Number FIQ (Debug only) */
    uint32_t                nbirq;               /*!< Number IRQ (Debug only) */
    uint32_t                nbper;               /*!< Number period elapsed (Debug only) */
    uint32_t                sync:1;                /*!< Synchronized flag */
    uint32_t                started:1;              /*!< Started flag */
} aai_device_t;

#define NB_CONSTRAINTS  10
struct card_data_t
{
    /* alsa structures      */
    struct snd_card     *card;
    struct snd_pcm      **pcms;
    int32_t             pcms_cnt;
    aai_device_t        *chans;
    uint32_t            deven;
    int                 irq;
    int                 fiq;

    /* memory ressource     */
    void __iomem        *iobase;
    struct resource     *ioarea;
    struct device       *dev;

    /* hardware dep         */
    struct snd_hwdep    *hwdep;
    spinlock_t          mixer_lock;
    spinlock_t          hwlock;
    aai_regrules_t      constraints[NB_CONSTRAINTS]; /*!< registers constraints */

    /* internal ops         */
    struct spec_ops_t   *spec_ops;

    /* debug purpose: virtual reg description */
#if CONFIG_AAI_DBG_LEVEL > 0
    struct virtual_reg *vaai;
#endif

    /* IRQ handle */
    irq_handler_t       irq_handler;
};

/**
 * AAI internal target specific operations
 */
struct spec_ops_t
{
    /* Init */
    int (*hw_init)(struct card_data_t *aai);

    /* Config */
    int (*set_channelrate)(aai_device_t *chan, int rate);
    int (*get_volume)(aai_device_t *chan);
    int (*set_volume)(aai_device_t *chan);

    /* DMA */
    int (*get_dma_cnt)(struct card_data_t *aai, uint32_t ipcm);
    int (*set_dma_cnt)(struct card_data_t *aai, uint32_t ipcm, int nbdma);
    int (*dma_prepare)(aai_device_t *chan);
    int (*dma_close)(aai_device_t *chan);

    /* Sample Rate Converter */
    int (*set_srconvdir)(struct card_data_t *aai, int val);
    int (*get_srconvrate)(struct card_data_t *aai);
    int (*set_srconvrate)(struct card_data_t *aai, int rate);

    /* I2S clocks */
    void (*disable_i2s_bit_clock)(void);
    void (*enable_i2s_bit_clock)(void);
    void (*disable_i2s_master_clock)(void);
    void (*enable_i2s_master_clock)(void);
};

/*
 * Exported prototypes
 */
struct snd_card * __devinit aai_alsa_probe(struct platform_device *pdev);
int __devexit aai_alsa_remove(struct snd_card *card);

/*
 * Exported datas
 */
extern struct snd_pcm_ops aai_pcm_ops;

/*
 * Inlined functions
 */

/**
 * Get pointer to playback substream
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

/**
 * Get pointer to capture substream
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

/**
 * Get pointer to substream's aai-pcm data
 *
 * @param substream
 * @return pointer to aai-pcm data
 */
static inline aai_device_t* substream2chan(struct snd_pcm_substream * substream)
{
    return (aai_device_t*)(substream->pcm->private_data);
}

/**
 * Get the irq handler function for running target
 */
irq_handler_t aai_irq_gethandler(void);

#define chan2dev(_chan_) (((struct card_data_t*)chan->pdrvdata)->dev)

#endif

