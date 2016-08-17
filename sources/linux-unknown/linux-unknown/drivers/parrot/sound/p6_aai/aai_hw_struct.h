
#ifndef INCONCE_AAI_HW_STRUCT
#define INCONCE_AAI_HW_STRUCT

#define DFLT_BUFFER_BYTES_MAX (64*1024)
#define DFLT_PERIOD_BYTES_MAX (16384)
#define DFLT_PERIOD_MIN       (2)
#define DFLT_PERIOD_MAX       (PAGE_SIZE / 16)

static struct snd_pcm_hardware aai_pcm_hw_srconvout =
{
    .info             = SNDRV_PCM_INFO_MMAP               |
                        SNDRV_PCM_INFO_MMAP_VALID         |
                        SNDRV_PCM_INFO_INTERLEAVED        |
                        /* portaudio cannot handle correctly non interleaved streams */
                        /* SNDRV_PCM_INFO_NONINTERLEAVED   | */
                        SNDRV_PCM_INFO_BLOCK_TRANSFER     |
                        SNDRV_PCM_INFO_RESUME,

    .formats          = SNDRV_PCM_FMTBIT_S32_LE,
    .rates            = SNDRV_PCM_RATE_CONTINUOUS,
    .rate_max         = 88000,
    .rate_min         = 16000,
    .channels_min     = 2,
    .channels_max     = 2,
    .buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
    .period_bytes_min = DMASIZE_32w32bits,    /*128 bytes*/
    .period_bytes_max = DFLT_PERIOD_BYTES_MAX*2,
    .periods_min      = DFLT_PERIOD_MIN,
    .periods_max      = DFLT_PERIOD_MAX,
};

static struct snd_pcm_hardware aai_pcm_hw_srconvin =
{
    .info             = SNDRV_PCM_INFO_MMAP           |
                        SNDRV_PCM_INFO_MMAP_VALID     |
                        SNDRV_PCM_INFO_INTERLEAVED    |
                        SNDRV_PCM_INFO_BLOCK_TRANSFER |
                        SNDRV_PCM_INFO_RESUME,

    .formats          = SNDRV_PCM_FMTBIT_S16_LE,
    .rates            = SNDRV_PCM_RATE_CONTINUOUS,
    .rate_max         = 88000,
    .rate_min         = 16000,
    .channels_min     = 2,
    .channels_max     = 2,
    .buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
    .period_bytes_min = DMASIZE_16w32bits,    /*64 bytes*/
    .period_bytes_max = DFLT_PERIOD_BYTES_MAX*2,
    .periods_min      = DFLT_PERIOD_MIN,
    .periods_max      = DFLT_PERIOD_MAX,
};


static struct snd_pcm_hardware aai_pcm_hw_music =
{
    .info             = SNDRV_PCM_INFO_MMAP           |
                        SNDRV_PCM_INFO_MMAP_VALID     |
                        SNDRV_PCM_INFO_INTERLEAVED    |
                        SNDRV_PCM_INFO_BLOCK_TRANSFER |
                        SNDRV_PCM_INFO_RESUME,

    .formats          = SNDRV_PCM_FMTBIT_S16_LE,
    .rates            = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
    .rate_max         = 48000,
    .rate_min         = 44100,
    .channels_min     = 2,
    .channels_max     = 2,
    .buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
    .period_bytes_min = DMASIZE_16w32bits,    /*64 bytes*/
    .period_bytes_max = DFLT_PERIOD_BYTES_MAX,
    .periods_min      = DFLT_PERIOD_MIN,
    .periods_max      = DFLT_PERIOD_MAX,
};


static struct snd_pcm_hardware aai_pcm_hw_voice8kmono =
{
    .info             = SNDRV_PCM_INFO_MMAP           |
                        SNDRV_PCM_INFO_MMAP_VALID     |
                        SNDRV_PCM_INFO_INTERLEAVED    |
                        SNDRV_PCM_INFO_BLOCK_TRANSFER |
                        SNDRV_PCM_INFO_RESUME,

    .formats          = SNDRV_PCM_FMTBIT_S16_LE,
    .rates            = SNDRV_PCM_RATE_8000,
    .rate_max         = 8000,
    .rate_min         = 8000,
    .channels_min     = 1,
    .channels_max     = 1,
    .buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
    .period_bytes_min = DMASIZE_2w32bits,     /*8 bytes*/
    .period_bytes_max = DFLT_PERIOD_BYTES_MAX,
    .periods_min      = DFLT_PERIOD_MIN,
    .periods_max      = DFLT_PERIOD_MAX,
};


static struct snd_pcm_hardware aai_pcm_hw_voice8kstereo =
{
    .info             = SNDRV_PCM_INFO_MMAP           |
                        SNDRV_PCM_INFO_MMAP_VALID     |
                        SNDRV_PCM_INFO_INTERLEAVED    |
                        SNDRV_PCM_INFO_BLOCK_TRANSFER |
                        SNDRV_PCM_INFO_RESUME,

    .formats          = SNDRV_PCM_FMTBIT_S16_LE,
    .rates            = SNDRV_PCM_RATE_8000,
    .rate_max         = 8000,
    .rate_min         = 8000,
    .channels_min     = 2,
    .channels_max     = 2,
    .buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
    .period_bytes_min = DMASIZE_4w32bits,     /*16 bytes*/
    .period_bytes_max = DFLT_PERIOD_BYTES_MAX,
    .periods_min      = DFLT_PERIOD_MIN,
    .periods_max      = DFLT_PERIOD_MAX,
};


static struct snd_pcm_hardware aai_pcm_hw_voice16kmono =
{
    .info             = SNDRV_PCM_INFO_MMAP           |
                        SNDRV_PCM_INFO_MMAP_VALID     |
                        SNDRV_PCM_INFO_INTERLEAVED    |
                        SNDRV_PCM_INFO_BLOCK_TRANSFER |
                        SNDRV_PCM_INFO_RESUME,

    .formats          = SNDRV_PCM_FMTBIT_S16_LE,
    .rates            = SNDRV_PCM_RATE_16000,
    .rate_max         = 16000,
    .rate_min         = 16000,
    .channels_min     = 1,
    .channels_max     = 1,
    .buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
    .period_bytes_min = DMASIZE_4w32bits,     /*16 bytes*/
    .period_bytes_max = DFLT_PERIOD_BYTES_MAX,
    .periods_min      = DFLT_PERIOD_MIN,
    .periods_max      = DFLT_PERIOD_MAX,
};

static struct snd_pcm_hardware aai_pcm_hw_voice16kstereo =
{
    .info             = SNDRV_PCM_INFO_MMAP             |
                        SNDRV_PCM_INFO_MMAP_VALID       |
                        SNDRV_PCM_INFO_INTERLEAVED      |
                        SNDRV_PCM_INFO_BLOCK_TRANSFER   |
                        SNDRV_PCM_INFO_RESUME,

    .formats          = SNDRV_PCM_FMTBIT_S16_LE,
    .rates            = SNDRV_PCM_RATE_16000,
    .rate_max         = 16000,
    .rate_min         = 16000,
    .channels_min     = 2,
    .channels_max     = 2,
    .buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
    .period_bytes_min = DMASIZE_8w32bits,     /*32 bytes*/
    .period_bytes_max = DFLT_PERIOD_BYTES_MAX,
    .periods_min      = DFLT_PERIOD_MIN,
    .periods_max      = DFLT_PERIOD_MAX,
};

static struct snd_pcm_hardware aai_pcm_hw_pcm =
{
    .info             = SNDRV_PCM_INFO_MMAP             |
                        SNDRV_PCM_INFO_MMAP_VALID       |
                        SNDRV_PCM_INFO_INTERLEAVED      |
                        SNDRV_PCM_INFO_BLOCK_TRANSFER   |
                        SNDRV_PCM_INFO_RESUME,

    .formats          = SNDRV_PCM_FMTBIT_S16_LE,
    .rates            = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000,
    .rate_max         = 16000,
    .rate_min         = 8000,
    .channels_min     = 1,
    .channels_max     = 1,
    .buffer_bytes_max = DFLT_BUFFER_BYTES_MAX,
    .period_bytes_min = DMASIZE_2w32bits,     /*8 bytes*/
    .period_bytes_max = DFLT_PERIOD_BYTES_MAX,
    .periods_min      = DFLT_PERIOD_MIN,
    .periods_max      = DFLT_PERIOD_MAX,
};

#endif

