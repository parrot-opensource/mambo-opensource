/**
 * @file   aai_hw_struct_p6.c
 * @brief  AAI structures initialization
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
#include "aai_hw_struct.h"
#include "aai_hw_p6.h"

#if CONFIG_AAI_DBG_LEVEL > 0

#define VIRTUAL_REG(_reg)  [IREG(_reg)] = {.addr = _reg, .reg=0, .name=#_reg}

struct virtual_reg vaai_p6[] =
{
   VIRTUAL_REG(AAI_CFG),
   VIRTUAL_REG(AAI_ITS),
   VIRTUAL_REG(AAI_ITEN),
   VIRTUAL_REG(AAI_SRCONV_RATIO),
   VIRTUAL_REG(AAI_I2S_FORMAT),
   VIRTUAL_REG(AAI_PCM0_CFG),
   VIRTUAL_REG(AAI_PCM1_CFG),
   VIRTUAL_REG(AAI_DMACTL),
   VIRTUAL_REG(AAI_PCM01_SYNC),
   VIRTUAL_REG(AAI_PCM10_SYNC),
   VIRTUAL_REG(AAI_LOUDREF_OUT0),
   VIRTUAL_REG(AAI_LOUDREF_OUT1),
   VIRTUAL_REG(AAI_LOUD_CTL),
   VIRTUAL_REG(AAI_MUSIC_DMA_COUNT),
   VIRTUAL_REG(AAI_AUX_DMA_COUNT),
   VIRTUAL_REG(AAI_VOICE_DMA_COUNT),
   VIRTUAL_REG(AAI_LEFT_MUSIC_VOL_OUT0),
   VIRTUAL_REG(AAI_RIGHT_MUSIC_VOL_OUT0),
   VIRTUAL_REG(AAI_LEFT_MUSIC_VOL_OUT1),
   VIRTUAL_REG(AAI_RIGHT_MUSIC_VOL_OUT1),
   VIRTUAL_REG(AAI_LEFT_AUX_VOL_OUT0),
   VIRTUAL_REG(AAI_RIGHT_AUX_VOL_OUT0),
   VIRTUAL_REG(AAI_LEFT_AUX_VOL_OUT1),
   VIRTUAL_REG(AAI_RIGHT_AUX_VOL_OUT1),
   VIRTUAL_REG(AAI_LEFT_16KHZ_VOL_OUT0),
   VIRTUAL_REG(AAI_RIGHT_16KHZ_VOL_OUT0),
   VIRTUAL_REG(AAI_LEFT_16KHZ_VOL_OUT1),
   VIRTUAL_REG(AAI_RIGHT_16KHZ_VOL_OUT1),
   VIRTUAL_REG(AAI_LEFT_AUX_VOL_OUT0),
   VIRTUAL_REG(AAI_RIGHT_AUX_VOL_OUT0),
   VIRTUAL_REG(AAI_LEFT_AUX_VOL_OUT1),
   VIRTUAL_REG(AAI_RIGHT_AUX_VOL_OUT1),
   VIRTUAL_REG(AAI_LEFT_16KHZ_VOL_OUT0),
   VIRTUAL_REG(AAI_RIGHT_16KHZ_VOL_OUT0),
   VIRTUAL_REG(AAI_LEFT_8KHZ_VOL_OUT0),
   VIRTUAL_REG(AAI_RIGHT_8KHZ_VOL_OUT0),
   VIRTUAL_REG(AAI_LEFT_8KHZ_VOL_OUT1),
   VIRTUAL_REG(AAI_RIGHT_8KHZ_VOL_OUT1),
   VIRTUAL_REG(AAI_LEFT_MUSIC_DMASA_OUT0),
   VIRTUAL_REG(AAI_LEFT_MUSIC_DMAFA_OUT0),
   VIRTUAL_REG(AAI_RIGHT_MUSIC_DMASA_OUT0),
   VIRTUAL_REG(AAI_RIGHT_MUSIC_DMAFA_OUT0),
   VIRTUAL_REG(AAI_LEFT_MUSIC_DMASA_OUT1),
   VIRTUAL_REG(AAI_LEFT_MUSIC_DMAFA_OUT1),
   VIRTUAL_REG(AAI_RIGHT_MUSIC_DMASA_OUT1),
   VIRTUAL_REG(AAI_RIGHT_MUSIC_DMAFA_OUT1),
   VIRTUAL_REG(AAI_16KHZ_DMASA_OUT),
   VIRTUAL_REG(AAI_16KHZ_DMAFA_OUT),
   VIRTUAL_REG(AAI_8KHZ_DMASA_OUT),
   VIRTUAL_REG(AAI_8KHZ_DMAFA_OUT),
   VIRTUAL_REG(AAI_AUX_DMASA_OUT),
   VIRTUAL_REG(AAI_AUX_DMAFA_OUT),
   VIRTUAL_REG(AAI_MUSIC_DMASA_ICH1),
   VIRTUAL_REG(AAI_MUSIC_DMAFA_ICH1),
   VIRTUAL_REG(AAI_MUSIC_DMASA_ICH0),
   VIRTUAL_REG(AAI_MUSIC_DMAFA_ICH0),
   VIRTUAL_REG(AAI_16KHZ_DMASA_ICH0),
   VIRTUAL_REG(AAI_16KHZ_DMAFA_ICH0),
   VIRTUAL_REG(AAI_8KHZ_DMASA_ICH0),
   VIRTUAL_REG(AAI_8KHZ_DMAFA_ICH0),
   VIRTUAL_REG(AAI_MUSIC_DMASA_ICH2),
   VIRTUAL_REG(AAI_MUSIC_DMAFA_ICH2),
   VIRTUAL_REG(AAI_16KHZ_DMASA_ICH2),
   VIRTUAL_REG(AAI_16KHZ_DMAFA_ICH2),
   VIRTUAL_REG(AAI_8KHZ_DMASA_ICH2),
   VIRTUAL_REG(AAI_8KHZ_DMAFA_ICH2),
   VIRTUAL_REG(AAI_MUSIC_DMASA_ICH3),
   VIRTUAL_REG(AAI_MUSIC_DMAFA_ICH3),
   VIRTUAL_REG(AAI_MUSIC_DMASA_BACK),
   VIRTUAL_REG(AAI_MUSIC_DMAFA_BACK),
   VIRTUAL_REG(AAI_16KHZ_DMASA_BACK),
   VIRTUAL_REG(AAI_16KHZ_DMAFA_BACK),
   VIRTUAL_REG(AAI_8KHZ_DMASA_BACK),
   VIRTUAL_REG(AAI_8KHZ_DMAFA_BACK),
   VIRTUAL_REG(AAI_SO1_DMASA_PCM0),
   VIRTUAL_REG(AAI_SO1_DMAFA_PCM0),
   VIRTUAL_REG(AAI_SI1_DMASA_PCM0),
   VIRTUAL_REG(AAI_SI1_DMAFA_PCM0),
   VIRTUAL_REG(AAI_SO2_DMASA_PCM0),
   VIRTUAL_REG(AAI_SO2_DMAFA_PCM0),
   VIRTUAL_REG(AAI_SI2_DMASA_PCM0),
   VIRTUAL_REG(AAI_SI2_DMAFA_PCM0),
   VIRTUAL_REG(AAI_SO3_DMASA_PCM0),
   VIRTUAL_REG(AAI_SO3_DMAFA_PCM0),
   VIRTUAL_REG(AAI_SI3_DMASA_PCM0),
   VIRTUAL_REG(AAI_SI3_DMAFA_PCM0),
   VIRTUAL_REG(AAI_SO1_DMASA_PCM1),
   VIRTUAL_REG(AAI_SO1_DMAFA_PCM1),
   VIRTUAL_REG(AAI_SI1_DMASA_PCM1),
   VIRTUAL_REG(AAI_SI1_DMAFA_PCM1),
   VIRTUAL_REG(AAI_SO2_DMASA_PCM1),
   VIRTUAL_REG(AAI_SO2_DMAFA_PCM1),
   VIRTUAL_REG(AAI_SI2_DMASA_PCM1),
   VIRTUAL_REG(AAI_SI2_DMAFA_PCM1),
   VIRTUAL_REG(AAI_BINARY_DMASA),
   VIRTUAL_REG(AAI_BINARY_DMAFA),
   VIRTUAL_REG(AAI_MUSIC_DMA_COUNT),
   VIRTUAL_REG(AAI_AUX_DMA_COUNT),
   VIRTUAL_REG(AAI_VOICE_DMA_COUNT),
   VIRTUAL_REG(AAI_BINARY_DMA_COUNT),
   {.addr = 0xFFFFFFFF, .reg=0, .name=""},
};
#endif

/**
 * Audio Channels
 */
aai_device_t aai_channels_p6[AAI_NB_CHANNELS] =
{
    [AAI_SPK_OUT0] =
    {
        .name     = "spk-out0",
        .ipcm     = AAI_SPK_OUT0,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_srconvout,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_OUT0,
        .intflag  = AAI_ITS_OUT0,
        .enflag   = AAI_ITEN_OUT0,
        .mode     = AAI_FIFO_16W_64b,
        .access   = AAI_NONINTERLEAVED,
        .fifo[0]  =
        {
            .hfifo = AAI_MUSIC_OUT0_LEFT,
            .dmasa = AAI_LEFT_MUSIC_DMASA_OUT0,
            .dmafa = AAI_LEFT_MUSIC_DMAFA_OUT0
        },
        .fifo[1]   =
        { 
            .hfifo = AAI_MUSIC_OUT0_RIGHT,
            .dmasa = AAI_RIGHT_MUSIC_DMASA_OUT0,
            .dmafa = AAI_RIGHT_MUSIC_DMAFA_OUT0
        },
    },
    [AAI_SPK_OUT1] =
    {
        .name     = "spk-out1",
        .ipcm     = AAI_SPK_OUT1,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_srconvout,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_OUT1,
        .intflag  = AAI_ITS_OUT1,
        .enflag   = AAI_ITEN_OUT1,
        .mode     = AAI_FIFO_16W_64b,
        .access   = AAI_NONINTERLEAVED,
        .fifo[0]  =
        { 
            .hfifo = AAI_MUSIC_OUT1_LEFT,
            .dmasa = AAI_LEFT_MUSIC_DMASA_OUT1,
            .dmafa = AAI_LEFT_MUSIC_DMAFA_OUT1
        },
        .fifo[1]   =
        { 
            .hfifo = AAI_MUSIC_OUT1_RIGHT,
            .dmasa = AAI_RIGHT_MUSIC_DMASA_OUT1,
            .dmafa = AAI_RIGHT_MUSIC_DMAFA_OUT1
        },
    },
    [AAI_SPK_16KHZ] =
    {
        .name     = "spk-16khz",
        .ipcm     = AAI_SPK_16KHZ,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_voice16kmono,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_16KHZ,
        .intflag  = AAI_ITS_16KHZ,
        .enflag   = AAI_ITEN_16KHZ,
        .mode     = AAI_FIFO_4W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_16KHZ_OUT,
            .dmasa = AAI_16KHZ_DMASA_OUT,
            .dmafa = AAI_16KHZ_DMAFA_OUT
        },
    },
    [AAI_SPK_8KHZ] =
    {
        .name     = "spk-8khz",
        .ipcm     = AAI_SPK_8KHZ,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_voice8kmono,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_8KHZ,
        .intflag  = AAI_ITS_8KHZ,
        .enflag   = AAI_ITEN_8KHZ,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_8KHZ_OUT,
            .dmasa = AAI_8KHZ_DMASA_OUT,
            .dmafa = AAI_8KHZ_DMAFA_OUT
        },
    },
    [AAI_SPK_AUX] =
    {
        .name     = "spk-aux",
        .ipcm     = AAI_SPK_AUX,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_music,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_AUX,
        .intflag  = AAI_ITS_AUX,
        .enflag   = AAI_ITEN_AUX,
        .mode     = AAI_FIFO_16W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_AUX_OUT,
            .dmasa = AAI_AUX_DMASA_OUT,
            .dmafa = AAI_AUX_DMAFA_OUT
        },
    },
    [AAI_MIC1_MUSIC] =
    {
        .name     = "mic1-music",
        .ipcm     = AAI_MIC1_MUSIC,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_music,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_ICH1,
        .intflag  = AAI_ITS_ICH1,
        .enflag   = AAI_ITEN_ICH1,
        .mode     = AAI_FIFO_16W_32b,
        .fifo[0]  =
        {
            .hfifo = AAI_MUSIC_ICH1,
            .dmasa = AAI_MUSIC_DMASA_ICH1,
            .dmafa = AAI_MUSIC_DMAFA_ICH1
        },
    },
    [AAI_MIC0_MUSIC] = 
    {
        .name     = "mic0-music",
        .ipcm     = AAI_MIC0_MUSIC,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_music,
        .devexclusion = (MIC0_8K_ON | MIC0_16K_ON),
        .regrules = { ICH0_MUSIC, END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_ICH0,
        .intflag  = AAI_ITS_MUSIC_ICH0,
        .enflag   = AAI_ITEN_16KHZ_ICH0|AAI_ITEN_8KHZ_ICH0,
        .mode     = AAI_FIFO_16W_32b,
        .fifo[0] =
        { 
            .hfifo = AAI_16KHZ_ICH0,
            .dmasa = AAI_MUSIC_DMASA_ICH0,
            .dmafa = AAI_MUSIC_DMAFA_ICH0
        },
    },
    [AAI_MIC0_16KHZ] = 
    {
        .name     = "mic0-16khz",
        .ipcm     = AAI_MIC0_16KHZ,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_voice16kstereo,
        .devexclusion = MIC0_MUSIC_ON,
        .regrules = { ICH0_VOICE, END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_ICH0,
        .intflag  = AAI_ITS_16KHZ_ICH0,
        .enflag   = AAI_ITEN_16KHZ_ICH0,
        .mode     = AAI_FIFO_8W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_16KHZ_ICH0,
            .dmasa = AAI_16KHZ_DMASA_ICH0,
            .dmafa = AAI_16KHZ_DMAFA_ICH0
        },
    },
    [AAI_MIC0_8KHZ] =
    {
        .name     = "mic0-8khz",
        .ipcm     = AAI_MIC0_8KHZ,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_voice8kstereo,
        .devexclusion = MIC0_MUSIC_ON,
        .regrules = { ICH0_VOICE, END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_ICH0,
        .intflag  = AAI_ITS_8KHZ_ICH0,
        .enflag   = AAI_ITEN_8KHZ_ICH0,
        .mode     = AAI_FIFO_4W_32b,
        .fifo[0]  =
        {
            .hfifo = AAI_8KHZ_ICH0,
            .dmasa = AAI_8KHZ_DMASA_ICH0,
            .dmafa = AAI_8KHZ_DMAFA_ICH0
        },
    },
    [AAI_MIC2_MUSIC] =
    {
        .name     = "mic2-music",
        .ipcm     = AAI_MIC2_MUSIC,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_srconvin,
        .devexclusion = MIC2_8K_ON | MIC2_16K_ON,
        .regrules = { ICH2_MUSIC, END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_ICH2,
        .intflag  = AAI_ITS_MUSIC_ICH2,
        .enflag   = AAI_ITEN_8KHZ_ICH2|AAI_ITEN_16KHZ_ICH2,
        .mode     = AAI_FIFO_16W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_16KHZ_ICH2,
            .dmasa = AAI_MUSIC_DMASA_ICH2,
            .dmafa = AAI_MUSIC_DMAFA_ICH2
        },
    },
    [AAI_MIC2_16KHZ] =
    {
        .name     = "mic2-16khz",
        .ipcm     = AAI_MIC2_16KHZ,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_voice16kstereo,
        .devexclusion = MIC2_MUSIC_ON,
        .regrules = { ICH2_VOICE, END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_ICH2,
        .intflag  = AAI_ITS_16KHZ_ICH2,
        .enflag   = AAI_ITEN_16KHZ_ICH2,
        .mode     = AAI_FIFO_8W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_16KHZ_ICH2,
            .dmasa = AAI_16KHZ_DMASA_ICH2,
            .dmafa = AAI_16KHZ_DMAFA_ICH2
        },
    },
    [AAI_MIC2_8KHZ] =
    {
        .name     = "mic2-8khz",
        .ipcm     = AAI_MIC2_8KHZ,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_voice8kstereo,
        .devexclusion = MIC2_MUSIC_ON,
        .regrules = { ICH2_VOICE, END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_ICH1,
        .intflag  = AAI_ITS_8KHZ_ICH2,
        .enflag   = AAI_ITEN_8KHZ_ICH2,
        .mode     = AAI_FIFO_4W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_8KHZ_ICH2,
            .dmasa = AAI_8KHZ_DMASA_ICH2,
            .dmafa = AAI_8KHZ_DMAFA_ICH2
        },
    },
    [AAI_MIC3_MUSIC] =
    {
        .name     = "mic3-music",
        .ipcm     = AAI_MIC3_MUSIC,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_srconvin,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_ICH3,
        .intflag  = AAI_ITS_ICH3,
        .enflag   = AAI_ITEN_ICH3,
        .mode     = AAI_FIFO_16W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_MUSIC_ICH3,
            .dmasa = AAI_MUSIC_DMASA_ICH3,
            .dmafa = AAI_MUSIC_DMAFA_ICH3
        },
    },
    [AAI_FBACK_MUSIC] =
    {
        .name     = "back-music",
        .ipcm     = AAI_FBACK_MUSIC,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_music,
        .devexclusion = (BACK_8K_ON|BACK_16K_ON),
        .regrules = { BACK_MUSIC, END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_BACK,
        .intflag  = AAI_ITS_MUSIC_BACK,
        .enflag   = AAI_ITEN_8KHZ_BACK|AAI_ITEN_16KHZ_BACK,
        .mode     = AAI_FIFO_16W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_16KHZ_BACK,
            .dmasa = AAI_MUSIC_DMASA_BACK,
            .dmafa = AAI_MUSIC_DMAFA_BACK
        },
    },
    [AAI_FBACK_16KHZ] =
    {
        .name     = "back-16khz",
        .ipcm     = AAI_FBACK_16KHZ,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_voice16kstereo,
        .devexclusion = BACK_MUSIC_ON,
        .regrules = { BACK_VOICE, END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_BACK,
        .intflag  = AAI_ITS_16KHZ_BACK,
        .enflag   = AAI_ITEN_16KHZ_BACK,
        .mode     = AAI_FIFO_8W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_16KHZ_BACK,
            .dmasa = AAI_16KHZ_DMASA_BACK,
            .dmafa = AAI_16KHZ_DMAFA_BACK
        },
    },
    [AAI_FBACK_8KHZ] = 
    {
        .name           = "back-8khz",
        .ipcm           = AAI_FBACK_8KHZ,
        .ops            = &aai_pcm_ops,
        .hw             = &aai_pcm_hw_voice8kstereo,
        .devexclusion   = BACK_MUSIC_ON,
        .regrules       = { BACK_VOICE, END_OF_RULES, },
        .direction      = AAI_RX,
        .dmaflag        = AAI_DMACTL_BACK,
        .intflag        = AAI_ITS_8KHZ_BACK,
        .enflag         = AAI_ITEN_8KHZ_BACK,
        .mode           = AAI_FIFO_4W_32b,
        .fifo[0]        =
        {
            .hfifo = AAI_8KHZ_BACK,
            .dmasa = AAI_8KHZ_DMASA_BACK,
            .dmafa = AAI_8KHZ_DMAFA_BACK
        },
    },
    [AAI_PCM0_SO1] = 
    {
        .name     = "pcm0-out1",
        .ipcm     = AAI_PCM0_SO1,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_PCM0,
        .intflag  = AAI_ITS_SO1_PCM0,
        .enflag   = AAI_ITEN_SO1_PCM0,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_SO1_PCM0,
            .dmasa = AAI_SO1_DMASA_PCM0,
            .dmafa = AAI_SO1_DMAFA_PCM0
        },
    },
    [AAI_PCM0_SI1] =
    {
        .name     = "pcm0-in1",
        .ipcm     = AAI_PCM0_SI1,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_PCM0,
        .intflag  = AAI_ITS_SI1_PCM0,
        .enflag   = AAI_ITEN_SI1_PCM0,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_SI1_PCM0,
            .dmasa = AAI_SI1_DMASA_PCM0,
            .dmafa = AAI_SI1_DMAFA_PCM0
        },
    },
    [AAI_PCM0_SO2] =
    {
        .name     = "pcm0-out2",
        .ipcm     = AAI_PCM0_SO2,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_PCM0,
        .intflag  = AAI_ITS_SO2_PCM0,
        .enflag   = AAI_ITEN_SO2_PCM0,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_SO2_PCM0,
            .dmasa = AAI_SO2_DMASA_PCM0,
            .dmafa = AAI_SO2_DMAFA_PCM0
        },
    },
    [AAI_PCM0_SI2] =
    {
        .name     = "pcm0-in2",
        .ipcm     = AAI_PCM0_SI2,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_PCM0,
        .intflag  = AAI_ITS_SI2_PCM0,
        .enflag   = AAI_ITEN_SI2_PCM0,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_SI2_PCM0,
            .dmasa = AAI_SI2_DMASA_PCM0,
            .dmafa = AAI_SI2_DMAFA_PCM0
        },
    },
    [AAI_PCM0_SO3] =
    {
        .name     = "pcm0-out3",
        .ipcm     = AAI_PCM0_SO3,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_PCM0,
        .intflag  = AAI_ITS_SO3_PCM0,
        .enflag   = AAI_ITEN_SO3_PCM0,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        {
            .hfifo = AAI_SO3_PCM0,
            .dmasa = AAI_SO3_DMASA_PCM0,
            .dmafa = AAI_SO3_DMAFA_PCM0
        },
    },
    [AAI_PCM0_SI3] =
    {
        .name     = "pcm0-in3",
        .ipcm     = AAI_PCM0_SI3,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_PCM0,
        .intflag  = AAI_ITS_SI3_PCM0,
        .enflag   = AAI_ITEN_SI3_PCM0,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_SI3_PCM0,
            .dmasa = AAI_SI3_DMASA_PCM0,
            .dmafa = AAI_SI3_DMAFA_PCM0
        },
    },
    [AAI_PCM1_SO1] =
    {
        .name     = "pcm1-out1",
        .ipcm     = AAI_PCM1_SO1,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_PCM1,
        .intflag  = AAI_ITS_SO1_PCM1,
        .enflag   = AAI_ITEN_SO1_PCM1,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_SO1_PCM1,
            .dmasa = AAI_SO1_DMASA_PCM1,
            .dmafa = AAI_SO1_DMAFA_PCM1
        },
    },
    [AAI_PCM1_SI1] =
    {
        .name     = "pcm1-in1",
        .ipcm     = AAI_PCM1_SI1,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_PCM1,
        .intflag  = AAI_ITS_SI1_PCM1,
        .enflag   = AAI_ITEN_SI1_PCM1,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_SI1_PCM1,
            .dmasa = AAI_SI1_DMASA_PCM1,
            .dmafa = AAI_SI1_DMAFA_PCM1
        },
    },
    [AAI_PCM1_SO2] =
    {
        .name     = "pcm1-out2",
        .ipcm     = AAI_PCM1_SO2,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_TX,
        .dmaflag  = AAI_DMACTL_PCM1,
        .intflag  = AAI_ITS_SO2_PCM1,
        .enflag   = AAI_ITEN_SO2_PCM1,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_SO2_PCM1,
            .dmasa = AAI_SO2_DMASA_PCM1,
            .dmafa = AAI_SO2_DMAFA_PCM1
        },
    },
    [AAI_PCM1_SI2] =
    {
        .name     = "pcm1-in2",
        .ipcm     = AAI_PCM1_SI2,
        .ops      = &aai_pcm_ops,
        .hw       = &aai_pcm_hw_pcm,
        .devexclusion = 0,
        .regrules = { END_OF_RULES, },
        .direction= AAI_RX,
        .dmaflag  = AAI_DMACTL_PCM1,
        .intflag  = AAI_ITS_SI2_PCM1,
        .enflag   = AAI_ITEN_SI2_PCM1,
        .mode     = AAI_FIFO_2W_32b,
        .fifo[0]  =
        { 
            .hfifo = AAI_SI2_PCM1,
            .dmasa = AAI_SI2_DMASA_PCM1,
            .dmafa = AAI_SI2_DMAFA_PCM1
        },
    },
    /*
       [AAI_BINARY] = {
       .name     = "mic-binary",
       .ipcm     = AAI_BINARY,
       .ops      = &aai_pcm_ops,
       .hw       = &aai_pcm_hw_music,
       .devexclusion = 0,
       .regrules = {BINARY_SPDIF,END_OF_RULES,},
       .direction= AAI_RX,
       .dmaflag  = 0,
       .intflag  = AAI_ITS_SPDIF_BIN,
       .enflag   = AAI_ITEN_BINARY,
       .fifo[0]  = { .hfifo = 0,
       .dmasa = AAI_BINARY_DMASA,
       .dmafa = AAI_BINARY_DMAFA },
       },
       */
};
 
