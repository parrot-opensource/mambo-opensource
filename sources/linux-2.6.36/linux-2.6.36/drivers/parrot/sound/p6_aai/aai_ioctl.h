/**
 * @file   aai_ioctl.h
 * @brief  AAI ioctl header
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

#ifndef INCONCE_AAI_IO_H
#define INCONCE_AAI_IO_H

#include <linux/ioctl.h>

#define AAI_IOW_AAISLAVE            _IOW('p', 1, int)    /*<! DAC i2s interface Master/Slave */
#define AAI_IOW_I2SSLAVE            _IOW('p', 2, int)    /*<! Master/Slave */
#define AAI_IOW_SYNCEDGE            _IOW('p', 3, int)
#define AAI_IOW_MASTERCLK           _IOW('p', 4, int)
enum
{
   MASTERCLK_2 = 0,
   MASTERCLK_4 = 1,
   MASTERCLK_8 = 2,
};

#define AAI_IOW_AUXSYNC             _IOW('p', 5, int)    /*<! 8 and 16 Khz voice channels synchronization */
enum
{
   AUXSYNC_NONE = 0,
   AUXSYNC_PARTIAL,
   AUXSYNC_DAC,
};

#define AAI_IOW_BACKSRC             _IOW('p', 6, int)    /*<! Feedback source */
enum
{
   BACKSRC_OUT0 = 0, /*<! Feedback source is output 0 */
   BACKSRC_OUT1      /*<! Feedback source is output 1 */
};

#define AAI_IOW_PCM0MAST             _IOW('p', 12, int)
#define AAI_IOW_PCM1MAST             _IOW('p', 13, int)
enum
{
   PCM_SLAVE = 0,
   PCM_MASTER
};
#define AAI_IOW_SRCONVOUT            _IOW('p', 14, int)
enum
{
   SRCONV_IN = 0,
   SRCONV_OUT
};
#define AAI_IOW_ICH1SRC             _IOW('p', 16, int)
enum
{
   ICH1SEL_I2S = 0,
   ICH1SEL_MOST
};
#define AAI_IOW_ICH2SRC             _IOW('p', 17, int)
enum
{
   ICH2SRC_ACIN2 = 0,
   ICH2SRC_ACIN3,
   ICH2SRC_ACIN4,
   ICH2SRC_MOST
};
#define AAI_IOW_ICH3SRC             _IOW('p', 18, int)
enum
{
   ICH3SRC_ACIN5 = 0,
   ICH3SRC_ACIN6,
   ICH3SRC_ACIN7,
   ICH3SRC_MOST
};
#define AAI_IOW_ASYNCI2S            _IOW('p', 19, int)
enum
{
   ICH2_ONLY = 0,
   ICH3_ONLY,
   ICH2_ICH3_WITH_ICH2FORMAT,
   ICH2_ICH3_WITH_ICH3FORMAT
};

/**
 * I2S format
 */
struct aai_i2s_config
{
   unsigned int  msbphase;
   unsigned int  leftframe;
   unsigned int  rightjust;
   unsigned int  lsbfirst;
};
#define AAI_IOW_I2S_FORMAT_ICH0     _IOW('p', 21, int)
#define AAI_IOW_I2S_FORMAT_ICH1     _IOW('p', 22, int)
#define AAI_IOW_I2S_FORMAT_1        _IOW('p', 23, int)
#define AAI_IOW_I2S_FORMAT_2        _IOW('p', 24, int)
#define AAI_IOW_I2S_FORMAT_3        _IOW('p', 25, int)

#define AAI_IOR_I2S_FORMAT_ICH0     _IOW('p', 26, int)
#define AAI_IOR_I2S_FORMAT_ICH1     _IOW('p', 27, int)
#define AAI_IOR_I2S_FORMAT_1        _IOW('p', 28, int)
#define AAI_IOR_I2S_FORMAT_2        _IOW('p', 29, int)
#define AAI_IOR_I2S_FORMAT_3        _IOW('p', 30, int)

#define AAI_IOW_SPDIF_ALTPAD        _IOW('p', 31, int)

/*
 * warning: this values are linked to hardware
 */
typedef enum
{
   I2S_ICH_ACIN_F1 = 0,
   I2S_ICH_ACIN_F2,
   I2S_ICH_ACIN_F3,
   I2S_ICH_ACIN_SPDIF
} aai_i2s_format;

struct aai_i2sin_sources
{
   aai_i2s_format  first;
   aai_i2s_format  second;
   aai_i2s_format  third;
};
#define AAI_IOW_I2S_ICH2            _IOW('p', 32, int)
#define AAI_IOW_I2S_ICH3            _IOW('p', 33, int)
#define AAI_IOR_I2S_ICH2            _IOW('p', 34, int)
#define AAI_IOR_I2S_ICH3            _IOW('p', 35, int)

#define AAI_IOW_I2S_FORMAT_ASYNC1   _IOW('p', 36, int)
#define AAI_IOW_I2S_FORMAT_ASYNC2   _IOW('p', 37, int)
#define AAI_IOW_I2S_FORMAT_ASYNC3   _IOW('p', 38, int)

enum
{
    FORMAT_SYNC_WITH_DAC = 0,
    FORMAT_ASYNC_WITH_DAC
};

/**
 * PCM configuration
 */
#define AAI_IOW_PCM0_SHIFT2         _IOW('p', 39, int)
#define AAI_IOW_PCM0_SHIFT3         _IOW('p', 40, int)
#define AAI_IOW_PCM0_OKI            _IOW('p', 41, int)
#define AAI_IOW_PCM0_1MHZ           _IOW('p', 42, int)

#define AAI_IOW_PCM1_SHIFT2         _IOW('p', 43, int)
#define AAI_IOW_PCM1_OKI            _IOW('p', 44, int)
#define AAI_IOW_PCM1_1MHZ           _IOW('p', 45, int)

#define AAI_IOW_DMACNT_MUSIC        _IOW('p', 46, int)
#define AAI_IOW_DMACNT_AUX          _IOW('p', 47, int)
#define AAI_IOW_DMACNT_VOICE        _IOW('p', 48, int)

#define AAI_IOW_LOUDREF_OUT0        _IOW('p', 49, int)
#define AAI_IOW_LOUDREF_OUT1        _IOW('p', 50, int)

#define AAI_IOW_PCM_CYCLESHIGH      _IOW('p', 51, int)
#define AAI_IOW_PCM_CYCLESLOW       _IOW('p', 52, int)

#define AAI_IOR_STATUS              _IOR('p', 80, int)
#define AAI_IOR_DMACNT_MUSIC        _IOR('p', 81, int)
#define AAI_IOR_DMACNT_AUX          _IOR('p', 82, int)
#define AAI_IOR_DMACNT_VOICE        _IOR('p', 83, int)
#define AAI_IOR_SRCONV_RATE         _IOW('p', 84, int)

#define AAI_IOWR_READREG            _IOWR('p', 100, int)
#define AAI_IOWR_SRCONV_RATE        _IOWR('p', 101, int)

#define AAI_IOW_STARTGROUP          _IOW('p', 110, int)
#define AAI_IOW_SYNC                _IOW('p', 111, int)

/**
 * ADC management
 */
#define AAI_IOW_ADC_MONITORING_START _IOW('p', 112, int)
#define AAI_IOR_ADC_MONITORING_DATA  _IOR('p', 113, int)
#define AAI_NB_ADC 8

/**
 * P6i internal codec configuration
 */

/**
 * P6i internal codec record selection
 */
enum
{
    RECSEL_LINE_INPUT1_SE,      /*<! line input 1 (single ended) */
    RECSEL_LINE_INPUT2,         /*<! line input 2 */
    RECSEL_MICROPHONE_SE,       /*<! microphone input (single ended) */
    RECSEL_LINE_INPUT1_DIFF,    /*<! line input 1 (differential) */
    RECSEL_MICROPHONE_DIFF,     /*<! microphone input (differential) */
    RECSEL_MAX_VALUE            /*<! max value */
};

/**
 * P6i internal codec i2s frequencies
 */
enum
{
    I2SFREQ_8000HZ,         /*<! 8 kHz */
    I2SFREQ_11025HZ,        /*<! 11.025 kHz */
    I2SFREQ_12000HZ,        /*<! 12 kHz */
    I2SFREQ_16000HZ,        /*<! 16 kHz */
    I2SFREQ_220050Z,        /*<! 22.05 kHz */
    I2SFREQ_24000HZ,        /*<! 24 kHz */
    I2SFREQ_32000HZ,        /*<! 32 kHz */
    I2SFREQ_44100HZ,        /*<! 44.1 kHz */
    I2SFREQ_48000HZ,        /*<! 48 kHz */
    I2SFREQ_88200HZ,        /*<! 88.2 kHz */
    I2SFREQ_96000HZ,        /*<! 96 kHz */
    I2SFREQ_192000HZ,       /*<! 192 kHz */
    I2SFREQ_MAX_VALUE       /*<! max value */
};

#define AAI_IOW_CODEC_REC_SELECTION     _IOW('p', 150, int)
#define AAI_IOW_CODEC_I2S_FREQ          _IOW('p', 151, int)

#endif

