/**
 * @file   aai_codec_p6i.c
 * @brief  P6i internal codec control
 *
 * @author adrien.charruel@parrot.com
 * @date   2010-11-24
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *      Parrot Advanced Audio Interface Driver
 */

#include "aai.h"
#include "aai_hw.h"
#include "aai_ioctl.h"

/**
 * @brief Latch codec
 *        This is required to set new codec values
 **/
void aai_codec_latch(int blocking)
{
    uint32_t reg;

    /* latch codec configuration */
    reg = readl(PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL1);
    reg |= P6I_SYS_CODEC_LATCH_MASK;
    writel(reg, PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL1);

    /* 2 ms delay then release latch, either blocking (for boot) or not */
    if (blocking)
        mdelay(2);
    else
        msleep(2);
    reg &= ~P6I_SYS_CODEC_LATCH_MASK;
    writel(reg, PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL1);
}

/**
 * @brief P6i internal codec record selection
 *
 * @param aai  aai card to configure
 * @param val  record selection value
 *
 * @return error code
 **/
int aai_codec_set_recsel(struct card_data_t *aai, int val)
{
    uint32_t reg = readl(PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL1);

    if (val >= RECSEL_MAX_VALUE)
    {
        aai_err(aai->dev, "%s: value out of range\n", __FUNCTION__);
        return -EINVAL;
    }

    reg &= ~P6I_SYS_CODEC_RECSEL_MASK;  /* clear recsel values */
    reg |= val | (val << 8);            /* set new values */
    writel(reg, PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL1);
    aai_codec_latch(0);

    return 0;
}

/**
 * @brief Set p6i internal codec i2s sampling frequency
 *
 * @param aai  aai card to configure
 * @param val  i2s sampling frequency
 *
 * @return error code
 **/
int aai_codec_set_i2sfreq(struct card_data_t *aai, int val)
{
    uint32_t reg = readl(PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL1);

    if (val >= I2SFREQ_MAX_VALUE)
    {
        aai_err(aai->dev, "%s: value out of range\n", __FUNCTION__);
        return -EINVAL;
    }

    reg &= ~P6I_SYS_CODEC_I2SFS_MASK;   /* clear recsel values */
    reg |= val << 17;                   /* set new values */
    writel(reg, PARROT6_VA_SYS + _P6I_SYS_CODEC_CTL1);
    aai_codec_latch(0);

    return 0;
}

