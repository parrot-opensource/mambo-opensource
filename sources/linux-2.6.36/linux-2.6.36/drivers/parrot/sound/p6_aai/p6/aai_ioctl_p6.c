/**
 * @file   aai_ioctl.c
 * @brief  AAI ioctl management
 *
 * @author gregoire.etienne@parrot.com
 * @date   2008-10-02
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *      Parrot Advanced Audio Interface Driver
 */

#include "aai.h"
#include "aai_hw.h"
#include "aai_hw_p6.h"
#include "aai_ioctl.h"

/**
 * Get a data from user space
 *
 * @param src ptr to userspace data
 * @param dst ptr to kernelspace data
 * @param size of data
 *
 * @return error code
 */
static int get_data(void __user * src, void* dst, int size)
{
   int err = 0;
   if (copy_from_user(dst, src, size))
            err = -EFAULT;

   return err;
}

/**
 * Put data to userspace
 *
 * @param src ptr to userspace data
 * @param dst ptr to kernelspace data
 * @param size of data
 *
 * @return error code
 */
static int put_data(void *src, void __user *dst, int size)
{
   int err = 0;
   if (copy_to_user(dst, src, size))
      err = -EFAULT;

   return err;
}

/**
 * AAI Driver IO control function
 */
static int aai_hwdep_ioctl(struct snd_hwdep *hwdep,
                           struct file      *file,
                           unsigned int     cmd,
                           unsigned long    arg)
{
    int err = -EINVAL;
    unsigned int reg = 0, val = 0;
    struct card_data_t *aai = hwdep->private_data;

    switch (cmd)
    {
        /** <ul>
         */

        /** <li><b>AAI_IOWR_READREG:</b><br>
         * Get AAI register value<br>
         * Param  : AAI register offset (uint)<br>
         * Return : register value (uint)
         * <hr>
         */
        case AAI_IOWR_READREG:
            err = get_data((void __user *) arg, &reg, sizeof(unsigned int));
            if (err == 0)
            {
                val = aai_readreg(aai, reg);
                err = put_data(&val, (void __user *)arg, sizeof(unsigned int));
            }
            break;

        /** <li><b>AAI_IOR_STATUS:</b><br>
         * Print AAI registers on stdout<br>
         * Param  : none (uint)<br>
         * Return : none
         * <hr>
         */
        case AAI_IOR_STATUS:
            aai_hw_status(aai);
            err = put_data(&val, (void __user *)arg, sizeof(unsigned int));
            break;

        /** <li><b>AAI_IOW_SRCONVOUT:</b><br>
         * Set sample rate converter direction. Apply SRConv on inputs(ich2-music and ich3-music) or outputs(spk-out0 and spk-out1)<br>
         * Param  : integer value (uint):
         * <ul> <li>SRCONV_IN
         *      <li>SRCONV_OUT
         * </ul>
         * Return : none<br>
         * <hr>
         */
        case AAI_IOW_SRCONVOUT:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                err = aai_hw_set_srconvdir(aai, val);
            break;

        /** <li><b>AAI_IOR_SRCONV_RATE:</b><br>
         * Get sample rate converter rate<br>
         * Param  : none(uint)<br>
         * Return : rate value with a precision of 0,01Hz(ie 2205010 -> 22050,10Hz.(uint)<br>
         * <hr>
         */
        case AAI_IOR_SRCONV_RATE:
            val = aai_hw_get_srconvrate(aai);
            err = put_data(&val, (void __user *)arg, sizeof(unsigned int));
            break;

        /** <li><b>AAI_IOWR_SRCONV_RATE:</b><br>
         * Set sample rate converter rate<br>
         * Param  : User rate value with a precision of 0,01Hz(ie 2205010 -> 22050,10Hz.(uint)<br>
         * Return : Memorized rate value with a precision of 0,01Hz(ie 2205010 -> 22050,10Hz.(uint)<br>
         * <hr>
         */
        case AAI_IOWR_SRCONV_RATE:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
            {
                err = aai_hw_set_srconvrate(aai, val); /* XXX use hw function */
                val = aai_hw_get_srconvrate(aai);
                err = put_data(&val, (void __user *)arg, sizeof(unsigned int));
            }
            break;

        /** <li><b>AAI_IOW_AAISLAVE:</b><br>
         * AAI Master/Slave bit. Default value is master(0)<br>
         * Param  : integer value  (uint):
         * <ul> <li>0=Standalone Sampling rate
         *      <li>1=Sampling rate synchronized by a MOST device or an I2S external device
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_AAISLAVE:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_CFG, AAI_CFG_AAI_SLAVE, (!!val));
            break;

        /** <li><b>AAI_IOW_I2SSLAVE:</b><br>
         * AAI Master/Slave bit. Default value is master(0)<br>
         * Param  : integer value (uint):
         * <ul> <li>0=Slave on a MOST device
         *      <li>1=Slave on an external I2S
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_I2SSLAVE:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_CFG, AAI_CFG_I2S_SLAVE, (!!val));
            break;

        /** <li><b>AAI_IOW_SYNCEDGE:</b><br>
         * Master clock edge polarity with respect to serial bit clock<br>
         * Param  : integer value(uint):
         * <ul> <li>0=Master clock falling edge is synchronous with serial bit clock and frame edges
         *      <li>1=Master clock rising edge is synchronous with serial bit clock and frame edges
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_SYNCEDGE:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_CFG, AAI_CFG_SYNC_EDGE, (!!val));
            break;

        /** <li><b>AAI_IOW_MASTERCLK:</b><br>
         * <br>
         * Param  : integer value(uint):
         * <ul> <li>MASTERCLK_2=Master clock output is twice the rate of external serial bit clock
         *      <li>MASTERCLK_4=Master clock output is four times the rate of external serial bit clock
         *      <li>MASTERCLK_8=Master clock output is eight times the rate of external serial bit clock
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_MASTERCLK:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
            {
                aai_set_nbit(aai, AAI_CFG, 0x3, AAI_CFG_MASTER_CLK_SHIFT, val);
            }
            break;

        /** <li><b>AAI_IOW_AUXSYNC:</b><br>
         * Synchronization of the filling of multimedia auxiliary input/output FIFO<br>
         * Param  : integer value(uint):
         * <ul> <li>AUXSYNC_NONE=No filling synchronization at all
         *      <li>AUXSYNC_PARTIAL=The multimedia auxiliary FIFO have their filling and flags synchronized only between them
         *      <li>AUXSYNC_DAC=The multimedia auxiliary FIFO have their filling and flags synchronized with FIFO synchronous to the DAC
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_AUXSYNC:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_set_nbit(aai, AAI_CFG, 0x3, AAI_CFG_AUX_SYNC_SHIFT, val);
            break;

        /** <li><b>AAI_IOW_BACKSRC:</b><br>
         * Usage of the feedback channels<br>
         * Param  : integer value
         * <ul> <li>BACKSRC_OUT0=Feedback source is output 0
         *      <li>BACKSRC_OUT1=Feedback source is output 1
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_BACKSRC:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_CFG, AAI_CFG_OUT1_BACK, (!!val));
            break;

        /*  SPDIF_TOLERANCE XXX */
        /*  SPDIF_RAW */
        /*  SPDIF_BINARY */
        /*  MIC_FILTER_P5P */

        /** <li><b>AAI_IOW_ICH1SRC:</b><br>
         * Usage of the feedback channels<br>
         * Param  : integer value
         * <ul> <li>ICH1SEL_I2S=Feedback source is output 0
         *      <li>ICH1SEL_MOST=Feedback source is output 1
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_ICH1SRC:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_ITEN, AAI_ITEN_ICH1_SEL, (!!val));
            break;

        /** <li><b>AAI_IOW_ICH2SRC:</b><br>
         * Usage of the feedback channels<br>
         * Param  : integer value
         * <ul> <li>ICH2SRC_ACIN2=AC_IN2 source with a format defined and selected in the AAI_I2S_FORMAT register
         *      <li>ICH2SRC_ACIN3=AC_IN3 source with a format defined and selected in the AAI_I2S_FORMAT register
         *      <li>ICH2SRC_ACIN3=AC_IN4 source with a format defined and selected in the AAI_I2S_FORMAT register
         *      <li>ICH2SRC_MOST=MOST
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_ICH2SRC:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_set_nbit(aai, AAI_ITEN, 0x3, AAI_ITEN_ICH2_SEL_SHIFT, val);
            break;

        /** <li><b>AAI_IOW_ICH3SRC:</b><br>
         * Usage of the feedback channels<br>
         * Param  : integer value
         * <ul> <li>ICH3SRC_ACIN5=AC_IN5 source with a format defined and selected in the AAI_I2S_FORMAT register
         *      <li>ICH3SRC_ACIN6=AC_IN6 source with a format defined and selected in the AAI_I2S_FORMAT register
         *      <li>ICH3SRC_ACIN7=AC_IN7 source with a format defined and selected in the AAI_I2S_FORMAT register
         *      <li>ICH3SRC_MOST=MOST
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_ICH3SRC:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_set_nbit(aai, AAI_ITEN, 0x3, AAI_ITEN_ICH3_SEL_SHIFT, val);
            break;

        /** <li><b>AAI_IOW_ASYNCI2S:</b><br>
         * Usage of the feedback channels<br>
         * Param  : integer value
         * <ul> <li>ICH2_ONLY=Only ICH2 can be validated for an asynchronous format (if the SRC_OUT bit is not set)
         *      <li>ICH3_ONLY=Only ICH3 can be validated for an asynchronous format (if the SRC_OUT bit is not set)
         *      <li>ICH2_ICH3_WITH_ICH2FORMAT=ICH2 and ICH3 are using the same format selected by ICH2_SEL, that must be set as asynchronous
         *      <li>ICH2_ICH3_WITH_ICH3FORMAT=ICH2 and ICH3 are using the same format selected by ICH3_SEL, that must be set as asynchronous
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_ASYNCI2S:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_set_nbit(aai, AAI_ITEN, 0x3, AAI_ITEN_ASYNC_I2S_SHIFT, val);
            break;

       /*-----------------------------------------------------------------------------
        * ICH0 i2s format
        *---------------------------------------------------------------------------*/

        /** <li><b>AAI_IOW_I2S_FORMAT_ICH0:</b><br>
         * Set i2s input format configuration for ICH0<br>
         * Param  : structure (aai_i2s_config structure)
         * \code
        struct aai_i2s_config{
            unsigned int  msbphase;
            unsigned int  leftframe;
            unsigned int  rightjust;
            unsigned int  lsbfirst;
        };
         * \endcode
         * <ul> <li>msbphase: 0:First bit is one clock after frame change, 1:First bit is in phase with frame change (Matsushita style)
         *      <li>leftframe:Left channel sync polarity, 0:frame = 0 for left channel, 1:frame = 1 for left channel.
         *      <li>rightjust: unavailable
         *      <li>lsbfirst: unavailable
         * </ul>
         * <hr>
         */
        case AAI_IOW_I2S_FORMAT_ICH0:
            {
                struct aai_i2s_config f;
                err = get_data((void __user *)arg, &f, sizeof(struct aai_i2s_config));
                if (err == 0)
                {
                    aai_setbit(aai, AAI_CFG, AAI_CFG_MSBPHASE , !!(f.msbphase));
                    aai_setbit(aai, AAI_CFG, AAI_CFG_LEFTFRAME, !!(f.leftframe));
                }
            }
            break;

        /** <li><b>AAI_IOR_I2S_FORMAT_ICH0:</b><br>
         * Get i2s input format configuration for ICH0<br>
         * Param  : structure (aai_i2s_config structure) (see AAI_IOW_I2S_FORMAT_ICH0).
         * <hr>
         */
        case AAI_IOR_I2S_FORMAT_ICH0:
            {
                struct aai_i2s_config f;
                f.msbphase  = aai_getbit(aai, AAI_CFG, AAI_CFG_MSBPHASE);
                f.leftframe = aai_getbit(aai, AAI_CFG, AAI_CFG_LEFTFRAME);
                f.rightjust = f.lsbfirst = 0xFFFFFFFF;
                err = put_data(&f, (void __user *)arg, sizeof(struct aai_i2s_config));
            }
            break;

        /** <li><b>AAI_IOW_I2S_FORMAT_ICH1:</b><br>
         * Set i2s input format configuration for ICH1<br>
         * Param  : structure (aai_i2s_config structure)
         * \code
        struct aai_i2s_config{
            unsigned int  msbphase;
            unsigned int  leftframe;
            unsigned int  rightjust;
            unsigned int  lsbfirst;
        };
            \endcode
         * <ul> <li>msbphase: 0:First bit is one clock after frame change, 1:First bit is in phase with frame change (Matsushita style)
         *      <li>leftframe:Left channel sync polarity, 0:frame = 0 for left channel, 1:frame = 1 for left channel.
         *      <li>rightjust: 0:Serial data bits are left justified relative to frame changes, 1:Serial data bits are right justified relative to frame changes
         *      <li>lsbfirst: 0:the MSB is sent first, 1:the LSB is sent first
         * </ul>
         * <hr>
         */
        case AAI_IOW_I2S_FORMAT_ICH1:
            {
                struct aai_i2s_config f;
                err = get_data((void __user *)arg, &f, sizeof(struct aai_i2s_config));
                if (err == 0)
                {
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_AUX_MSBPHASE , !!(f.msbphase));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_AUX_LEFTFRAME, !!(f.leftframe));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_AUX_RIGHTJUST, !!(f.rightjust));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_AUX_LSBFIRST , !!(f.lsbfirst));
                }
            }
            break;

        /** <li><b>AAI_IOR_I2S_FORMAT_ICH1:</b><br>
         * Get i2s input format configuration for ICH1<br>
         * Param  : structure (aai_i2s_config structure) (see AAI_IOW_I2S_FORMAT_ICH1).
         * <hr>
         */
        case AAI_IOR_I2S_FORMAT_ICH1:
            {
                struct aai_i2s_config f;
                f.msbphase  = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_AUX_MSBPHASE);
                f.leftframe = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_AUX_LEFTFRAME);
                f.rightjust = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_AUX_RIGHTJUST);
                f.lsbfirst  = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_AUX_LSBFIRST);
                err = put_data(&f, (void __user *)arg, sizeof(struct aai_i2s_config));
            }
            break;

        /** <li><b>AAI_IOW_I2S_FORMAT_1:</b><br>
         * Set i2s input format configuration for FORMAT1<br>
         * Param  : structure (aai_i2s_config structure)
         * \code
        struct aai_i2s_config{
            unsigned int  msbphase;
            unsigned int  leftframe;
            unsigned int  rightjust;
            unsigned int  lsbfirst;
        };
            \endcode
         * <ul> <li>msbphase: 0:First bit is one clock after frame change, 1:First bit is in phase with frame change (Matsushita style)
         *      <li>leftframe:Left channel sync polarity, 0:frame = 0 for left channel, 1:frame = 1 for left channel.
         *      <li>rightjust: 0:Serial data bits are left justified relative to frame changes, 1:Serial data bits are right justified relative to frame changes
         *      <li>lsbfirst: 0:the MSB is sent first, 1:the LSB is sent first
         * </ul>
         * <hr>
         */
        case AAI_IOW_I2S_FORMAT_1:
            {
                struct aai_i2s_config f;
                err = get_data((void __user *)arg, &f, sizeof(struct aai_i2s_config));
                if (err == 0)
                {
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_1_MSBPHASE , !!(f.msbphase));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_1_LEFTFRAME, !!(f.leftframe));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_1_RIGHTJUST, !!(f.rightjust));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_1_LSBFIRST , !!(f.lsbfirst));
                }
            }
            break;

        /** <li><b>AAI_IOR_I2S_FORMAT_1:</b><br>
         * Get i2s input format configuration for FORMAT1<br>
         * Param  : structure (aai_i2s_config structure) (see AAI_IOW_I2S_FORMAT_1).
         * <hr>
         */
        case AAI_IOR_I2S_FORMAT_1:
            {
                struct aai_i2s_config f;
                f.msbphase  = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_1_MSBPHASE);
                f.leftframe = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_1_LEFTFRAME);
                f.rightjust = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_1_RIGHTJUST);
                f.lsbfirst  = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_1_LSBFIRST);
                err = put_data(&f, (void __user *)arg, sizeof(struct aai_i2s_config));
            }
            break;

        /** <li><b>AAI_IOW_I2S_FORMAT_2:</b><br>
         * Set i2s input format configuration for FORMAT2<br>
         * Param  : structure (aai_i2s_config structure) (see AAI_IOW_I2S_FORMAT_1).
         * <hr>
         */
        case AAI_IOW_I2S_FORMAT_2:
            {
                struct aai_i2s_config f;
                err = get_data((void __user *)arg, &f, sizeof(struct aai_i2s_config));
                if (err == 0)
                {
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_2_MSBPHASE , !!(f.msbphase));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_2_LEFTFRAME, !!(f.leftframe));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_2_RIGHTJUST, !!(f.rightjust));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_2_LSBFIRST , !!(f.lsbfirst));
                }
            }
            break;

        /** <li><b>AAI_IOR_I2S_FORMAT_2:</b><br>
         * Get i2s input format configuration for FORMAT2<br>
         * Param  : structure (aai_i2s_config structure) (see AAI_IOW_I2S_FORMAT_1).
         * <hr>
         */
        case AAI_IOR_I2S_FORMAT_2:
            {
                struct aai_i2s_config f;
                f.msbphase  = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_2_MSBPHASE);
                f.leftframe = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_2_LEFTFRAME);
                f.rightjust = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_2_RIGHTJUST);
                f.lsbfirst  = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_2_LSBFIRST);
                err = put_data(&f, (void __user *)arg, sizeof(struct aai_i2s_config));
            }
            break;

        /** <li><b>AAI_IOW_I2S_FORMAT_3:</b><br>
         * Set i2s input format configuration for FORMAT3<br>
         * Param  : structure (aai_i2s_config structure) (see AAI_IOW_I2S_FORMAT_1).
         * <hr>
         */
        case AAI_IOW_I2S_FORMAT_3:
            {
                struct aai_i2s_config f;
                err = get_data((void __user *)arg, &f, sizeof(struct aai_i2s_config));
                if (err == 0)
                {
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_3_MSBPHASE , !!(f.msbphase));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_3_LEFTFRAME, !!(f.leftframe));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_3_RIGHTJUST, !!(f.rightjust));
                    aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_3_LSBFIRST , !!(f.lsbfirst));
                }
            }
            break;

        /** <li><b>AAI_IOR_I2S_FORMAT_3:</b><br>
         * Get i2s input format configuration for FORMAT3<br>
         * Param  : structure (aai_i2s_config structure) (see AAI_IOW_I2S_FORMAT_1).
         * <hr>
         */
        case AAI_IOR_I2S_FORMAT_3:
            {
                struct aai_i2s_config f;
                f.msbphase  = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_3_MSBPHASE);
                f.leftframe = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_3_LEFTFRAME);
                f.rightjust = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_3_RIGHTJUST);
                f.lsbfirst  = aai_getbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_3_LSBFIRST);
                err = put_data(&f, (void __user *)arg, sizeof(struct aai_i2s_config));
            }
            break;

        /** <li><b>AAI_IOW_I2S_ICH2:</b><br>
         * Set Format selection for ICH2 sources<br>
         * Param  : structure (aai_i2sin_sources)
         * \code
        struct aai_i2sin_sources{
           aai_i2s_format  first;
           aai_i2s_format  second;
           aai_i2s_format  third;
        };
            \endcode
         * <ul> <li>first: format of the first ICH2 source: AC_IN2
         *      <li>second: format of the second ICH2 source: AC_IN3
         *      <li>third: format of the third ICH2 source: AC_IN4
         * </ul>
         * these fields can take following values : I2S_ICH_ACIN_F1(format1), I2S_ICH_ACIN_F2(format2), I2S_ICH_ACIN_F3(format3) or I2S_ICH_ACIN_SPDIF
         * <hr>
         */
        case AAI_IOW_I2S_ICH2:
            {
                struct aai_i2sin_sources s;
                err = get_data((void __user *)arg, &s, sizeof(struct aai_i2sin_sources));
                if (err == 0)
                {
                    aai_set_nbit(aai, AAI_I2S_FORMAT, 0x3, AAI_I2S_IHC2_IN1_SHIFT, s.first);
                    aai_set_nbit(aai, AAI_I2S_FORMAT, 0x3, AAI_I2S_IHC2_IN2_SHIFT, s.second);
                    aai_set_nbit(aai, AAI_I2S_FORMAT, 0x3, AAI_I2S_IHC2_IN3_SHIFT, s.third);
                }
            }
            break;

        /** <li><b>AAI_IOR_I2S_ICH2:</b><br>
         * Get Format selection for ICH2 sources<br>
         * Param  : structure (aai_i2sin_sources) (see AAI_IOW_I2S_ICH2).
         * <hr>
         */
        case AAI_IOR_I2S_ICH2:
            {
                struct aai_i2sin_sources s;
                s.first  = aai_get_nbit(aai, AAI_I2S_FORMAT, 0x3,AAI_I2S_IHC2_IN1_SHIFT);
                s.second = aai_get_nbit(aai, AAI_I2S_FORMAT, 0x3,AAI_I2S_IHC2_IN2_SHIFT);
                s.third  = aai_get_nbit(aai, AAI_I2S_FORMAT, 0x3,AAI_I2S_IHC2_IN3_SHIFT);
                err = put_data(&s, (void __user *)arg, sizeof(struct aai_i2sin_sources));
            }
            break;

        /** <li><b>AAI_IOW_I2S_ICH3:</b><br>
         * Set Format selection for ICH3 sources<br>
         * Param  : structure (aai_i2sin_sources) (see AAI_IOW_I2S_ICH2).
         * <ul> <li>first: format of the first ICH3 source: AC_IN5
         *      <li>second: format of the second ICH3 source: AC_IN6
         *      <li>third: format of the third ICH3 source: AC_IN7
         * </ul>
         * <hr>
         */
        case AAI_IOW_I2S_ICH3:
            {
                struct aai_i2sin_sources s;
                err = get_data((void __user *)arg, &s, sizeof(struct aai_i2sin_sources));
                if (err == 0)
                {
                    aai_set_nbit(aai, AAI_I2S_FORMAT, 0x3, AAI_I2S_IHC3_IN1_SHIFT, s.first);
                    aai_set_nbit(aai, AAI_I2S_FORMAT, 0x3, AAI_I2S_IHC3_IN2_SHIFT, s.second);
                    aai_set_nbit(aai, AAI_I2S_FORMAT, 0x3, AAI_I2S_IHC3_IN3_SHIFT, s.third);
                }
            }
            break;

        /** <li><b>AAI_IOR_I2S_ICH3:</b><br>
         * Get Format selection for ICH3 sources<br>
         * Param  : structure (aai_i2sin_sources) (see AAI_IOW_I2S_ICH3).<br>
         * Return : none
         * <hr>
         */
        case AAI_IOR_I2S_ICH3:
            {
                struct aai_i2sin_sources s;
                s.first  = aai_get_nbit(aai, AAI_I2S_FORMAT, 0x3,AAI_I2S_IHC3_IN1_SHIFT);
                s.second = aai_get_nbit(aai, AAI_I2S_FORMAT, 0x3,AAI_I2S_IHC3_IN2_SHIFT);
                s.third  = aai_get_nbit(aai, AAI_I2S_FORMAT, 0x3,AAI_I2S_IHC3_IN3_SHIFT);
                err = put_data(&s, (void __user *)arg, sizeof(struct aai_i2sin_sources));
            }
            break;

        /** <li><b>AAI_IOW_I2S_FORMAT_ASYNC1:</b><br>
         * I2s synchronization of format 1<br>
         * Param  : integer value (uint)
         * <ul> <li>FORMAT_SYNC_WITH_DAC=format 1 is synchronous with the DAC
         *      <li>FORMAT_ASYNC_WITH_DAC=format 1 is asynchronous with the DAC and each
         *          input using this format, uses AC_CLK25 as serial bit clock and AC_SYNC25 as frame signal.
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_I2S_FORMAT_ASYNC1:
            err = get_data((void __user *) arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_ASYNC1, (!!val));
            break;

        /** <li><b>AAI_IOW_I2S_FORMAT_ASYNC2:</b><br>
         * I2s synchronization of format 1<br>
         * Param  : integer value (uint)
         * <ul> <li>FORMAT_SYNC_WITH_DAC=format 2 is synchronous with the DAC
         *      <li>FORMAT_ASYNC_WITH_DAC=format 2 is asynchronous with the DAC and each
         *          input using this format, uses AC_CLK36 as serial bit clock and AC_SYNC36 as frame signal.
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_I2S_FORMAT_ASYNC2:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_ASYNC2, (!!val));
            break;

        /** <li><b>AAI_IOW_I2S_FORMAT_ASYNC3:</b><br>
         * I2s synchronization of format 1<br>
         * Param  : integer value (uint)
         * <ul> <li>FORMAT_SYNC_WITH_DAC=format 3 is synchronous with the DAC
         *      <li>FORMAT_ASYNC_WITH_DAC=format 3 is asynchronous with the DAC and each
         *          input using this format, uses AC_CLK47 as serial bit clock and AC_SYNC47 as frame signal.
         * </ul>
         * Return : none
         * <hr>
         */
        case AAI_IOW_I2S_FORMAT_ASYNC3:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_I2S_FORMAT, AAI_I2S_FORMAT_ASYNC3, (!!val));
            break;

        /** <li><b>AAI_IOW_PCM0MAST:</b><br>
         * PCM0 clock generation<br>
         * Param  : integer value (uint) can take following values: PCM_MASTER or PCM_SLAVE.<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_PCM0MAST:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_PCM0_CFG, AAI_PCM0_CFG_MAST, (!!val));
            break;

        /** <li><b>AAI_IOW_PCM0_SHIFT2:</b><br>
         * Shift of PCM0's second slot<br>
         * Param  : integer value (uint) can take values within this range:[0x0F;0xEF].<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_PCM0_SHIFT2:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_set_nbit(aai, AAI_PCM0_CFG, 0xFF, AAI_PCM0_CFG_SHIFT2_SHIFT, val);
            break;

        /** <li><b>AAI_IOW_PCM0_SHIFT3:</b><br>
         * Shift of PCM0's third slot<br>
         * Param  : integer value (uint) can take values within this range:[0x1F;0xEF].<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_PCM0_SHIFT3:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_set_nbit(aai, AAI_PCM0_CFG, 0xFF, AAI_PCM0_CFG_SHIFT3_SHIFT, val);
            break;

        /** <li><b>AAI_IOW_PCM0_OKI:</b><br>
         * PCM0 mode Texas / OKI <br>
         * Param  : integer value (uint) 0: PCM0's slots are in Texas mode, 1:PCM0's slots are in OKI mode.<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_PCM0_OKI:
            err = get_data((void __user *) arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_PCM0_CFG, AAI_PCM0_CFG_OKI, (!!val));
            break;

        /** <li><b>AAI_IOW_PCM0_1MHZ:</b><br>
         * Set PCM0 clock speed<br>
         * Param  : integer value (uint) 0:bit clock is 2MHz, 1:bit clock is 1MHz(available only for 8kHz sample rate).<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_PCM0_1MHZ:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_PCM0_CFG, AAI_PCM0_CFG_1MHZ, (!!val));
            break;

        /** <li><b>AAI_IOW_PCM1MAST:</b><br>
         * PCM1 clock generation<br>
         * Param  : integer value (uint) can take following values: PCM_MASTER or PCM_SLAVE.<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_PCM1MAST:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_PCM1_CFG, AAI_PCM1_CFG_MAST, (!!val));
            break;

        /** <li><b>AAI_IOW_PCM1_SHIFT2:</b><br>
         * Shift of PCM1's second slot<br>
         * Param  : integer value (uint) can take values within this range:[0x0F;0xEF].<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_PCM1_SHIFT2:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_set_nbit(aai, AAI_PCM1_CFG, 0xFF, AAI_PCM1_CFG_SHIFT2_SHIFT, val);
            break;

        /** <li><b>AAI_IOW_PCM1_OKI:</b><br>
         * Set PCM1 mode Texas / OKI <br>
         * Param  : integer value (uint) 0: PCM1's slots are in Texas mode, 1:PCM1's slots are in OKI mode.<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_PCM1_OKI:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_PCM1_CFG, AAI_PCM1_CFG_OKI, (!!val));
            break;

        /** <li><b>AAI_IOW_PCM1_1MHZ:</b><br>
         * Set PCM1 clock speed<br>
         * Param  : integer value (uint) 0:bit clock is 2MHz, 1:bit clock is 1MHz(available only for 8kHz sample rate).<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_PCM1_1MHZ:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                aai_setbit(aai, AAI_PCM1_CFG, AAI_PCM1_CFG_1MHZ, (!!val));
            break;

        /* add others PCM configurations XXX */

        /** <li><b>AAI_IOW_DMACNT_MUSIC:</b><br>
         * Set Number of DMA accesses for music channels.
         * Each DMA access moves 16 words of 20 bits or 16 words of 32 bits<br>
         * Param  : integer value (uint) [1,128].<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_DMACNT_MUSIC:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
            {
                if (aai->spec_ops->set_dma_cnt)
                    err = aai->spec_ops->set_dma_cnt(aai, AAI_SPK_OUT0, val);
            }
            break;

        /** <li><b>AAI_IOW_DMACNT_AUX:</b><br>
         * Set Number of DMA accesses for auxiliaries channels.
         * Each DMA access moves 16 words of 32 bits<br>
         * Param  : integer value (uint) [1,128].<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_DMACNT_AUX:
            err = get_data((void __user *) arg, &val, sizeof(unsigned int));
            if (err == 0)
            {
                if (aai->spec_ops->set_dma_cnt)
                    err = aai->spec_ops->set_dma_cnt(aai, AAI_SPK_AUX, val);
            }
            break;

        /** <li><b>AAI_IOW_DMACNT_VOICE:</b><br>
         * Set Number of DMA accesses for voice channels.
         * Each DMA access moves 2,4 or 8 words of 32 bits<br>
         * Param  : integer value (uint).<br>
         * Return : value within [1,8]
         * <hr>
         */
        case AAI_IOW_DMACNT_VOICE:
            err = get_data((void __user *) arg, &val, sizeof(unsigned int));
            if(err == 0)
            {
                if (aai->spec_ops->set_dma_cnt)
                    err = aai->spec_ops->set_dma_cnt(aai, AAI_SPK_8KHZ, val);
            }
            break;

        /** <li><b>AAI_IOR_DMACNT_MUSIC:</b><br>
         * Get Number of DMA accesses for music channels.
         * Each DMA access moves 16 words of 20 or 32 bits<br>
         * Param  : integer value (uint).<br>
         * Return : value within [1,128]
         * <hr>
         */
        case AAI_IOR_DMACNT_MUSIC:
            if (aai->spec_ops->get_dma_cnt)
                val = aai->spec_ops->get_dma_cnt(aai, AAI_SPK_OUT0);
            err = put_data(&val, (void __user *)arg, sizeof(unsigned int));
            break;

        /** <li><b>AAI_IOR_DMACNT_AUX:</b><br>
         * Get Number of DMA accesses for auxiliaries channels.
         * Each DMA access moves 8 words of 32 bits<br>
         * Param  : integer value (uint).<br>
         * Return : value within [1,8]
         * <hr>
         */
        case AAI_IOR_DMACNT_AUX:
            if (aai->spec_ops->get_dma_cnt)
                val = aai->spec_ops->get_dma_cnt(aai, AAI_SPK_AUX);
            err = put_data(&val, (void __user *)arg, sizeof(unsigned int));
            break;

        /** <li><b>AAI_IOR_DMACNT_VOICE:</b><br>
         * Get Number of DMA accesses for music channels.
         * Each DMA access moves 2,4 or 8 words of 32 bits<br>
         * Param  : integer value (uint).<br>
         * Return : value within [1,8]
         * <hr>
         */
        case AAI_IOR_DMACNT_VOICE:
            if (aai->spec_ops->get_dma_cnt)
                val = aai->spec_ops->get_dma_cnt(aai, AAI_SPK_8KHZ);
            err = put_data(&val, (void __user *) arg, sizeof(unsigned int));
            break;

        /** <li><b>AAI_IOW_LOUDREF_OUT0:</b><br>
         * Set loudness reference for OUT0.<br>
         * Param  : integer value (uint).Unit is 0,25 dB, range value is [-128;+40]
         *          it corresponds to [-32dB;+10dB](see Parrot6 user's manual for more details<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_LOUDREF_OUT0:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                err = aai_hw_setloudref(aai, 0, val );
            break;

        /** <li><b>AAI_IOW_LOUDREF_OUT1:</b><br>
         * Set loudness reference for OUT1.<br>
         * Param  : integer value (uint).Unit is 0,25 dB, range value is [-128;+40]
         *          it corresponds to [-32dB;+10dB](see Parrot6 user's manual for more details<br>
         * Return : none
         * <hr>
         */
        case AAI_IOW_LOUDREF_OUT1:
            err = get_data((void __user *) arg, &val, sizeof(unsigned int));
            if (err == 0)
                err = aai_hw_setloudref(aai, 1, val );
            break;

        /** <li><b>AAI_IOW_SYNC:</b><br>
         * Mark a device to be synchronized.<br>
         * Param  : name of the device
         * Return : none
         * <hr>
         */
        case AAI_IOW_SYNC:
            err = get_data((void __user *)arg, &val, sizeof(unsigned int));
            if (err == 0)
                err = aai_hw_setsync(aai, (const char *)val);
            break;

        /** <li><b>AAI_IOW_STARTGROUP:</b><br>
         * Start all previously synchronized devices altogether.<br>
         * Param  : none
         * Return : none
         * <hr>
         */
        case AAI_IOW_STARTGROUP:
            err = aai_hw_startgroup(aai);
            break;
        /** </ul>
         */

        default:
            aai_err(aai->dev,"unknown key 0x%08x\n", cmd);
            err = -EINVAL;
            break;
    }

    if(err!=0){
        aai_err(aai->dev,"aai_ioctl failed err 0x%08x key 0x%08x\n", err, cmd);
    }

    return err;
}

static int aai_hwdep_dummy(struct snd_hwdep * hw, struct file *file)
{
    return 0;
}

int aai_ioctl_hwdep_new_p6(struct card_data_t  *aai, char * id, int device)
{
    struct snd_hwdep * rhwdep;
    int err;

    if ((err = snd_hwdep_new(aai->card, id, 0, &rhwdep)) < 0)
        return err;

    rhwdep->private_data = aai;
    rhwdep->ops.open     = aai_hwdep_dummy;
    rhwdep->ops.ioctl    = aai_hwdep_ioctl;
    rhwdep->ops.release  = aai_hwdep_dummy;

    return 0;
}

