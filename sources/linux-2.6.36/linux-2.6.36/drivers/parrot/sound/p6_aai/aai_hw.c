/**
 * @file   aai_hw.c
 * @brief  AAI hardware layer
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

#include <linux/clk.h>

#include "aai.h"
#include "aai_hw.h"
#include <mach/aai.h>
#include <mach/parrot.h>

/******************************************************************************
 *
 * DEVICE MANAGEMENT functions
 *
 *****************************************************************************/

/**
 * @brief Applies device rules
 *
 * @param aai         AAI card instance
 * @param rules_list  list of rules to apply
 *
 * @return error code
 */
int aai_hw_rules(struct card_data_t *aai, aai_regrules_t * rules_list)
{
    int i = 0;
    uint32_t reg;

    while ((i <= NB_RULES) && (rules_list[i].addr!=0))
    {
        /*aai_dbg(aai->dev,"rule %d - 0x%04x 0x%08x 0x%08x\n",
                  i,rules_list[i].addr,rules_list[i].mask,rules_list[i].value);*/
        reg = aai_readreg(aai, rules_list[i].addr);
        reg = reg & ~(rules_list[i].mask);
        reg |= rules_list[i].value;
        aai_writereg(aai,reg,rules_list[i].addr);
        i++;
    }
    return 0;
}

/**
 * @brief Print AAI Registers
 *
 * @param aai  AAI card instance
 */
void aai_hw_status(struct card_data_t *aai)
{
#if CONFIG_AAI_DBG_LEVEL > 2
    int i = AAI_CFG;

    while (aai->vaai[IREG(i)].addr!=0xFFFFFFFF)
    {
        if (aai->vaai[IREG(i)].name[0]== 'A')
        {
            aai_print(aai->dev,"0x%08x: %s\n",
                      aai_readreg(aai,aai->vaai[IREG(i)].addr),
                      aai->vaai[IREG(i)].name);
        }
        i++;
    }
#else
    int offset = AAI_CFG;
    while (offset <= AAI_DMA_INT_ACK)
    {
        aai_print(aai->dev, "0x%08x: 0x%04x\n", aai_readreg(aai, offset), offset);
        offset += 4;
    }
#endif
}

/**
 * @brief Initialize AAI driver and hardware.
 *        This function set defaults values for AAI_CFG register.<br>
 * <ul>
 *    <li>@ref AAI_CFG_COMPACT    : Set mutimedia sampling frequency (44.1 kHz)
 *    <li>@ref AAI_CFG_AUX_FIFO_32: Set auxiliary fifo depth to 64 bytes
 *    <li>@ref AAI_CFG_RUN_MULT   : enable multimedia interfaces
 *    <li>@ref AAI_CFG_USE_ICH2   : enable music input2 interfaces
 *    <li>@ref AAI_CFG_USE_ICH3   : enable music input3 interfaces
 *    <li>@ref AAI_CFG_FEEDBACK   : enable feedback interfaces
 * </ul>
 *
 * @param aai  AAI card instance
 *
 * @return error code
 */
int aai_hw_init(struct card_data_t *aai)
{
    int32_t err;
    int32_t ipcm;
    struct clk *clock;

    spin_lock(&aai->hwlock);

    clock = clk_get(NULL, "aai");
    clk_enable(clock);

    /*
     * disable all channels
     */
    aai_writereg(aai, 0x0, AAI_ITEN);

    /*
     * call chip specific init
     */
    err = aai->spec_ops->hw_init(aai);

    /*
     * Retrieve volume for each channel
     */
    for (ipcm = 0; ipcm < aai->pcms_cnt; ipcm++)
    {
        aai->spec_ops->get_volume(&(aai->chans[ipcm]));
    }

    spin_unlock(&aai->hwlock);
    return err;
}

/**
 * @brief Disable AAI hardware.
 *        We assume that this is common to every targets
 *
 * @param aai  AAI card instance
 *
 * @return error code
 */
int aai_hw_remove(struct card_data_t *aai)
{
    struct clk * clock;

    spin_lock(&aai->hwlock);

    /*
     * reset AAI control registers
     */
    aai_writereg(aai, 0x0, AAI_ITEN);
    aai_writereg(aai, 0x0, AAI_CFG);
    aai_writereg(aai, 0x0, AAI_PCM0_CFG);
    aai_writereg(aai, 0x0, AAI_PCM1_CFG);
    aai_writereg(aai, 0x0, AAI_DMACTL);
    aai_writereg(aai, 0x0, AAI_I2S_FORMAT);

    clock = clk_get(NULL, "aai");
    clk_disable(clock);

    spin_unlock(&aai->hwlock);
    return 0;
}

/**
 * @brief Start channel.
 *        This function enables device interruptions
 *
 * @param chan pointer to an audio device descriptor
 *
 * @return error code
 */
int aai_hw_start_channel(aai_device_t *chan)
{
    uint32_t reg;
    struct card_data_t *aai = chan->pdrvdata;
    struct parrot_aai_platform_data *pdata = aai->dev->platform_data;

    if (chan->ipcm < aai->pcms_cnt)
    {
        spin_lock(&aai->hwlock);
        chan->nbfiq = 0; /*TODO: only for tests*/
        chan->nbirq = 0; /*TODO: only for tests*/
        chan->nbper = 0; /*TODO: only for tests*/
        reg = aai_readreg(aai,AAI_ITEN );

        if (reg&chan->enflag)
        {
            aai_err(aai->dev,"%s(%d) - IT already started iten:0x%08x\n",chan->name, chan->ipcm, reg);
        }
        else
        {
            if (!chan->sync)
            {
              /*
               * Enable i2s clock
               */
                if(pdata && (chan->enflag & AAI_ITEN_I2S))
                {
                    if(pdata->i2s_bit_clock_control)
                        aai->spec_ops->enable_i2s_bit_clock();
                    if(pdata->i2s_master_clock_control)
                        aai->spec_ops->enable_i2s_master_clock();
		}

                aai_writereg(aai, (reg | chan->enflag), AAI_ITEN);
	    }
	    chan->started = 1;
	}
	spin_unlock(&aai->hwlock);
	return 0;
    }

    return -EINVAL;
}

/**
 * @brief Stop channel.
 *        This function disables device interruptions
 *
 * @param chan pointer to an audio device descriptor
 *
 * @return error code
 */
int aai_hw_stop_channel(aai_device_t *chan)
{
    uint32_t reg;
    struct card_data_t *aai = chan->pdrvdata;
    struct parrot_aai_platform_data *pdata = aai->dev->platform_data;

    if (chan->ipcm < aai->pcms_cnt)
    {
        spin_lock(&aai->hwlock);
        reg = aai_readreg(aai, AAI_ITEN);
        if ((reg&chan->enflag) == 0)
        {
            aai_err(aai->dev,"%s(%d) - IT already stopped iten:0x%08x, flag:0x%08x\n",
                    chan->name, chan->ipcm, reg, chan->enflag);
        }
        else
        {
            aai_writereg(aai, (reg&(~chan->enflag)), AAI_ITEN);
          /*
           * Disable i2s clocks
           */
            if(pdata && !((reg&(~chan->enflag)) & AAI_ITEN_I2S))
            {
                if(pdata->i2s_bit_clock_control)
                    aai->spec_ops->disable_i2s_bit_clock();
                if(pdata->i2s_master_clock_control)
                    aai->spec_ops->disable_i2s_master_clock();
            }
        }
        chan->started = 0;
        spin_unlock(&aai->hwlock);
        return 0;
    }

    return -EINVAL;
}

/******************************************************************************
 *
 * RATE functions
 *
 *****************************************************************************/

/**
 * @brief Set Sample rate convertor direction.
 *
 * @param chan pointer to an audio device descriptor
 * @param rate numerical value
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
int aai_hw_set_srconvdir(struct card_data_t *aai, int val)
{
    return aai->spec_ops->set_srconvdir(aai, val);
}

/**
 * Get Sample rate convertor ratio.
 *
 * @param chan pointer to an audio device descriptor
 * @param rate numerical value
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
int aai_hw_get_srconvrate(struct card_data_t *aai)
{
    return aai->spec_ops->get_srconvrate(aai);
}

/**
 * Set Sample rate convertor ratio.
 *
 * @param chan pointer to an audio device descriptor
 * @param rate numerical value
 *
 * @retval 0 if succeed
 * @retval -EINVAL if rate can't be set
 */
int aai_hw_set_srconvrate(struct card_data_t *aai, int rate)
{
    return aai->spec_ops->set_srconvrate(aai, rate);
}

/******************************************************************************
 *
 * VOLUME functions
 *
 *****************************************************************************/
int aai_hw_setloudref(struct card_data_t *aai,
                     uint32_t            outn,
                     int                 ref)
{
    uint32_t reg = aai_readreg(aai, AAI_LOUD_CTL);

    if (ref)
    {
        aai_writereg(aai, reg | (0x1 << outn), AAI_LOUD_CTL);
    }
    else
    {
        aai_writereg(aai, reg & ~(0x1 << outn), AAI_LOUD_CTL);
    }

    aai_writereg(aai, ref, AAI_LOUDREF_OUT0+(outn*sizeof(uint32_t)));
    return 0;
}

/**
 * @brief Start synchronized AAI devices.
 *        This ckeck all devices marked as synchronized
 *        and enable the interruptions for all of them
 *        at once.
 *
 * @param aai  AAI card structure
 *
 * @return Error code
 */
int aai_hw_startgroup(struct card_data_t *aai)
{
    unsigned int oldreg, reg = 0;
    int ipcms;
    struct parrot_aai_platform_data *pdata = aai->dev->platform_data;
    int ret = 0;

    spin_lock(&aai->hwlock);

    oldreg = aai_readreg(aai, AAI_ITEN);

    for (ipcms = 0; ipcms < aai->pcms_cnt; ipcms++)
    {
        if (aai->chans[ipcms].sync)
        {
            if (!aai->chans[ipcms].started) {
                aai_err(aai->dev, "startgroup channel %s is not started\n",
                        aai->chans[ipcms].name);
                ret = -EBUSY;

            }
            reg |= aai->chans[ipcms].enflag;
            aai->chans[ipcms].sync = 0;
        }
    }
    /*Ensure that all the channels have the same  sync state enven if it fails*/
    if (ret == -EBUSY)
	goto end;

    /*
     * Make i2s clock pin selection
     */
    if(pdata && (reg & AAI_ITEN_I2S))
    {
        if(pdata->i2s_bit_clock_control)
            aai->spec_ops->enable_i2s_bit_clock();
        if(pdata->i2s_master_clock_control)
            aai->spec_ops->enable_i2s_master_clock();
    }

    /*
     * enable all requested channels
     */
    aai_writereg(aai, reg | oldreg, AAI_ITEN);
    aai_dbg(aai->dev,"%s: 0x%08x\n", __FUNCTION__, reg | oldreg);
end:
    spin_unlock(&aai->hwlock);

    return ret;
}

/**
 * @brief Mark a channel as being synchronized.
 *        All synchronized devices will be started altogether
 *        when calling aai_hw_startgroup().
 *
 * @param aai   AAI card structure
 * @param name  name of the device
 *
 * @return Error code
 **/
int aai_hw_setsync(struct card_data_t *aai, const char *name)
{
    int ipcms;

    spin_lock(&aai->hwlock);

    for (ipcms = 0; ipcms < aai->pcms_cnt; ipcms++)
    {
        if (!strcmp(name, aai->chans[ipcms].name))
        {
            aai_dbg(aai->dev,"%s: sync %s\n", __FUNCTION__, name);
            aai->chans[ipcms].sync = 1;
            break;
        }
    }

    spin_unlock(&aai->hwlock);

    return 0;
}

