/**
 * @file   aai_hw.h
 * @brief  AAI hardware layer header
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

#ifndef INCONCE_AAI_HW_H
#define INCONCE_AAI_HW_H

#include <mach/regs-aai-p6.h>

#define IREG(_i)  (_i-AAI_CFG)

#define AAI_DMACNT_MAX_VOICE  8
#define AAI_DMACNT_MAX_MUSIC  128
#define AAI_DMACNT_MAX_AUX    128

#define DMASIZE_2w32bits   ( 2*sizeof(uint32_t))   /*<! size of  2 words of 32 bits */
#define DMASIZE_4w32bits   ( 4*sizeof(uint32_t))   /*<! size of  4 words of 32 bits */
#define DMASIZE_8w32bits   ( 8*sizeof(uint32_t))   /*<! size of  8 words of 32 bits */
#define DMASIZE_16w32bits  (16*sizeof(uint32_t))   /*<! size of 16 words of 32 bits */
#define DMASIZE_32w32bits  (32*sizeof(uint32_t))   /*<! size of 32 words of 32 bits */

#define PCM0_16K(_reg) (!!(_reg&AAI_PCM0_CFG_16K))
#define PCM1_16K(_reg) (!!(_reg&AAI_PCM1_CFG_16K))

#define SRCONV_PRECISION    (1000)
#define MASTER_DAC_RATE_44K (44099507) /* real value:44099,506578Hz */
#define MASTER_DAC_RATE_48K (48004150) /* real value:48004,1503906Hz */
#define MIN_RATIO_RATE      (16000)
#define MAX_RATIO_RATE      (96000)

#define AAI_ITEN_I2S	(AAI_ITEN_OUT0|AAI_ITEN_OUT1|		\
			AAI_ITEN_16KHZ|AAI_ITEN_8KHZ|		\
			AAI_ITEN_8KHZ_ICH0|AAI_ITEN_16KHZ_ICH0|	\
			AAI_ITEN_8KHZ_ICH2|AAI_ITEN_16KHZ_ICH2| \
			AAI_ITEN_AUX|AAI_ITEN_ICH1|AAI_ITEN_ICH3)

/**
 * Virtual register structure definition
 * This provides register information during debug
 */
struct virtual_reg
{
   uint32_t addr;
   uint32_t reg;
   char     name[32];
};

/**
 * Card initialization functions
 * There is one version for each target
 */
int aai_init_card_p6(struct device *dev, struct card_data_t *aai, int dev_id);
int aai_init_card_p6i(struct device *dev, struct card_data_t *aai, int dev_id);

/*
 * Exported prototypes of aai_hw.c
 * These are common for all targets
 */
int  aai_hw_init(struct card_data_t *aai);
int  aai_hw_remove(struct card_data_t *aai);
void aai_hw_status(struct card_data_t *aai);

int  aai_hw_rules(struct card_data_t *aai, aai_regrules_t * rules_list);

int  aai_hw_start_channel(aai_device_t *chan);
int  aai_hw_stop_channel(aai_device_t *chan);

int  aai_hw_set_srconvdir(struct card_data_t *aai, int val);
int  aai_hw_set_srconvrate(struct card_data_t *aai, int rate);
int  aai_hw_get_srconvrate(struct card_data_t *aai);

int  aai_hw_setloudref(struct card_data_t *aai, uint32_t outn, int ref);

int aai_hw_startgroup(struct card_data_t *aai);
int aai_hw_setsync(struct card_data_t *aai, const char *name);

/**
 * @brief Write register wrapper.
 *        Write register value and print debug info
 *
 * @param aai private driver data
 * @param val value to write
 * @param offset register offset
 *
 * @return none
 */
static inline void aai_writereg(struct card_data_t *aai, uint32_t val, uint32_t offset)
{
    if(offset <= AAI_DMA_INT_ACK){
        writel(val, aai->iobase+offset);
#if CONFIG_AAI_DBG_LEVEL > 0
        aai_dbg(aai->dev, "0x%08x: %s\n", val, aai->vaai[IREG(offset)].name );
        if((offset != AAI_PCM0_CFG)                 &&
           (offset != AAI_PCM1_CFG)                 &&
           (offset != AAI_LEFT_MUSIC_DMASA_OUT0)    &&
           (offset != AAI_LEFT_MUSIC_DMAFA_OUT0)    &&
           (offset != AAI_RIGHT_MUSIC_DMASA_OUT0)   &&
           (offset != AAI_RIGHT_MUSIC_DMAFA_OUT0)   &&
           (offset != AAI_AUX_DMAFA_OUT)            &&
           (offset != AAI_AUX_DMAFA_OUT)            &&
           (offset != AAI_8KHZ_DMAFA_OUT)           &&
           (offset != AAI_8KHZ_DMAFA_ICH0)          &&
           (offset != AAI_8KHZ_DMAFA_ICH2)          &&
           (offset != AAI_8KHZ_DMAFA_BACK)          &&
           (offset != AAI_16KHZ_DMAFA_OUT)          &&
           (offset != AAI_16KHZ_DMAFA_ICH0)         &&
           (offset != AAI_16KHZ_DMAFA_ICH2)         &&
           (offset != AAI_16KHZ_DMAFA_BACK)         &&
           (offset != AAI_MUSIC_DMAFA_ICH0)         &&
           (offset != AAI_MUSIC_DMAFA_ICH1)         &&
           (offset != AAI_MUSIC_DMAFA_ICH0)         &&
           (offset != AAI_MUSIC_DMAFA_ICH2)         &&
           (offset != AAI_MUSIC_DMAFA_ICH3)         &&
           (offset != AAI_MUSIC_DMAFA_BACK)         &&
           (offset != AAI_SO1_DMAFA_PCM0)           &&
           (offset != AAI_SO1_DMAFA_PCM1)           &&
           (offset != AAI_SO2_DMAFA_PCM0)           &&
           (offset != AAI_SO2_DMAFA_PCM1)           &&
           (offset != AAI_SO3_DMAFA_PCM0)           &&
           (offset != AAI_SI1_DMAFA_PCM0)           &&
           (offset != AAI_SI1_DMAFA_PCM1)           &&
           (offset != AAI_SI2_DMAFA_PCM0)           &&
           (offset != AAI_SI2_DMAFA_PCM1)           &&
           (offset != AAI_SI3_DMAFA_PCM0))
        {
            uint32_t regcheck = readl(aai->iobase+offset);
            if (val != regcheck){
                aai_warn(aai->dev,"write 0x%08x failed to %s(%x)=0x%08x\n",val,
#if CONFIG_AAI_DBG_LEVEL > 2
                        aai->vaai[IREG(offset)].name,
#else
                        "",
#endif
                        offset, regcheck);
            }
        }
#endif
    }else{
        aai_err(aai->dev,"0x%x: unknown offset\n", offset);
    }
}

/**
 * @brief Read register wrapper.
 *        Read register value and test offset value
 *
 * @param aai private driver data
 * @param offset register offset
 *
 * @return register value
 */
static inline unsigned int aai_readreg(struct card_data_t *aai, uint32_t offset)
{
    if(offset <= AAI_DMA_INT_ACK){
        return readl(aai->iobase+offset);
    }else{
        aai_err(aai->dev,"0x%x: unknown offset\n", offset);
    }
    return 0xFFFFFFFF; //error;
}

/**
 * @brief Set several consecutive bits.
 *
 * @param aai private driver data
 * @param offset register offset
 * @param mask bits mask
 * @param shift first bit shift
 * @param val value
 *
 * @return none
 */
static inline void aai_set_nbit(struct card_data_t *aai,
                                unsigned int       offset,
                                unsigned int       mask,
                                unsigned int       shift,
                                unsigned int       val)
{
    unsigned int reg;

    spin_lock(&aai->hwlock);

    reg = aai_readreg(aai, offset);
    reg &= ~(mask << shift);  /* set all concerned bits to 0 */
    reg |= (val << shift);    /* set needed bits to 1        */
    aai_writereg(aai, reg, offset);

    spin_unlock(&aai->hwlock);
}

static inline int aai_get_nbit(struct card_data_t *aai,
                               unsigned int       offset,
                               unsigned int       mask,
                               unsigned int       shift)
{
    unsigned int reg = aai_readreg(aai, offset);
    reg = reg >> shift;
    return reg & mask; /* set needed bits to 1 */
}

/**
 * @brief Set bit value.
 *
 * @param aai private driver data
 * @param offset register offset
 * @param mask bits mask
 * @param val value
 *
 * @return none
 */
static inline void aai_setbit(struct card_data_t *aai,
                              unsigned int       offset,
                              unsigned int       mask,
                              unsigned int       val)
{
    unsigned int reg;
    spin_lock(&aai->hwlock);

    reg = aai_readreg(aai, offset );
    /*
     * if the bit has to be modified
     */
    if (!!(reg&mask) != (!!val))
    {
        reg &= ~(mask);           /* reset concerned bit to 0 */
        if (val)
            reg |= mask;          /* set it if needed */
        aai_writereg(aai,reg, offset);
    }
    spin_unlock(&aai->hwlock);
}

/**
 * @brief Get bit value
 *
 * @param aai  AAI card instance
 * @param offset register offset
 * @param mask bits mask
 *
 * @return requested bit value
 */
static inline unsigned int aai_getbit(struct card_data_t *aai,
                                      unsigned int       offset,
                                      unsigned int       mask)
{
    unsigned int reg = aai_readreg(aai, offset);
    return !!(reg&mask);
}

/*
 * mark a device as enabled in deven field
 */
#define DEV_START(_deven_,_ipcm_)   ((_deven_) | (1 << _ipcm_))

/*
 * mark a device as disabled in deven field
 */
#define DEV_STOP(_deven_,_ipcm_)    ((_deven_) & (~(1 << _ipcm_)))
#define DEV_CONFLICT(_reg_,_mask_)  ((_reg_&_mask_) != 0)

#endif

