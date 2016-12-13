/*
 * linux/drivers/io/aai_irq.c
 *	Copyright (c) Parrot SA
 *
 *	Written by Gregoire ETIENNE <gregoire.etienne@parrot.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    Parrot Advanced Audio Interface Driver
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include "aai.h"
#include "aai_hw.h"



//! Copy 32 bytes with 16-bit swap (endianness inversion)
#define aai_copy_32_swap( _src_, _dst_ )                        \
    asm volatile (                                              \
                  "ldmia %0,{r0-r7}\n\t"                        \
                  "mov r0,r0,ror #16\n\t"                       \
                  "mov r1,r1,ror #16\n\t"                       \
                  "mov r2,r2,ror #16\n\t"                       \
                  "mov r3,r3,ror #16\n\t"                       \
                  "mov r4,r4,ror #16\n\t"                       \
                  "mov r5,r5,ror #16\n\t"                       \
                  "mov r6,r6,ror #16\n\t"                       \
                  "mov r7,r7,ror #16\n\t"                       \
                  "stmia %1,{r0-r7}\n" :                        \
                  : "r" ((_src_)), "r" ((_dst_))                \
                  : "r0","r1","r2","r3","r4","r5","r6","r7" )

//! Copy 32 bytes using a burst access
#define aai_copy_32( _src_, _dst_ )                             \
    asm volatile (                                              \
                  "ldmia %0,{r0-r7}\n\t"                        \
                  "stmia %1,{r0-r7}\n" :                        \
                  : "r" ((_src_)), "r" ((_dst_))                \
                  : "r0","r1","r2","r3","r4","r5","r6","r7" )

/** Unpack 8x16 bits samples into 8x32 bits samples and write to FIFO as follow
 * <br><tt>
 * R0 &nbsp;&nbsp; M1|M2 &nbsp;&nbsp; --> &nbsp;&nbsp; M1|M2 &nbsp;&nbsp; --> &nbsp;&nbsp; FIFO <br>
 * R1 &nbsp;&nbsp; --|-- &nbsp;&nbsp; |-> &nbsp;&nbsp; --|M1 &nbsp;&nbsp; --> &nbsp;&nbsp; FIFO <br>
 * </tt>
 * It does the same for R2/R3, R4/R5, R6/R7.<br>
 * This macro assume that the (output) fifo is mono
 */
#define aai_write_16_to_fifo( _reg_, _addr_ )                   \
    asm volatile (                                              \
                  "ldmia %0,{r0,r2,r4,r6}\n\t"                  \
                  "mov r1,r0,lsr #16\n\t"                       \
                  "mov r3,r2,lsr #16\n\t"                       \
                  "mov r5,r4,lsr #16\n\t"                       \
                  "mov r7,r6,lsr #16\n\t"                       \
                  "stmia %1,{r0-r7}\n" :                        \
                  : "r" ((_addr_)), "r" ((_reg_))               \
                  : "r0","r1","r2","r3","r4","r5","r6","r7" )

/** Read 8 stereo samples (8x32bits) from FIFO and pack them into 8 mono samples (8x16bits)
 * <br><tt>
 * R0 &nbsp;&nbsp; --|R1 &nbsp;&nbsp; --> &nbsp;&nbsp; R2|R1 &nbsp;&nbsp; --> &nbsp;&nbsp; buffer <br>
 * R1 &nbsp;&nbsp; --|R2 &nbsp;&nbsp; --> &nbsp;&nbsp; --|R2<br>
 * </tt>
 * It does the same for R2/R3, R4/R5, R6/R7.<br>
 * This macro assume that the (input) fifo is mono
 */
#define aai_read_fifo_to_16( _reg_, _addr_ )                    \
    asm volatile (                                              \
                  "ldmia %1,{r0-r7}\n\t"                        \
                  "orr r0,r0,r1,lsl #16\n\t"                    \
                  "orr r2,r2,r3,lsl #16\n\t"                    \
                  "orr r4,r4,r5,lsl #16\n\t"                    \
                  "orr r6,r6,r7,lsl #16\n\t"                    \
                  "stmia %0,{r0,r2,r4,r6}\n\t" :                \
                  : "r" ((_addr_)), "r" ((_reg_))               \
                  : "r0","r1","r2","r3","r4","r5","r6","r7" )

const aai_fifo_mode_t aai_modes_spk[4][2] = {
    /* stream mode                    surround=0     surround=1 */
    [AAI_STREAM_MODE_8KHZ_16W    ] = {AAI_MONO_8,    AAI_MONO_8 },
    [AAI_STREAM_MODE_16KHZ_32W   ] = {AAI_MONO_16,   AAI_MONO_16},
    [AAI_STREAM_MODE_44_48KHZ_16W] = {AAI_STEREO_8,  AAI_QUAD_8 },
    [AAI_STREAM_MODE_44_48KHZ_32W] = {AAI_STEREO_16, AAI_QUAD_16},
};

const aai_fifo_mode_t aai_modes_mic[4] = {
    /* stream mode */
    [AAI_STREAM_MODE_8KHZ_16W    ] = AAI_STEREO_8,
    [AAI_STREAM_MODE_16KHZ_32W   ] = AAI_STEREO_16,
    [AAI_STREAM_MODE_44_48KHZ_16W] = AAI_STEREO_8,
    [AAI_STREAM_MODE_44_48KHZ_32W] = AAI_STEREO_16,
};

static void aai_process_tx(aai_device_t *chan, aai_fifo_mode_t mode, void * iobase);
static void aai_process_rx(aai_device_t *chan, aai_fifo_mode_t mode, void * iobase);

/******************************************************************************
 *
 * Public functions
 *
 *****************************************************************************/
//! Additional flag masks
#define AAI_INT_BT_P5P                          \
    (AAI_INT_BT_RX  | AAI_INT_BT_TX |           \
     AAI_INT_BT_RX2 | AAI_INT_BT_TX2|           \
     AAI_INT_BT_RX3 | AAI_INT_BT_TX3)

//! Additional flag masks
#define AAI_INT_EXT_P5P                           \
    (AAI_INT_EXT_RX  | AAI_INT_EXT_TX |           \
     AAI_INT_EXT_RX2 | AAI_INT_EXT_TX2)


static void aai_period_elapsed(struct card_data_t *aai, aai_device_t *chan)
{
   if(chan->bytes_count >= chan->period_bytes){
      if(chan->direction == AAI_RX)
         snd_pcm_period_elapsed( chan2captsubstream(aai,chan->ipcm) );
      else if(chan->direction == AAI_TX)
         snd_pcm_period_elapsed( chan2pbacksubstream(aai,chan->ipcm) );

      chan->bytes_count = 0;
   }
}

int nbirq;

irqreturn_t aai_irq(int irq, void *dev)
{
   struct card_data_t * aai = dev;
   aai_device_t * chan;
   aai_fifo_mode_t mode;
   uint32_t cfg   = readl(aai->iobase+_AAI_CFG );
   uint32_t iten  = readl(aai->iobase+_AAI_FE );
   uint32_t itsrc = readl(aai->iobase+_AAI_INT );

   nbirq++;

   if(itsrc & (aai_channels[AAI_CHANNEL_MIC12].intflag) ){
      chan = &(aai_channels[AAI_CHANNEL_MIC12]);
      mode = aai_modes_mic[AAI_FE_MIC12_FREQ(iten)];
      aai_process_rx(chan, mode, aai->iobase);
      aai_period_elapsed(aai,chan);
   }

   if(itsrc & (aai_channels[AAI_CHANNEL_MIC34].intflag) ){
      chan = &(aai_channels[AAI_CHANNEL_MIC34]);
      mode = aai_modes_mic[AAI_FE_MIC34_FREQ(iten)];
      aai_process_rx(chan, mode, aai->iobase);
      aai_period_elapsed(aai,chan);
   }

   if(itsrc & (aai_channels[AAI_CHANNEL_SPK].intflag) ){
      chan = &(aai_channels[AAI_CHANNEL_SPK]);
      mode = aai_modes_spk[AAI_FE_SPK_FREQ(iten)][(iten & AAI_FE_SURROUND)? 1 : 0];
      aai_process_tx(chan, mode, aai->iobase);
      aai_period_elapsed(aai,chan);
   }

    /* bluetooth channels */
    if (itsrc & AAI_INT_BT_P5P) {
      mode = (cfg & AAI_CFG_BLUE_16K)? AAI_MONO_16 : AAI_MONO_8;

      if(itsrc & (aai_channels[AAI_CHANNEL_BT_IN].intflag) ){
         chan = &(aai_channels[AAI_CHANNEL_BT_IN]);
         aai_process_rx(chan, mode, aai->iobase);
         aai_period_elapsed(aai,chan);
      }

      if(itsrc & (aai_channels[AAI_CHANNEL_BT_OUT].intflag) ){
         chan = &(aai_channels[AAI_CHANNEL_BT_OUT]);
         aai_process_tx(chan, mode, aai->iobase);
         aai_period_elapsed(aai,chan);
      }
   } /* end of bluetooth processing */

    /* external channels */
    if (itsrc & AAI_INT_EXT_P5P) {
      mode = (cfg & AAI_CFG_EXT_16K)? AAI_MONO_16 : AAI_MONO_8;

      if(itsrc & (aai_channels[AAI_CHANNEL_EXT_IN].intflag) ){
         chan = &(aai_channels[AAI_CHANNEL_EXT_IN]);
         aai_process_rx(chan, mode, aai->iobase);
         aai_period_elapsed(aai,chan);
      }

      if(itsrc & (aai_channels[AAI_CHANNEL_EXT_OUT].intflag) ){
         chan = &(aai_channels[AAI_CHANNEL_EXT_OUT]);
         aai_process_tx(chan, mode, aai->iobase);
         aai_period_elapsed(aai,chan);
      }
   } /* end of external processing */

	return IRQ_HANDLED;
}

/******************************************************************************
 *
 * Private functions
 *
 *****************************************************************************/
#define STEREO_SAMPLE_SIZE   4  /* number of bytes of a stereo sample */
#define MONO_SAMPLE_SIZE     2  /* number of bytes of a mono sample */

/** Write 8 stereo samples(16bits) to hw fifo and update buffer pointer
 *
 * @param src   Pointer to the source buffer
 * @param fifo	Pointer to fifo
 *
 * @return none
 */
static inline void write_8st_samples_to_fifo(char **src, void *fifo)
{
   aai_copy_32( *src, fifo );
   *src += 8 * STEREO_SAMPLE_SIZE;/* + 32bytes */
}

/** Write 8 mono samples(16bits) to hw fifo and update buffer pointer
 *
 * @param src   Pointer to the source buffer
 * @param fifo	Pointer to fifo
 *
 * @return none
 */
static inline void write_8mo_samples_to_fifo(char **src, void *fifo)
{
   aai_write_16_to_fifo( fifo, *src);
   *src += 8 * MONO_SAMPLE_SIZE; /* + 16bytes */
}

/** Read 8 stereo samples(16bits) to hw fifo and update buffer pointer
 *
 * @param src   Pointer to the source buffer
 * @param fifo	Pointer to fifo
 *
 * @return none
 */
static inline void read_8st_samples_from_fifo(void *fifo, char **dst)
{
   aai_copy_32( fifo, *dst );
   *dst += 8 * STEREO_SAMPLE_SIZE;/* + 32bytes */
}


/** Read 8 mono samples(16bits) from hw fifo and update buffer pointer
 * Pack 8x32bits into 8x16bits
 * @param src   Pointer to the source buffer
 * @param fifo	Pointer to fifo
 *
 * @return none
 */
static inline void read_8mo_samples_from_fifo(void *fifo, char **dst)
{
   aai_read_fifo_to_16( fifo, *dst );
   *dst += 8 * MONO_SAMPLE_SIZE;/* + 16bytes */
}


/** Write 8 or 16 samples to hw fifo and update buffer pointer
 *
 * @param channel       Channel
 * @param fifo_mode     Channel fifo mode
 * @param iobase        AAI base adress
 *
 * @return none
 */
static void aai_process_tx(aai_device_t *chan, aai_fifo_mode_t mode, void * iobase)
{
    char *pcurrent, *pcurrent_old;
    static const uint32_t mute_fifo[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    void * hfifo = (iobase+chan->fifo[0].hfifo);

    pcurrent = (chan->fifo[0].mute)? (char *)mute_fifo : chan->fifo[0].pcurrent;
    pcurrent_old = pcurrent;

    switch (mode) {

    case AAI_MONO_16:
        write_8mo_samples_to_fifo(&pcurrent, hfifo );
    case AAI_MONO_8:
        write_8mo_samples_to_fifo(&pcurrent, hfifo );
        break;

    case AAI_QUAD_16:
         write_8st_samples_to_fifo(&pcurrent, hfifo );
         write_8st_samples_to_fifo(&pcurrent, hfifo );
    case AAI_QUAD_8:
    case AAI_STEREO_16:
         write_8st_samples_to_fifo(&pcurrent, hfifo );
    case AAI_STEREO_8:
         write_8st_samples_to_fifo(&pcurrent, hfifo );
         break;

    default:
        break;
    }

   chan->fifo[0].pcurrent += (pcurrent - pcurrent_old);
   chan->bytes_count += (pcurrent - pcurrent_old);

    /* update sample pointer */
   if (chan->fifo[0].pcurrent >= chan->fifo[0].pend){
		   chan->fifo[0].pcurrent = chan->fifo[0].pstart;
   }
}

/** Read 8 or 16 samples from hw fifo and update buffer pointer
 *
 * @param channel       Channel
 * @param fifo_mode     Channel fifo mode
 *
 * @return none
 */
static void aai_process_rx(aai_device_t *chan, aai_fifo_mode_t mode, void * iobase)
{
   char *pcurrent, *pcurrent_old;
   aai_hwchan_t *fifo = &(chan->fifo[0]);
   void * hfifo = (iobase+chan->fifo[0].hfifo);

   pcurrent = fifo->pcurrent;
   pcurrent_old = pcurrent;

    switch (mode) {

    case AAI_MONO_16:
        read_8mo_samples_from_fifo(hfifo, &pcurrent );
    case AAI_MONO_8:
        read_8mo_samples_from_fifo(hfifo, &pcurrent );
        break;

    case AAI_STEREO_16:
         read_8st_samples_from_fifo(hfifo, &pcurrent );
    case AAI_STEREO_8:
         read_8st_samples_from_fifo(hfifo, &pcurrent );
		break;

    default:
        /* no rx quad mode support */
        break;
    }

    /* update sample pointer */
   fifo->pcurrent += (pcurrent - pcurrent_old);
   chan->bytes_count += (pcurrent - pcurrent_old);

   if (chan->fifo[0].pcurrent >= chan->fifo[0].pend){
		   chan->fifo[0].pcurrent = chan->fifo[0].pstart;
   }
}



