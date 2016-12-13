/**
 * @file   aai_irq_dma.c
 * @brief  AAI DMA interrupt routine
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
#include "aai_irq_dma.h"

/*#define USE_LTT*/
#ifdef USE_LTT
# include <trace/parrot.h>
# define MY_FUNC_ID     1

DEFINE_TRACE(trace_parrot_evt_start);
DEFINE_TRACE(trace_parrot_evt_stop);
#endif

/**
 * Update DMA buffer pointers
 *
 * @param channel       Channel
 * @param iobase        AAI base adress
 * @param ififo         fifo id
 *
 * @return none
 */
void aai_dmaxfer(aai_device_t *chan,
                 void         *iobase,
                 uint32_t     ififo)
{
    /*
     * IRQ is acknowledged when following address is written
     */
    aai_hwchan_t *fifo = &(chan->fifo[ififo]);
    writel(fifo->pfollow, iobase + fifo->dmafa);
}

/**
 * FIFO management functions
 */
/**
 * Copy 32 bytes using a burst access
 */
#define aai_copy_32( _src_, _dst_ )                             \
    asm volatile (                                              \
                  "ldmia %0,{r0-r7}\n\t"                        \
                  "stmia %1,{r0-r7}\n" :                        \
                  : "r" ((_src_)), "r" ((_dst_))                \
                  : "r0","r1","r2","r3","r4","r5","r6","r7" )

/**
 * Copy 16 bytes using a burst access
 */
#define aai_copy_16(_src_, _dst_)                   \
    asm volatile (                                  \
                  "ldmia %0,{r0-r3}\n\t"            \
                  "stmia %1,{r0-r3}\n" :            \
                  : "r" ((_src_)), "r" ((_dst_))    \
                  : "r0","r1","r2","r3")

#define aai_copy_8(_src_, _dst_)                    \
    asm volatile (                                  \
                  "ldmia %0,{r0-r1}\n\t"            \
                  "stmia %1,{r0-r1}\n" :            \
                  : "r" ((_src_)), "r" ((_dst_))    \
                  : "r0","r1")

#define aai_copy_4(_src_, _dst_)                    \
    asm volatile (                                  \
                  "ldmia %0,{r0}\n\t"               \
                  "stmia %1,{r0}\n" :               \
                  : "r" ((_src_)), "r" ((_dst_))    \
                  : "r0","r1")

#define aai_interleaved_copy_32(_src_, _dst1_, _dst2_)              \
    asm volatile (                                                  \
                  "ldmia %0, {r0-r7}\n\t"                           \
                  "stmia %1, {r0,r2,r4,r6}\n\t"                     \
                  "stmia %2, {r1,r3,r5,r7}\n" :                     \
                  : "r" ((_src_)), "r" ((_dst1_)), "r" ((_dst2_))   \
                  : "r0","r1","r2","r3","r4","r5","r6","r7")

/*
 * number of bytes of a sample
 */
#define SAMPLE_SIZE   2

static inline void read_4samples_from_fifo(void *fifo, char **dst)
{
   aai_copy_8(fifo, *dst);
   *dst += 4 * SAMPLE_SIZE; /* 4 * 16-bytes */
}

/**
 * Read 8 samples(16bits) to hw fifo and update buffer pointer
 *
 * @param src   Pointer to the source buffer
 * @param fifo  Pointer to fifo
 *
 * @return none
 */
static inline void read_8samples_from_fifo(void *fifo, char **dst)
{
   aai_copy_16(fifo, *dst);
   *dst += 8 * SAMPLE_SIZE; /* 8 * 16-bytes */
}

/**
 * Read 16 samples(16bits) to hw fifo and update buffer pointer
 *
 * @param src   Pointer to the source buffer
 * @param fifo  Pointer to fifo
 *
 * @return none
 */
static inline void read_16samples_from_fifo(void *fifo, char **dst)
{
   aai_copy_32(fifo, *dst);
   *dst += 16 * SAMPLE_SIZE; /* + 32bytes */
}

/**
 * Read 32 samples(16bits) to hw fifo and update buffer pointer
 *
 * @param src   Pointer to the source buffer
 * @param fifo  Pointer to fifo
 *
 * @return none
 */
static inline void read_32samples_from_fifo(void *fifo, char **dst)
{
   aai_copy_32(fifo, *dst);
   *dst += 16 * SAMPLE_SIZE;
   aai_copy_32(fifo, *dst);
   *dst += 16 * SAMPLE_SIZE;
}

static inline void write_4samples_from_fifo(void *fifo, char **dst)
{
   aai_copy_8(*dst, fifo);
   *dst += 4 * SAMPLE_SIZE; /* 4 * 16-bytes */
}

static inline void write_8samples_from_fifo(void *fifo, char **dst)
{
   aai_copy_16(*dst, fifo);
   *dst += 8 * SAMPLE_SIZE; /* 8 * 16-bytes */
}

static inline void write_16samples_from_fifo(void *fifo, char **dst)
{
   aai_copy_32(*dst, fifo);
   *dst += 16 * SAMPLE_SIZE; /* + 32bytes */
}

static inline void write_32samples_from_fifo(void *fifo, char **dst)
{
   aai_copy_32(*dst, fifo);
   *dst += 16 * SAMPLE_SIZE; /* + 32bytes */
   aai_copy_32(*dst, fifo);
   *dst += 16 * SAMPLE_SIZE;
}

static inline void write_interleaved_from_fifo(void *fifo1, void *fifo2, char **dst)
{
    aai_interleaved_copy_32(*dst, fifo1, fifo2); /* copy 8 32-bits samples */
    *dst += 8 * SAMPLE_SIZE * 2; /* 32 bits samples */

    aai_interleaved_copy_32(*dst, fifo1, fifo2);
    *dst += 8 * SAMPLE_SIZE * 2;

    aai_interleaved_copy_32(*dst, fifo1, fifo2);
    *dst += 8 * SAMPLE_SIZE * 2;

    aai_interleaved_copy_32(*dst, fifo1, fifo2);
    *dst += 8 * SAMPLE_SIZE * 2;
}

void aai_process_rx(aai_device_t *chan)
{
    char *cur;
    char *fifo_cur;
    struct card_data_t *aai = chan->pdrvdata;
    struct snd_pcm_substream *substream;

    aai_hwchan_t *fifo = &(chan->fifo[0]);

    substream = chan2captsubstream(aai, chan->ipcm);
    fifo_cur = fifo->pcurrent - chan->dmaxfersz;
    if (fifo_cur < fifo->pstart)
        fifo_cur += fifo->pend - fifo->pstart + 1;
    cur = (char *)substream->runtime->dma_area - fifo->pstart + fifo_cur;

    /*
     * Processing FIFO mode
     */
    switch (chan->mode)
    {
        case AAI_FIFO_2W_32b:
            read_4samples_from_fifo(aai->iobase+chan->fifo->hfifo, &cur);
            break;
        case AAI_FIFO_4W_32b:
            read_8samples_from_fifo(aai->iobase+chan->fifo->hfifo, &cur);
            break;
        case AAI_FIFO_8W_32b:
            read_16samples_from_fifo(aai->iobase+chan->fifo->hfifo, &cur);
            break;
        case AAI_FIFO_16W_32b:
            read_32samples_from_fifo(aai->iobase+chan->fifo->hfifo, &cur);
            break;
        default:
            aai_err(aai->dev, "Wrong mode in FIFO transfer\n");
            break;
    }
}

void aai_process_tx(aai_device_t *chan)
{
    char *cur;
    char *fifo_cur;
    struct card_data_t *aai = chan->pdrvdata;
    struct snd_pcm_substream *substream;

    aai_hwchan_t *fifo  = &(chan->fifo[0]);

    substream = chan2pbacksubstream(aai, chan->ipcm);
    fifo_cur = fifo->pcurrent - chan->dmaxfersz;
    if (fifo_cur < fifo->pstart)
        fifo_cur += fifo->pend - fifo->pstart + 1;
    cur = (char *)substream->runtime->dma_area - fifo->pstart + fifo_cur;

    /*
     * Processing FIFO mode
     */
    switch (chan->mode)
    {
        case AAI_FIFO_2W_32b:
            write_4samples_from_fifo(aai->iobase+chan->fifo->hfifo, &cur);
            break;
        case AAI_FIFO_4W_32b:
            write_8samples_from_fifo(aai->iobase+chan->fifo->hfifo, &cur);
            break;
        case AAI_FIFO_8W_32b:
            write_16samples_from_fifo(aai->iobase+chan->fifo->hfifo, &cur);
            break;
        case AAI_FIFO_16W_32b:
            write_32samples_from_fifo(aai->iobase+chan->fifo->hfifo, &cur);
            break;
        default:
            aai_err(aai->dev, "Wrong mode in FIFO transfer\n");
            break;
    }
}

void aai_process_tx_interleaved(aai_device_t *chan)
{
    char *cur;
    struct card_data_t *aai = chan->pdrvdata;
    struct snd_pcm_substream *substream;

    aai_hwchan_t *fifol = &(chan->fifo[0]);
    aai_hwchan_t *fifor = &(chan->fifo[1]);

    substream = chan2pbacksubstream(aai, chan->ipcm);

    cur = substream->runtime->dma_area + (fifol->pcurrent - fifol->pstart)
                                       + (fifor->pcurrent - fifor->pstart);

    write_interleaved_from_fifo(aai->iobase + chan->fifo[0].hfifo,
                                aai->iobase + chan->fifo[1].hfifo,
                                &cur);

    fifol->pcurrent = fifol->pfollow;
    fifol->pfollow = fifol->pcurrent + chan->dmaxfersz;
    if (fifol->pfollow >= fifol->pend)
        fifol->pfollow = fifol->pstart;

    fifor->pcurrent = fifor->pfollow;
    fifor->pfollow = fifor->pcurrent + chan->dmaxfersz;
    if (fifor->pfollow >= fifor->pend)
        fifor->pfollow = fifor->pstart;
}

/**
 * Inform Alsa driver that a period size has been reached
 *
 * @param channel       card's pointer
 * @param channel       channel's pointer
 *
 * @return none
 */
void aai_period_elapsed(struct card_data_t *aai, aai_device_t *chan)
{
    if (chan->access == AAI_NONINTERLEAVED)
    {
        chan->bytes_count += 2*chan->dmaxfersz;
        aai_process_tx_interleaved(chan);
    }

    if (chan->bytes_count >= chan->period_bytes)
    {
        /*
         * In this case, AAI IRQ is not acknowledge in the FIQ.
         * Do it now by writing the follow address in DMAFA reg.
         */
        if (chan->mode == AAI_DMA_XFER)
        {
            aai_hwchan_t *fifo = &(chan->fifo[0]);
            writel(fifo->pfollow, aai->iobase + fifo->dmafa);
        }
        else
        {
            if (chan->direction == AAI_RX)
            {
                aai_process_rx(chan);
            }
            else
            {
                if (chan->mode != AAI_FIFO_16W_64b)
                    aai_process_tx(chan);
            }
        }

        if (chan->direction == AAI_RX)
        {
            snd_pcm_period_elapsed(chan2captsubstream(aai, chan->ipcm));
            chan->nbper++;
        }
        else if (chan->direction == AAI_TX)
        {
            snd_pcm_period_elapsed(chan2pbacksubstream(aai, chan->ipcm));
            chan->nbper++;
        }

        chan->bytes_count -= chan->period_bytes;
    }
}

