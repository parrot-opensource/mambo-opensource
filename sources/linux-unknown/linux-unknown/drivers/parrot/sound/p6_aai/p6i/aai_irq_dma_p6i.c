#include "aai.h"
#include "aai_hw.h"
#include "aai_hw_p6i.h"
#include "aai_ioctl.h"
#include "aai_irq_dma.h"

static uint32_t buf[AAI_NB_ADC] = {0};

#ifdef CONFIG_AAI_IRQ_SHARED
irqreturn_t aai_irq_p6i(int irq, void *dev)
{
    struct card_data_t * aai = dev;
    uint32_t itsrc = readl(aai->iobase+AAI_ITS);
    int32_t ipcm;
    aai_device_t *chan;
    aai_hwchan_t *fifo;


    if (itsrc & AAI_ITS_DMA_ERROR)
        aai_err(aai->dev, "DMA failed 0x08%x\n", itsrc);

    if (itsrc & AAI_ITS_MONITOR)
        return IRQ_HANDLED;

    /*
     * Try to process every channel
     */
    for (ipcm = 0; ipcm < AAI_NB_CHANNELS; ipcm++)
    {
        if (itsrc & (aai->chans[ipcm].intflag))
        {
            chan = &aai->chans[ipcm];
            fifo = &(chan->fifo[0]);

            chan->nbirq++;

            /* update fifo pointers */
            fifo->pcurrent = fifo->pfollow;
            fifo->pfollow = fifo->pcurrent + chan->dmaxfersz;
            if (fifo->pfollow >= fifo->pend)
                fifo->pfollow = fifo->pstart;

            /* period elapsed */
            chan->bytes_count += chan->dmaxfersz;
            if (chan->bytes_count >= chan->period_bytes)
            {
                if (chan->direction == AAI_RX)
                    snd_pcm_period_elapsed(chan2captsubstream(aai, chan->ipcm));
                else if (chan->direction == AAI_TX)
                    snd_pcm_period_elapsed(chan2pbacksubstream(aai, chan->ipcm));
                chan->nbper++;
                chan->bytes_count -= chan->period_bytes;
            }

            /* ack interrupt and prepare next DMA transfer */
            writel(fifo->pfollow, aai->iobase + fifo->dmafa);
        }
    }

    return IRQ_HANDLED;
}
#else
irqreturn_t aai_irq_p6i(int irq, void *dev)
{
    struct card_data_t * aai = dev;
    uint32_t itsrc = readl(aai->iobase+AAI_ITS);
    int32_t ipcm;

    if (itsrc & AAI_ITS_DMA_ERROR)
        aai_err(aai->dev,"DMA failed 0x08%x\n", itsrc);

    if (itsrc & AAI_ITS_MONITOR)
        return IRQ_HANDLED;

    /*
     * Try to process every channel
     */
    for (ipcm = 0; ipcm < AAI_NB_CHANNELS; ipcm++)
    {
        if (itsrc & (aai->chans[ipcm].intflag))
        {
            aai->chans[ipcm].nbirq++;
            aai_period_elapsed(aai, &aai->chans[ipcm]);
        }
    }

    /*
     * Clear interruptions
     */
    writel((1 << aai->irq), PARROT6_VA_VIC + VIC_INT_SOFT_CLEAR);
    writel((1 << aai->fiq), PARROT6_VA_VIC + VIC_INT_ENABLE);

    return IRQ_HANDLED;
}
#endif

uint32_t* aai_adc_getdata(void)
{
    return &buf[0];
}

