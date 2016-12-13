#include "aai.h"
#include "aai_hw.h"
#include "aai_hw_p6.h"
#include "aai_irq_dma.h"

irqreturn_t aai_irq_p6(int irq, void *dev)
{
    struct card_data_t * aai = dev;
    uint32_t itsrc = readl(aai->iobase + AAI_ITS);
    int32_t ipcm;

    /*
     * Check for DMA error
     */
    if (itsrc & AAI_ITS_DMA_ERROR)
    {
        aai_err(aai->dev,"DMA failed 0x08%x\n", itsrc);
    }

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

