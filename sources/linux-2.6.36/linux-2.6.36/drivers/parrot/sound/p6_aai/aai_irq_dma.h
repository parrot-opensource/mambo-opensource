
#ifndef INCONCE_AAI_IRQ_DMA
#define INCONCE_AAI_IRQ_DMA

/*
 * Common DMA functions
 */
void aai_process_rx(aai_device_t *chan);
void aai_process_tx(aai_device_t *chan);
void aai_dmaxfer(aai_device_t *chan, void *iobase, uint32_t ififo);
void aai_period_elapsed(struct card_data_t *aai, aai_device_t *chan);

#endif

