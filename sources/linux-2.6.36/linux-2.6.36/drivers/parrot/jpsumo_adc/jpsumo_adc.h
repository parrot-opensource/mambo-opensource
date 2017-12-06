/**
 ******************************************************************************
 * @file jpsumo_adc.h
 * @brief Jumping Sumo ADC (aai) header
 *
 * Copyright(C) 2017 Parrot S.A.
 *
 * @author     Samir Ammenouche <samir.ammenouche@parrot.com>
 * @author     Hugo Grostabussiat <hugo.grostabussiat@parrot.com>
 * @date       2017-02-22
 ******************************************************************************
 */
#ifndef _JPSUMO_ADC_H
#define _JPSUMO_ADC_H

#include <linux/types.h>

enum jsadc_src {
	JSADC_SRC_VBAT = 0,
	JSADC_SRC_IJUMP,
	JSADC_SRC_LWHEEL,
	JSADC_SRC_RWHEEL
};

struct jsadc_timed_buf {
	int index;
	unsigned int missed;
	enum jsadc_src src;
};

struct jsadc_data {
	uint8_t has_vbat : 1;
	uint8_t has_ijump : 1;
	uint8_t has_lwheel : 1;
	uint8_t has_rwheel : 1;
	uint16_t vbat; /*< read on adc 0 (muxed with battery) */
	uint16_t ijump;
	uint16_t lwheel; /*< read on ADC1, unless 1 & 2 swapped */
	uint16_t rwheel; /*< read on ADC2, unless 1 & 2 swapped */
};


#define JSADC_DEVICE_NAME	"jpsumo_adc"
/* Maximum DMA transfer size allowed by the hardware: 8KB */
#define JSADC_DMA_SHIFT		13
#define JSADC_DMA_SIZE		(1 << JSADC_DMA_SHIFT)
/* Size of ADC DMA-able memory 256KB */
#define JSADC_NUM_BUFS		32
#define JSADC_SIZE_ALLBUFS	(JSADC_DMA_SIZE * JSADC_NUM_BUFS)

#define JSADC_IO_MAGIC 'j'
/* Return data for all ADC channels (blocking) */
#define JSADC_GET_DATA      _IOWR(JSADC_IO_MAGIC, 1, struct jsadc_data)
/* Set source input for ADC streming */
#define JSADC_SET_SRC       _IOW(JSADC_IO_MAGIC, 2, enum jsadc_src)
/* Start ADC data streaming */
#define JSADC_START         _IO(JSADC_IO_MAGIC, 3)
/* Attempt to get a buffer full of ADC samples
 * If no buffer is available, returns -1 with errno set to EAGAIN. */
#define JSADC_GETBUFFER     _IOR(JSADC_IO_MAGIC, 4, struct jsadc_timed_buf)
/* Release a buffer acquired with JSADC_GETBUFFER, specified by its offset
 * Buffers must be released in order. */
#define JSADC_RELEASEBUFFER _IOW(JSADC_IO_MAGIC, 5, int)
/* Stop ADC data streaming and release all buffers */
#define JSADC_STOP          _IO(JSADC_IO_MAGIC, 6)

#define JSADC_IOC_MAXNR 6

#endif
