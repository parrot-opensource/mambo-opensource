/**
 ******************************************************************************
 * @file ultra_snd.h
 * @brief Delos ultra sound(aai + spi) header
 *
 * Copyright(C) 2013 Parrot S.A.
 *
 * @author     Samir Ammenouche <samir.ammenouche@parrot.com>
 * @date       2013-04-02
 ******************************************************************************
 */
#ifndef _ULTRA_SND_H
#define _ULTRA_SND_H 1

#include <linux/types.h>

#define ULTRA_SND_NAME	"ultra_snd"
/* SIZEMAX: Maximum size of ADC DMA-able memory 256KB */
#define SIZEMAX		(1 << 18)
#define AAI_DMA_CTRL_ULTRA	(1 << 11)
#define MAXDMA		13

#define USND_MAX_PULSES		1024
#define USND_MAX_RAW_BUFLEN	4096

struct usnd_raw_buf {
	/* Pointer to sample data. Only the 8 lower bytes of the uint32_t are
	 * used as SPI data. */
	uint32_t *buf;
	/* Sample buffer length in bytes. From 0 to USND_MAX_RAW_BUFLEN. */
	uint16_t length;
	/* THOLDCS value to use when transmitting the buffer. From 0x0 to 0xf.
	 * Delay between bytes is (THOLDCS + 1) SCLK periods. Set to 0 if
	 * unsure. */
	uint8_t tholdcs;
};

#define USND_IO_MAGIC 'u'
#define USND_SETSIZE        _IOW(USND_IO_MAGIC, 1, int)
#define USND_COPYSAMPLE     _IOR(USND_IO_MAGIC, 2, int*)
/* Send n ultrasound pulses at 40 KHz. n must be less than USND_MAX_PULSES. */
#define USND_PULSES         _IOW(USND_IO_MAGIC, 3, unsigned int)
/* Send raw bitstream at 641990.7 bps through the ultrasound capsule. */
#define USND_RAW            _IOW(USND_IO_MAGIC, 4, struct usnd_raw_buf)
#define BATTERY             _IOR(USND_IO_MAGIC, 5, int)
#define TEMPERATURE         _IOR(USND_IO_MAGIC, 6, int)
#define BATTERY_INIT        _IO(USND_IO_MAGIC, 7)
#define TEMPERATURE_INIT    _IO(USND_IO_MAGIC, 8)

#define USND_IOC_MAXNR 8

#endif
