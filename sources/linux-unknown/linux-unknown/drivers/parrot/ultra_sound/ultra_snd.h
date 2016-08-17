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

struct timed_buf {
	uint32_t offset;
	uint64_t cycles;
};

struct adc_data_t {
	uint16_t ijump;     /*< read on adc 0 (muxed with battery) */
	uint16_t wheel50hz; /*< read on adc 2 */
};


#define ULTRA_SND_NAME	"ultra_snd"
/* SIZEMAX: Maximum size of ADC DMA-able memory 256KB */
#define SIZEMAX		(1 << 18)
#define AAI_DMA_CTRL_ULTRA	(1 << 11)
#define SPIMAX		0x100
#define MAXDMA		13

/* SPI base address */
#define P6_SPI1_BAD	0xd00c0000
#define SPI_CTRL	0x0000
#define SPI_SPEED	0x0004
#define SPI_STATUS	0x0008
#define SPI_SIZE	0x000c
#define SPI_THRES_RX	0x0010
#define SPI_THRES_TX	0x0014
#define SPI_DATA	0x0040

#define aai_writel(_value_, _reg_)\
	writel(_value_, _reg_ + u_snd.map)

#define aai_readl(_reg_) readl(_reg_ + u_snd.map)

#define spi_writel(_value_, _reg_)\
	writel(_value_, _reg_ + spi_dev.map)

#define spi_readl(_reg_) readl(_reg_ + spi_dev.map)


#define USND_IO_MAGIC 'u'
#define USND_SETSIZE        _IOW(USND_IO_MAGIC, 1, int)
#define USND_COPYSAMPLE     _IOR(USND_IO_MAGIC, 2, int*)
#define USND_SPI_LEN        _IOW(USND_IO_MAGIC, 3, int)
#define USND_SPI_DAT        _IOW(USND_IO_MAGIC, 4, int)
#define BATTERY             _IOR(USND_IO_MAGIC, 5, int)
#define TEMPERATURE         _IOR(USND_IO_MAGIC, 6, int)

#define JS_INIT             _IO(USND_IO_MAGIC, 6)
#define JS_GETBUFFER        _IOR(USND_IO_MAGIC, 7, struct timed_buf)
#define JS_RELEASEBUFFER    _IOW(USND_IO_MAGIC, 8, int)
#define JS_START            _IO(USND_IO_MAGIC, 9)
#define JS_START_8K         _IO(USND_IO_MAGIC, 10)
#define JS_STOP             _IO(USND_IO_MAGIC, 11)
#define JS_GET_DATA         _IOR(USND_IO_MAGIC, 12, struct adc_data_t)
#define BATTERY_INIT        _IO(USND_IO_MAGIC, 13)
#define JS_MUX_ADC_1_2      _IOW(USND_IO_MAGIC, 14, uint32_t)
#define TEMPERATURE_INIT    _IO(USND_IO_MAGIC, 15)

#define USND_IOC_MAXNR 14

enum pulse_mode {
	FOUR_PULSE = 0,
	EIGHT_PULSE,
	SIXTEEN_PULSE,
	TWELVE_FIVE_DEPHASED_PULSE,
	SIXTEEN_FOUR_DEPHASED_PULSE,
};

#endif
