
#ifndef _PARROT6I_H
#define _PARROT6I_H 1
void p6i_init(void);
void sip6_init(int bt_on);
void p6i_map_io(void);
void p6i_eth_on_jtag_init(void);
void p6i_set_pads_sdcard(uint32_t freq);
void p6i_set_pads_i2c(int num);
void p6i_set_i2c_drive_strength(int num, int val);
void p6i_set_usb_drive_strength(int param, int val);
void p6i_set_pads_uart1_pullup(void);
#ifdef CONFIG_GPIOLIB
int p6i_export_gpio(int gpio, unsigned int flags, char const* label, bool direction_may_change);
void p6i_unexport_gpio(int gpio);
#endif /* CONFIG_GPIOLIB */

#define P6I_UART0_RXTX_DEFAULT \
	P6I_UART0_RX, \
	P6I_UART0_TX

#define P6I_UART0_DEFAULT \
	P6I_UART0_RXTX_DEFAULT, \
	P6I_UART0_RTS, \
	P6I_UART0_CTS

#define P6I_UART1_RXTX_DEFAULT \
	P6I_UART1_RXa, \
	P6I_UART1_TXa

#define P6I_UART1_DEFAULT \
	P6I_UART1_RXTX_DEFAULT, \
	P6I_UART1_RTSa, \
	P6I_UART1_CTSa

#define P6I_UART2_RXTX_DEFAULT \
	P6I_UART2_RX, \
	P6I_UART2_TX

#define P6I_UART2_DEFAULT \
	P6I_UART2_RXTX_DEFAULT, \
	P6I_UART2_RTS, \
	P6I_UART2_CTS

#define P6I_SPI0_DEFAULT \
    P6I_SPI0_CLKa, \
    P6I_SPI0_SSa, \
    P6I_SPI0_MOSIa, \
    P6I_SPI0_MISOa

#define P6I_SPI1_DEFAULT \
    P6I_SPI1_CLK,   \
    P6I_SPI1_SS,    \
    P6I_SPI1_MOSI,  \
    P6I_SPI1_MISO


#define P6I_NAND8_DEFAULT \
	GPIO_029, \
	P6I_NAND_nW, \
	P6I_NAND_AL, \
	P6I_NAND_CL, \
	P6I_NAND_nCE, \
	P6I_NAND_RnB, \
	P6I_NAND_nR, \
	P6I_NAND_D0, \
	P6I_NAND_D1, \
	P6I_NAND_D2, \
	P6I_NAND_D3, \
	P6I_NAND_D4, \
	P6I_NAND_D5, \
	P6I_NAND_D6, \
	P6I_NAND_D7

#define P6I_I2CM0_DEFAULT \
	P6I_I2C0M_SCL, \
	P6I_I2C0M_SDA

#define P6I_I2CM1_DEFAULT \
	P6I_I2C1_SCL, \
	P6I_I2C1_SDA

#define P6I_SD0_DEFAULT \
	P6I_SD0_CLK, \
	P6I_SD0_CMD, \
	P6I_SD0_DAT0, \
	P6I_SD0_DAT1, \
	P6I_SD0_DAT2, \
	P6I_SD0_DAT3

#define P6I_SPI1_DEFAULT \
	P6I_SPI1_CLK, \
	P6I_SPI1_SS, \
	P6I_SPI1_MOSI, \
	P6I_SPI1_MISO

#define P6I_SPI2_DEFAULT \
	P6I_SPI2_CLKb, \
	P6I_SPI2_SSb, \
	P6I_SPI2_MOSIb, \
	P6I_SPI2_MISOb

#define P6I_MMC_GPIO_DEFAULT \
	P6I_GPIO_001, \
	P6I_GPIO_002

#define P6I_AAI_I2S_SYNC_DEFAULT \
	P6I_I2S_CLK, \
	P6I_I2S_SYNC, \
	P6I_MCLK

#define P6I_AAI_PCM0_DEFAULT \
	P6I_PCM_RX, \
	P6I_PCM_TX, \
	P6I_PCM_CLK, \
	P6I_PCM_FRM


#endif
