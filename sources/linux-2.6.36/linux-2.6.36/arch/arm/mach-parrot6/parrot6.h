
#ifndef _PARROT6_H
#define _PARROT6_H 1
#include <linux/i2c.h>

extern int parrot_force_usb_device;

void p6_init(void);
void p6_init_irq(void);
void p6_map_io(void);
void eth_on_jtag_init(void);
void eth_on_spi0_init(void);
void p6_device_release(struct device *dev);
void p6_device_init(struct device *dev, void *platform_data);
extern struct p6fb_mach_info p6_lcd_devices_info_7p_avea;
extern struct p6fb_mach_info p6_lcd_devices_info_10p_parelia;
extern struct p6fb_mach_info p6_lcd_devices_info_rnb4;
extern struct p6fb_mach_info p6_lcd_devices_info_fc6xxx;
int i2c0_write(int addr, int reg, int val);
int i2c0_read(int addr, int reg);
int clocks_init_p6(void);
extern struct i2c_board_info p6mu_rtc_i2c_board_info [1];

#define UART0_RXTX_DEFAULT \
	UART0_RX, \
	UART0_TX

#define UART0_DEFAULT \
	UART0_RXTX_DEFAULT, \
	UART0_RTS, \
	UART0_CTS

#define UART1_RXTX_DEFAULT \
	UART1_RX, \
	UART1_TX

#define UART1_DEFAULT \
	UART1_RXTX_DEFAULT, \
	UART1_RTS, \
	UART1_CTS

#define UART2_RXTX_DEFAULT \
	UART2_RX, \
	UART2_TX

#define UART2_DEFAULT \
	UART2_RXTX_DEFAULT, \
	UART2_RTS, \
	UART2_CTS

#define UART3_RXTX_DEFAULT \
	UART3_RX, \
	UART3_TX

#define UART3_DEFAULT \
	UART3_RXTX_DEFAULT, \
	UART3_RTS, \
	UART3_CTS


#define I2CM0_DEFAULT \
	SCL0, \
	SDA0

#define I2CM1_DEFAULT \
	SCL2, \
	SDA2

#define USB0_DEFAULT \
	USB_CLK, \
	ULPI0_CLK, \
	ULPI0_DAT0, \
	ULPI0_DAT1, \
	ULPI0_DAT2, \
	ULPI0_DAT3, \
	ULPI0_DAT4, \
	ULPI0_DAT5, \
	ULPI0_DAT6, \
	ULPI0_DAT7, \
	ULPI0_DIR, \
	ULPI0_STP, \
	ULPI0_NXT

#define USB1_DEFAULT \
	USB_CLK, \
	ULPI1_CLK, \
	ULPI1_DAT0, \
	ULPI1_DAT1, \
	ULPI1_DAT2, \
	ULPI1_DAT3, \
	ULPI1_DAT4, \
	ULPI1_DAT5, \
	ULPI1_DAT6, \
	ULPI1_DAT7, \
	ULPI1_DIR, \
	ULPI1_STP, \
	ULPI1_NXT

#define LCD18_DATA_DEFAULT \
	LCD_DAT00, \
	LCD_DAT01, \
	LCD_DAT02, \
	LCD_DAT03, \
	LCD_DAT04, \
	LCD_DAT05, \
	LCD_DAT06, \
	LCD_DAT07, \
	LCD_DAT08, \
	LCD_DAT09, \
	LCD_DAT10, \
	LCD_DAT11, \
	LCD_DAT12, \
	LCD_DAT13, \
	LCD_DAT14, \
	LCD_DAT15, \
	LCD_DAT16, \
	LCD_DAT17

#define LCD24_DATA_DEFAULT \
	LCD18_DATA_DEFAULT, \
	LCD_DAT18, \
	LCD_DAT19, \
	LCD_DAT20, \
	LCD_DAT21, \
	LCD_DAT22, \
	LCD_DAT23

#define LCD24_NODE_DEFAULT \
	LCD_VS, \
	LCD_HS, \
	LCD_CLK, \
	LCD24_DATA_DEFAULT

#define LCD24_DEFAULT \
	LCD_VS, \
	LCD_HS, \
	LCD_CLK, \
	LCD_DEN, \
	LCD24_DATA_DEFAULT

#define LCD18_DEFAULT \
	LCD_VS, \
	LCD_HS, \
	LCD_CLK, \
	LCD_DEN, \
	LCD18_DATA_DEFAULT

#define CAM0_DEFAULT \
	CAM0_HSYNC, \
	CAM0_VSYNC, \
	CAM0_CLK, \
	CAM0_DAT0, \
	CAM0_DAT1, \
	CAM0_DAT2, \
	CAM0_DAT3, \
	CAM0_DAT4, \
	CAM0_DAT5, \
	CAM0_DAT6, \
	CAM0_DAT7

#define CAM1_DEFAULT \
	CAM1_HSYNC, \
	CAM1_VSYNC, \
	CAM1_CLK, \
	CAM1_DAT0, \
	CAM1_DAT1, \
	CAM1_DAT2, \
	CAM1_DAT3, \
	CAM1_DAT4, \
	CAM1_DAT5, \
	CAM1_DAT6, \
	CAM1_DAT7

#define NAND8_DEFAULT \
	ND_nW, \
	ND_AL, \
	ND_CL, \
	ND_nCE, \
	ND_RnB, \
	ND_nR, \
	ND_IO00, \
	ND_IO01, \
	ND_IO02, \
	ND_IO03, \
	ND_IO04, \
	ND_IO05, \
	ND_IO06, \
	ND_IO07

#define NAND16_DEFAULT \
	ND_nW, \
	ND_AL, \
	ND_CL, \
	ND_nCE, \
	ND_RnB, \
	ND_nR, \
	ND_IO00, \
	ND_IO01, \
	ND_IO02, \
	ND_IO03, \
	ND_IO04, \
	ND_IO05, \
	ND_IO06, \
	ND_IO07, \
	ND_IO08, \
	ND_IO09, \
	ND_IO10, \
	ND_IO11, \
	ND_IO12, \
	ND_IO13, \
	ND_IO14, \
	ND_IO15

#define SD0_DEFAULT \
	SD0_CLK, \
	SD0_CMD, \
	SD0_DAT0, \
	SD0_DAT1, \
	SD0_DAT2, \
	SD0_DAT3

#define SD1_DEFAULT \
	SD1_CLK, \
	SD1_CMD, \
	SD1_DAT0, \
	SD1_DAT1, \
	SD1_DAT2, \
	SD1_DAT3

#define MC1_DEFAULT \
	MC1_CLK, \
	MC1_CMD, \
	MC1_DAT0, \
	MC1_DAT1, \
	MC1_DAT2, \
	MC1_DAT3

#define SPI0_DEFAULT \
	SPI_CLK, \
	SPI_MOSI, \
	SPI_MISO, \
	SPI_SS

#define SPI2_DEFAULT \
	SPI2_CLK, \
	SPI2_MOSI, \
	SPI2_MISO, \
	SPI2_SS

#define AAI_I2S_SYNC_DEFAULT \
	AC_CLKM, \
	AC_SYNCM, \
	MCLK

#define AAI_IO0_DEFAULT \
	AC_IN0, \
	AC_OUT0, \
	AAI_I2S_SYNC_DEFAULT

#define AAI_PCM0_DEFAULT \
	PCM_OUT0, \
	PCM_IN0, \
	PCM_SYNC0, \
	PCM_CLK0

#define MMC_GPIO_DEFAULT \
	GPIO_036, \
	GPIO_037

#define ETH_JTAG \
	GPIO_099

#endif
