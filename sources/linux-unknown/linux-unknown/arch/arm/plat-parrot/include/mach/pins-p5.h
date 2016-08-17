/**
 * @file linux/include/asm-arm/arch-parrot/pins-p5.h
 * @brief Parrot5/5+ pins
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     yves.lemoine@parrot.com
 * @author     ivan.djelic@parrot.com
 * @date       2005-11-17
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ARCH_ARM_PARROT5_PINS_H
#define __ARCH_ARM_PARROT5_PINS_H

/* This list defines 16-bit pin IDs with the following encoding:
 *
 * offset = offset of SYS_IOCR reg used for mux choice
 * shift  = shift of SYS_IOCR reg 2-bit field used for mux choice
 * value  = value of SYS_IOCR 2-bit field for choosing this pin mux
 *
 * id = ((offset & 0xff) << 8) | (((shift/2) & 0xf) << 4) | (value & 0x3)
 *
 * Example: GPIO18 is enabled by programming 0 in the 2-bit field
 * SYS_IOCR2[5:4], hence:
 *
 * offset = 0x44
 * shift  = 4
 * value  = 0
 * => id  = 0x4420
 */

/* IOs configured with _P5_SYS_IOCR1 */
#define GPIO00                     (0x4000)
#define A18                        (0x4001)

#define GPIO01                     (0x4010)
#define A19                        (0x4011)

#define GPIO02                     (0x4020)
#define A20                        (0x4021)

#define GPIO03                     (0x4030)
#define A21                        (0x4031)
#define SPI_SS_                    (0x4032)
#define SPI_SSb                    (0x4032)

#define GPIO04                     (0x4040)
#define A22                        (0x4041)
#define SPI_CLK_                   (0x4042)
#define SPI_CLKb                   (0x4042)

#define GPIO05                     (0x4050)
#define A23                        (0x4051)
#define SPI_MOSI_                  (0x4052)
#define SPI_MOSIb                  (0x4052)

#define GPIO06                     (0x4060)
#define A24                        (0x4061)
#define SPI_MISO_                  (0x4062)
#define SPI_MISOb                  (0x4062)

#define GPIO07                     (0x4070)
#define A25                        (0x4071)

#define GPIO08                     (0x4080)
#define nCS0                       (0x4081)

#define GPIO09                     (0x4090)
#define nCS2                       (0x4091)
#define UART2_RTSc                 (0x4092)

#define GPIO10                     (0x40A0)
#define nCS3                       (0x40A1)
#define UART2_CTSc                 (0x40A2)

#define GPIO11                     (0x40B0)
#define CAS                        (0x40B1)

#define GPIO12                     (0x40C0)
#define RAS                        (0x40C1)

#define GPIO13                     (0x40D0)
#define CAN_RX                     (0x40D1)
#define PWM4a                      (0x40D2)

#define GPIO14                     (0x40E0)
#define CAN_TX                     (0x40E1)
#define PWM5a                      (0x40E2)

#define GPIO15                     (0x40F0)
#define VSYNC                      (0x40F1)
#define UART2_RX                   (0x40F2)
#define UART2_RXb                  (0x40F2)
#define AC_OUT                     (0x40F3)
#define AC_OUT2b                   (0x40F3)

/* IOs configured with _P5_SYS_IOCR2 */
#define GPIO16                     (0x4400)
#define SCL                        (0x4401)
#define SCL0                       (0x4401)
#define PCLK                       (0x4402)
#define MCLK                       (0x4403)
#define MCLKb                      (0x4403)

#define GPIO17                     (0x4410)
#define PDAT0                      (0x4411)
#define UART2_TX                   (0x4412)
#define UART2_TXb                  (0x4412)
#define AC_IN                      (0x4413)
#define AC_IN2b                    (0x4413)

#define GPIO18                     (0x4420)
#define SDA                        (0x4421)
#define SDA0                       (0x4421)
#define PDAT1                      (0x4422)
#define AC_OUTS                    (0x4423)
#define AC_OUT1b                   (0x4423)

#define GPIO19                     (0x4430)
#define PDAT2                      (0x4431)
#define AC_CLK                     (0x4432)
#define AC_CLKb                    (0x4432)

#define GPIO20                     (0x4440)
#define PDAT3                      (0x4441)
#define AC_SYNC                    (0x4442)
#define AC_SYNCb                   (0x4442)

#define GPIO21                     (0x4450)
#define PDAT4                      (0x4451)
#define PCM_OUT                    (0x4452)
#define PCM_OUTb                   (0x4452)
#define PCM_BT_OUTb                (0x4453)

#define GPIO22                     (0x4460)
#define PDAT5                      (0x4461)
#define PCM_IN                     (0x4462)
#define PCM_INb                    (0x4462)
#define PCM_BT_INb                 (0x4463)

#define GPIO23                     (0x4470)
#define PDAT6                      (0x4471)
#define PCM_CLK                    (0x4472)
#define PCM_CLKb                   (0x4472)
#define PCM_BT_CLKb                (0x4473)

#define GPIO24                     (0x4480)
#define PDAT7                      (0x4481)
#define PCM_SYNC                   (0x4482)
#define PCM_SYNCb                  (0x4482)
#define PCM_BT_SYNCb               (0x4483)

#define GPIO25                     (0x4490)
#define HSYNC                      (0x4491)
#define SIM0_IO                    (0x4492)

#define GPIO26                     (0x44A0)
#define SIM_CLK                    (0x44A1)

#define GPIO27                     (0x44B0)
#define SIM1_IO                    (0x44B1)
#define PWM6b                      (0x44B2)

#define GPIO28                     (0x44C0)
#define SIM2_IO                    (0x44C1)
#define PWM7b                      (0x44C2)
#define CLK32K                     (0x44C3)

#define GPIO29                     (0x44D0)
#define SPI_SS                     (0x44D1)
#define MCLK_                      (0x44D2)

#define GPIO30                     (0x44E0)
#define MLBCLK                     (0x44E1)
#define SPI_CLK                    (0x44E2)

#define GPIO31                     (0x44F0)
#define MLBSIG                     (0x44F1)
#define SPI_MOSI                   (0x44F2)

/* IOs configured with _P5_SYS_IOCR3 */
#define GPIO32                     (0x4800)
#define MLBDAT                     (0x4801)
#define SPI_MISO                   (0x4802)

#define GPIO33                     (0x4810)
#define SD_CLK                     (0x4811)
#define SD0_CLK                    (0x4811)
#define UART2_RX_                  (0x4812)
#define UART2_RXa                  (0x4812)

#define GPIO34                     (0x4820)
#define SD_CMD                     (0x4821)
#define SD0_CMD                    (0x4821)
#define UART2_TX_                  (0x4822)
#define UART2_TXa                  (0x4822)

#define GPIO35                     (0x4830)
#define SD_DAT0                    (0x4831)
#define SD0_DAT0                   (0x4831)
#define PWM0                       (0x4832)
#define PWM0a                       (0x4832)

#define GPIO36                     (0x4840)
#define SD_DAT1                    (0x4841)
#define SD0_DAT1                   (0x4841)
#define PWM1                       (0x4842)
#define PWM1a                      (0x4842)

#define GPIO37                     (0x4850)
#define SD_DAT2                    (0x4851)
#define SD0_DAT2                   (0x4851)
#define PWM2                       (0x4852)
#define PWM2a                      (0x4852)

#define GPIO38                     (0x4860)
#define SD_DAT3                    (0x4861)
#define SD0_DAT3                   (0x4861)
#define PWM3                       (0x4862)
#define PWM3a                      (0x4862)

#define GPIO39                     (0x4870)
#define BXRD                       (0x4871)
#define SD1_CLK                    (0x4871)

#define GPIO40                     (0x4880)
#define BTXEN                      (0x4881)
#define SD1_CMD                    (0x4881)

#define GPIO41                     (0x4890)
#define BRXEN                      (0x4891)
#define SD1_DAT0                   (0x4891)

#define GPIO42                     (0x48A0)
#define BMISO                      (0x48A1)
#define SD1_DAT2                   (0x48A1)

#define GPIO43                     (0x48B0)
#define PWM0_                      (0x48B1)
#define PWM0b                      (0x48B1)
#define UART0_RTS                  (0x48B2)

#define GPIO44                     (0x48C0)
#define PWM1_                      (0x48C1)
#define PWM1b                      (0x48C1)
#define UART0_CTS                  (0x48C2)

#define GPIO45                     (0x48D0)
#define PWM2_                      (0x48D1)
#define PWM2b                      (0x48D1)
#define UART1_RTS                  (0x48D2)

#define GPIO46                     (0x48E0)
#define PWM3_                      (0x48E1)
#define PWM3b                      (0x48E1)
#define UART1_CTS                  (0x48E2)
#define AC_INS                     (0x48E3)
#define AC_IN1b                    (0x48E3)

#define GPIO47                     (0x48F0)
#define BSEN                       (0x48F1)
#define UART2_RX__                 (0x48F1)

/* IOs configured with _P5_SYS_IOCR5 */
#define GPIO48                     (0x5C00)
#define D16                        (0x5C01)
#define SD1_CLKc                   (0x5C02)

#define GPIO49                     (0x5C10)
#define D17                        (0x5C11)
#define SD1_CMDc                   (0x5C12)

#define GPIO50                     (0x5C20)
#define D18                        (0x5C21)
#define SD1_DAT0c                  (0x5C22)

#define GPIO51                     (0x5C30)
#define D19                        (0x5C31)
#define SD1_DAT1c                  (0x5C32)

#define GPIO52                     (0x5C40)
#define D20                        (0x5C41)
#define SD1_DAT2c                  (0x5C42)

#define GPIO53                     (0x5C50)
#define D21                        (0x5C51)
#define SD1_DAT3c                  (0x5C52)

#define GPIO54                     (0x5C60)
#define D22                        (0x5C61)
#define PCM_BT_INc                 (0x5C62)
#define MCLKc                      (0x5C63)

#define GPIO55                     (0x5C70)
#define D23                        (0x5C71)
#define PCM_BT_OUTc                (0x5C72)
#define PCM_OUTc                   (0x5C73)

#define GPIO56                     (0x5C80)
#define D24                        (0x5C81)
#define PCM_BT_CLKc                (0x5C82)
#define PCM_CLKc                   (0x5C83)

#define GPIO57                     (0x5C90)
#define D25                        (0x5C91)
#define PCM_BT_SYNCc               (0x5C92)
#define PCM_SYNCc                  (0x5C93)

#define GPIO58                     (0x5CA0)
#define D26                        (0x5CA1)
#define AC_IN2c                    (0x5CA2)
#define PCM_INc                    (0x5CA3)

#define GPIO59                     (0x5CB0)
#define D27                        (0x5CB1)
#define AC_IN1c                    (0x5CB2)

#define GPIO60                     (0x5CC0)
#define D28                        (0x5CC1)
#define AC_CLKc                    (0x5CC3)

#define GPIO61                     (0x5CD0)
#define D29                        (0x5CD1)
#define AC_SYNCc                   (0x5CD3)

#define GPIO62                     (0x5CE0)
#define D30                        (0x5CE1)
#define UART2_RXc                  (0x5CE2)

#define GPIO63                     (0x5CF0)
#define D31                        (0x5CF1)
#define UART2_TXc                  (0x5CF2)

/* IOs configured with _P5_SYS_IOCR6 */
#define GPIO64                     (0x6000)
#define MCLKa                      (0x6001)
#define PCM_CLKa                   (0x6002)

#define GPIO65                     (0x6010)
#define AC_OUT1a                   (0x6011)

#define GPIO66                     (0x6020)
#define AC_OUT2a                   (0x6021)
#define PCM_OUTa                   (0x6022)

#define GPIO67                     (0x6030)
#define AC_IN1a                    (0x6031)

#define GPIO68                     (0x6040)
#define AC_IN2a                    (0x6041)
#define PCM_INa                    (0x6042)

#define GPIO69                     (0x6050)
#define AC_IN1Sa                   (0x6051)
#define PCM_SYNCa                  (0x6052)

#define GPIO70                     (0x6060)
#define AC_CLKa                    (0x6061)

#define GPIO71                     (0x6070)
#define AC_SYNCa                   (0x6071)

#define GPIO72                     (0x6080)
#define UART0_RX                   (0x6081)

#define GPIO73                     (0x6090)
#define UART0_TX                   (0x6091)

#define GPIO74                     (0x60A0)
#define UART1_RX                   (0x60A1)

#define GPIO75                     (0x60B0)
#define UART1_TX                   (0x60B1)

#define GPIO76                     (0x60C0)
#define SCL1                       (0x60C1)
#define PWM6a                      (0x60C2)

#define GPIO77                     (0x60D0)
#define SDA1                       (0x60D1)
#define PWM7a                      (0x60D2)

#define GPIO78                     (0x60E0)
#define UART2_CTS                  (0x60E1)
#define SD1_DAT1                   (0x60E2)

#define GPIO79                     (0x60F0)
#define UART2_RTS                  (0x60F1)
#define SD1_DAT3                   (0x60F2)

#endif /* __ARCH_ARM_PARROT5_PINS_H */
