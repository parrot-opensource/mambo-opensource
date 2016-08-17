/**
 * @file linux/include/asm-arm/arch-parrot/pins-p5.h
 * @brief Parrot6 pins
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2008-11-05
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
#ifndef __ARCH_ARM_PARROT6_PINS_H
#define __ARCH_ARM_PARROT6_PINS_H

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

#define  D16                               (0x5000)
#define  GPIO_000                          (0x5001)
#define  PWM00c                            (0x5002)

#define  D17                               (0x5010)
#define  GPIO_001                          (0x5011)
#define  PWM01c                            (0x5012)

#define  D18                               (0x5020)
#define  GPIO_002                          (0x5021)
#define  PWM02c                            (0x5022)

#define  D19                               (0x5030)
#define  GPIO_003                          (0x5031)
#define  PWM03c                            (0x5032)

#define  D20                               (0x5040)
#define  GPIO_004                          (0x5041)
#define  PWM04c                            (0x5042)

#define  D21                               (0x5050)
#define  GPIO_005                          (0x5051)
#define  PWM05c                            (0x5052)

#define  D22                               (0x5060)
#define  GPIO_006                          (0x5061)
#define  PWM06c                            (0x5062)

#define  D23                               (0x5070)
#define  GPIO_007                          (0x5071)
#define  PWM07c                            (0x5072)

#define  D24                               (0x5080)
#define  GPIO_008                          (0x5081)

#define  D25                               (0x5090)
#define  GPIO_009                          (0x5091)

#define  D26                               (0x50A0)
#define  GPIO_010                          (0x50A1)

#define  D27                               (0x50B0)
#define  GPIO_011                          (0x50B1)

#define  D28                               (0x50C0)
#define  GPIO_012                          (0x50C1)

#define  D29                               (0x50D0)
#define  GPIO_013                          (0x50D1)

#define  D30                               (0x50E0)
#define  GPIO_014                          (0x50E1)

#define  D31                               (0x50F0)
#define  GPIO_015                          (0x50F1)

#define  A12                               (0x5400)
#define  GPIO_016                          (0x5401)

#define  A13                               (0x5410)
#define  GPIO_017                          (0x5411)

#define  DM2                               (0x5420)
#define  GPIO_018                          (0x5421)

#define  DM3                               (0x5430)
#define  GPIO_019                          (0x5431)

#define  DQS2                              (0x5440)
#define  GPIO_020                          (0x5441)

#define  DQS3                              (0x5450)
#define  GPIO_021                          (0x5451)

#define  ND_nW                             (0x5460)
#define  GPIO_022                          (0x5461)
#define  SPI1_SS                           (0x5463)

#define  ND_AL                             (0x5470)
#define  GPIO_023                          (0x5471)
#define  SPI1_MISO                         (0x5473)

#define  ND_CL                             (0x5480)
#define  GPIO_024                          (0x5481)
#define  SPI1_MOSI                         (0x5483)

#define  ND_nCE                            (0x5490)
#define  GPIO_025                          (0x5491)

#define  ND_IO00                           (0x54A0)
#define  GPIO_026                          (0x54A1)
#define  SD2_DAT0a                         (0x54A2)
#define  MC2_DAT0a                         (0x54A3)

#define  ND_IO01                           (0x54B0)
#define  GPIO_027                          (0x54B1)
#define  SD2_DAT1a                         (0x54B2)
#define  MC2_DAT1a                         (0x54B3)

#define  ND_IO02                           (0x54C0)
#define  GPIO_028                          (0x54C1)
#define  SD2_DAT2a                         (0x54C2)
#define  MC2_DAT2a                         (0x54C3)

#define  ND_IO03                           (0x54D0)
#define  GPIO_029                          (0x54D1)
#define  SD2_DAT3a                         (0x54D2)
#define  MC2_DAT3a                         (0x54D3)

#define  ND_IO04                           (0x54E0)
#define  GPIO_030                          (0x54E1)
#define  SD2_CMDa                          (0x54E2)
#define  MC2_CMDa                          (0x54E3)

#define  ND_IO05                           (0x54F0)
#define  GPIO_031                          (0x54F1)
#define  SD2_CLKa                          (0x54F2)
#define  MC2_CLKa                          (0x54F3)

#define  ND_IO06                           (0x5800)
#define  GPIO_032                          (0x5801)

#define  ND_IO07                           (0x5810)
#define  GPIO_033                          (0x5811)

#define  ND_IO08                           (0x5820)
#define  GPIO_034                          (0x5821)

#define  ND_IO09                           (0x5830)
#define  GPIO_035                          (0x5831)
#define  PWM09b                            (0x5832)

#define  ND_IO10                           (0x5840)
#define  GPIO_036                          (0x5841)
#define  PWM10b                            (0x5842)

#define  ND_IO11                           (0x5850)
#define  GPIO_037                          (0x5851)
#define  PWM11b                            (0x5852)

#define  ND_IO12                           (0x5860)
#define  GPIO_038                          (0x5861)
#define  pRSTOUT                           (0x5862)

#define  ND_IO13                           (0x5870)
#define  GPIO_039                          (0x5871)

#define  ND_IO14                           (0x5880)
#define  GPIO_040                          (0x5881)

#define  ND_IO15                           (0x5890)
#define  GPIO_041                          (0x5891)

#define  SD0_CLK                           (0x58A0)
#define  GPIO_042                          (0x58A1)

#define  SD0_CMD                           (0x58B0)
#define  GPIO_043                          (0x58B1)

#define  SD0_DAT0                          (0x58C0)
#define  GPIO_044                          (0x58C1)

#define  SD0_DAT1                          (0x58D0)
#define  GPIO_045                          (0x58D1)

#define  SD0_DAT2                          (0x58E0)
#define  GPIO_046                          (0x58E1)

#define  SD0_DAT3                          (0x58F0)
#define  GPIO_047                          (0x58F1)

#define  SD1_CLK                           (0x5C00)
#define  GPIO_048                          (0x5C01)
#define  MC1_CLK                           (0x5C02)

#define  SD1_CMD                           (0x5C10)
#define  GPIO_049                          (0x5C11)
#define  MC1_CMD                           (0x5C12)

#define  SD1_DAT0                          (0x5C20)
#define  GPIO_050                          (0x5C21)
#define  MC1_DAT0                          (0x5C22)

#define  SD1_DAT1                          (0x5C30)
#define  GPIO_051                          (0x5C31)
#define  MC1_DAT1                          (0x5C32)

#define  SD1_DAT2                          (0x5C40)
#define  GPIO_052                          (0x5C41)
#define  MC1_DAT2                          (0x5C42)

#define  SD1_DAT3                          (0x5C50)
#define  GPIO_053                          (0x5C51)
#define  MC1_DAT3                          (0x5C52)

#define  LCD_VS                            (0x5C60)
#define  GPIO_054                          (0x5C61)
#define  PAR_nRS                           (0x5C62)
#define  CAM1_VSYNC                        (0x5C63)

#define  LCD_HS                            (0x5C70)
#define  GPIO_055                          (0x5C71)
#define  PAR_nCS                           (0x5C72)
#define  CAM1_HSYNC                        (0x5C73)

#define  LCD_CLK                           (0x5C80)
#define  GPIO_056                          (0x5C81)
#define  PAR_nR                            (0x5C82)
#define  CAM1_CLK                          (0x5C83)

#define  LCD_DEN                           (0x5C90)
#define  GPIO_057                          (0x5C91)
#define  PAR_nW                            (0x5C92)

#define  LCD_DAT00                         (0x5CA0)
#define  GPIO_058                          (0x5CA1)
#define  PAR_DB00                          (0x5CA2)
#define  PWM00a                            (0x5CA3)

#define  LCD_DAT01                         (0x5CB0)
#define  GPIO_059                          (0x5CB1)
#define  PAR_DB01                          (0x5CB2)
#define  PWM01a                            (0x5CB3)

#define  LCD_DAT02                         (0x5CC0)
#define  GPIO_060                          (0x5CC1)
#define  PAR_DB02                          (0x5CC2)
#define  PWM02a                            (0x5CC3)

#define  LCD_DAT03                         (0x5CD0)
#define  GPIO_061                          (0x5CD1)
#define  PAR_DB03                          (0x5CD2)
#define  PWM03a                            (0x5CD3)

#define  LCD_DAT04                         (0x5CE0)
#define  GPIO_062                          (0x5CE1)
#define  PAR_DB04                          (0x5CE2)
#define  PWM04a                            (0x5CE3)

#define  LCD_DAT05                         (0x5CF0)
#define  GPIO_063                          (0x5CF1)
#define  PAR_DB05                          (0x5CF2)
#define  PWM05a                            (0x5CF3)

#define  LCD_DAT06                         (0x6000)
#define  GPIO_064                          (0x6001)
#define  PAR_DB06                          (0x6002)
#define  PWM06a                            (0x6003)

#define  LCD_DAT07                         (0x6010)
#define  GPIO_065                          (0x6011)
#define  PAR_DB07                          (0x6012)
#define  PWM07a                            (0x6013)

#define  LCD_DAT08                         (0x6020)
#define  GPIO_066                          (0x6021)
#define  PAR_DB08                          (0x6022)
#define  PWM08a                            (0x6023)

#define  LCD_DAT09                         (0x6030)
#define  GPIO_067                          (0x6031)
#define  PAR_DB09                          (0x6032)
#define  PWM09a                            (0x6033)

#define  LCD_DAT10                         (0x6040)
#define  GPIO_068                          (0x6041)
#define  PAR_DB10                          (0x6042)
#define  PWM10a                            (0x6043)

#define  LCD_DAT11                         (0x6050)
#define  GPIO_069                          (0x6051)
#define  PAR_DB11                          (0x6052)
#define  PWM11a                            (0x6053)

#define  LCD_DAT12                         (0x6060)
#define  GPIO_070                          (0x6061)
#define  PAR_DB12                          (0x6062)
#define  PWM12a                            (0x6063)

#define  LCD_DAT13                         (0x6070)
#define  GPIO_071                          (0x6071)
#define  PAR_DB13                          (0x6072)
#define  PWM13a                            (0x6073)

#define  LCD_DAT14                         (0x6080)
#define  GPIO_072                          (0x6081)
#define  PAR_DB14                          (0x6082)
#define  PWM14a                            (0x6083)

#define  LCD_DAT15                         (0x6090)
#define  GPIO_073                          (0x6091)
#define  PAR_DB15                          (0x6092)
#define  PWM15a                            (0x6093)

#define  LCD_DAT16                         (0x60A0)
#define  GPIO_074                          (0x60A1)
#define  CAM1_DAT0                         (0x60A3)

#define  LCD_DAT17                         (0x60B0)
#define  GPIO_075                          (0x60B1)
#define  CAM1_DAT1                         (0x60B3)

#define  LCD_DAT18                         (0x60C0)
#define  GPIO_076                          (0x60C1)
#define  CAM1_DAT2                         (0x60C3)

#define  LCD_DAT19                         (0x60D0)
#define  GPIO_077                          (0x60D1)
#define  CAM1_DAT3                         (0x60D3)

#define  LCD_DAT20                         (0x60E0)
#define  GPIO_078                          (0x60E1)
#define  SD0_DAT4                          (0x60E2)
#define  CAM1_DAT4                         (0x60E3)

#define  LCD_DAT21                         (0x60F0)
#define  GPIO_079                          (0x60F1)
#define  SD0_DAT5                          (0x60F2)
#define  CAM1_DAT5                         (0x60F3)

#define  LCD_DAT22                         (0x6400)
#define  GPIO_080                          (0x6401)
#define  SD0_DAT6                          (0x6402)
#define  CAM1_DAT6                         (0x6403)

#define  LCD_DAT23                         (0x6410)
#define  GPIO_081                          (0x6411)
#define  SD0_DAT7                          (0x6412)
#define  CAM1_DAT7                         (0x6413)

#define  CAN_RX                            (0x6420)
#define  GPIO_082                          (0x6421)
#define  PWM00b                            (0x6422)
#define  SD0_LED                           (0x6423)

#define  CAN_TX                            (0x6430)
#define  GPIO_083                          (0x6431)
#define  PWM01b                            (0x6432)
#define  SD1_LED                           (0x6433)

#define  SCL0                              (0x6440)
#define  GPIO_084                          (0x6441)
#define  SCL1b                             (0x6442)

#define  SDA0                              (0x6450)
#define  GPIO_085                          (0x6451)
#define  SDA1b                             (0x6452)

#define  SCL1                              (0x6460)
#define  GPIO_086                          (0x6461)
#define  SCL2                              (0x6462)
#define  PCM_SYNC1                         (0x6463)

#define  SDA1                              (0x6470)
#define  GPIO_087                          (0x6471)
#define  SDA2                              (0x6472)
#define  PCM_CLK1                          (0x6473)

#define  CAM0_HSYNC                        (0x6480)
#define  GPIO_088                          (0x6481)
#define  AC_IN5                            (0x6482)
#define  CAM1b_HSYNC                       (0x6483)

#define  CAM0_VSYNC                        (0x6490)
#define  GPIO_089                          (0x6491)
#define  AC_SYNC25                         (0x6492)
#define  CAM1b_VSYNC                       (0x6493)

#define  CAM0_CLK                          (0x64A0)
#define  GPIO_090                          (0x64A1)
#define  AC_CLK25                          (0x64A2)
#define  CAM1b_CLK                         (0x64A3)

#define  CAM0_DAT0                         (0x64B0)
#define  GPIO_091                          (0x64B1)
#define  AC_CLK36                          (0x64B2)
#define  PWM12b                            (0x64B3)

#define  CAM0_DAT1                         (0x64C0)
#define  GPIO_092                          (0x64C1)
#define  AC_IN4                            (0x64C2)
#define  PWM13b                            (0x64C3)

#define  CAM0_DAT2                         (0x64D0)
#define  GPIO_093                          (0x64D1)
#define  AC_IN7                            (0x64D2)
#define  PWM14b                            (0x64D3)

#define  CAM0_DAT3                         (0x64E0)
#define  GPIO_094                          (0x64E1)
#define  AC_CLK47                          (0x64E2)
#define  PWM15b                            (0x64E3)

#define  CAM0_DAT4                         (0x64F0)
#define  GPIO_095                          (0x64F1)
#define  AC_SYNC47                         (0x64F2)

#define  CAM0_DAT5                         (0x6800)
#define  GPIO_096                          (0x6801)
#define  AC_IN3                            (0x6802)

#define  CAM0_DAT6                         (0x6810)
#define  GPIO_097                          (0x6811)
#define  AC_SYNC36                         (0x6812)

#define  CAM0_DAT7                         (0x6820)
#define  GPIO_098                          (0x6821)
#define  AC_IN6                            (0x6822)

#define  SIM_CLK                           (0x6830)
#define  GPIO_099                          (0x6831)

#define  SIM0_IO                           (0x6840)
#define  GPIO_100                          (0x6841)
#define  PWM06b                            (0x6842)

#define  SIM1_IO                           (0x6850)
#define  GPIO_101                          (0x6851)
#define  PWM07b                            (0x6852)

#define  SPI_SS                            (0x6860)
#define  GPIO_102                          (0x6861)

#define  SPI_CLK                           (0x6870)
#define  GPIO_103                          (0x6871)
#define  MLBSIGa                           (0x6872)

#define  SPI_MOSI                          (0x6880)
#define  GPIO_104                          (0x6881)
#define  MLBCLKa                           (0x6882)

#define  SPI_MISO                          (0x6890)
#define  GPIO_105                          (0x6891)
#define  MLBDATa                           (0x6892)

#define  PCM_IN0                           (0x68A0)
#define  GPIO_106                          (0x68A1)

#define  PCM_OUT0                          (0x68B0)
#define  GPIO_107                          (0x68B1)

#define  PCM_SYNC0                         (0x68C0)
#define  GPIO_108                          (0x68C1)

#define  PCM_CLK0                          (0x68D0)
#define  GPIO_109                          (0x68D1)

#define  UART2_RX                          (0x68E0)
#define  GPIO_110                          (0x68E1)

#define  UART2_TX                          (0x68F0)
#define  GPIO_111                          (0x68F1)

#define  UART2_RTS                         (0x6C00)
#define  GPIO_112                          (0x6C01)

#define  UART2_CTS                         (0x6C10)
#define  GPIO_113                          (0x6C11)

#define  UART0_RX                          (0x6C20)
#define  GPIO_114                          (0x6C21)
#define  PWM02b                            (0x6C22)
#define  SPI_CLKb                          (0x6C23)

#define  UART0_TX                          (0x6C30)
#define  GPIO_115                          (0x6C31)
#define  PWM03b                            (0x6C32)
#define  SPI_SSb                           (0x6C33)

#define  UART0_RTS                         (0x6C40)
#define  GPIO_116                          (0x6C41)
#define  SCL2b                             (0x6C42)
#define  SPI_MISOb                         (0x6C43)

#define  UART0_CTS                         (0x6C50)
#define  GPIO_117                          (0x6C51)
#define  SDA2b                             (0x6C52)
#define  SPI_MOSIb                         (0x6C53)

#define  UART1_RX                          (0x6C60)
#define  GPIO_118                          (0x6C61)

#define  UART1_TX                          (0x6C70)
#define  GPIO_119                          (0x6C71)

#define  UART1_RTS                         (0x6C80)
#define  GPIO_120                          (0x6C81)
#define  PWM04b                            (0x6C82)

#define  UART1_CTS                         (0x6C90)
#define  GPIO_121                          (0x6C91)
#define  PWM05b                            (0x6C92)

#define  UART3_RX                          (0x6CA0)
#define  GPIO_122                          (0x6CA1)
#define  SPI2_CLK                          (0x6CA2)

#define  UART3_TX                          (0x6CB0)
#define  GPIO_123                          (0x6CB1)
#define  SPI2_SS                           (0x6CB2)

#define  UART3_RTS                         (0x6CC0)
#define  GPIO_124                          (0x6CC1)
#define  SPI2_MISO                         (0x6CC2)
#define  PCM_IN1                           (0x6CC3)

#define  UART3_CTS                         (0x6CD0)
#define  GPIO_125                          (0x6CD1)
#define  SPI2_MOSI                         (0x6CD2)
#define  PCM_OUT1                          (0x6CD3)

#define  MCLK                              (0x6CE0)
#define  GPIO_126                          (0x6CE1)

#define  AC_OUT0                           (0x6CF0)
#define  GPIO_127                          (0x6CF1)

#define  AC_OUT1                           (0x7000)
#define  GPIO_128                          (0x7001)

#define  AC_IN0                            (0x7010)
#define  GPIO_129                          (0x7011)

#define  AC_IN1                            (0x7020)
#define  GPIO_130                          (0x7021)

#define  AC_IN2                            (0x7030)
#define  GPIO_131                          (0x7031)
#define  SPDIF                             (0x7032)

#define  AC_CLKM                           (0x7040)
#define  GPIO_132                          (0x7041)

#define  AC_SYNCM                          (0x7050)
#define  GPIO_133                          (0x7051)

#define  USB_CLK                           (0x7060)
#define  GPIO_134                          (0x7061)

#define  ULPI0_CLK                         (0x7070)
#define  GPIO_135                          (0x7071)

#define  ULPI0_DAT0                        (0x7080)
#define  GPIO_136                          (0x7081)

#define  ULPI0_DAT1                        (0x7090)
#define  GPIO_137                          (0x7091)

#define  ULPI0_DAT2                        (0x70A0)
#define  GPIO_138                          (0x70A1)

#define  ULPI0_DAT3                        (0x70B0)
#define  GPIO_139                          (0x70B1)

#define  ULPI0_DAT4                        (0x70C0)
#define  GPIO_140                          (0x70C1)

#define  ULPI0_DAT5                        (0x70D0)
#define  GPIO_141                          (0x70D1)

#define  ULPI0_DAT6                        (0x70E0)
#define  GPIO_142                          (0x70E1)

#define  ULPI0_DAT7                        (0x70F0)
#define  GPIO_143                          (0x70F1)

#define  ULPI0_DIR                         (0x7400)
#define  GPIO_144                          (0x7401)

#define  ULPI0_STP                         (0x7410)
#define  GPIO_145                          (0x7411)

#define  ULPI0_NXT                         (0x7420)
#define  GPIO_146                          (0x7421)

#define  ULPI1_CLK                         (0x7430)
#define  GPIO_147                          (0x7431)
#define  SD2_CLKb                          (0x7432)
#define  MC2_CLKb                          (0x7433)

#define  ULPI1_DAT0                        (0x7440)
#define  GPIO_148                          (0x7441)
#define  SD2_DAT0b                         (0x7442)
#define  MC2_DAT0b                         (0x7443)

#define  ULPI1_DAT1                        (0x7450)
#define  GPIO_149                          (0x7451)
#define  SD2_DAT1b                         (0x7452)
#define  MC2_DAT1b                         (0x7453)

#define  ULPI1_DAT2                        (0x7460)
#define  GPIO_150                          (0x7461)
#define  SD2_DAT2b                         (0x7462)
#define  MC2_DAT2b                         (0x7463)

#define  ULPI1_DAT3                        (0x7470)
#define  GPIO_151                          (0x7471)
#define  SD2_DAT3b                         (0x7472)
#define  MC2_DAT3b                         (0x7473)

#define  ULPI1_DAT4                        (0x7480)
#define  GPIO_152                          (0x7481)
#define  SD2_CMDb                          (0x7482)
#define  MC2_CMDb                          (0x7483)

#define  ULPI1_DAT5                        (0x7490)
#define  GPIO_153                          (0x7491)

#define  ULPI1_DAT6                        (0x74A0)
#define  GPIO_154                          (0x74A1)

#define  ULPI1_DAT7                        (0x74B0)
#define  GPIO_155                          (0x74B1)

#define  ULPI1_DIR                         (0x74C0)
#define  GPIO_156                          (0x74C1)
#define  MLBSIGb                           (0x74C2)

#define  ULPI1_STP                         (0x74D0)
#define  GPIO_157                          (0x74D1)
#define  MLBDATb                           (0x74D2)

#define  ULPI1_NXT                         (0x74E0)
#define  GPIO_158                          (0x74E1)
#define  MLBCLKb                           (0x74E2)

#define  nRSTOUT                           (0x74F0)
#define  GPIO_159                          (0x74F1)

#define  ND_RnB                            (0x7800)
#define  GPIO_160                          (0x7801)

#define  ND_nR                             (0x7810)
#define  GPIO_161                          (0x7811)
#define  SPI1_CLK                          (0x7813)

/* Parrot6i pins (GPIO_xxx not listed) */

#define  P6I_nTRST                             (0x5000)

/* WARNING: ids below are mapped to RTC block what why the
   offset is so big
 */
#define  P6I_RTC_32K_OUT                       (0x19006C00)
#define  P6I_GPIO_001                          (0x19006C01)
#define  P6I_RTC_WKUP_0                        (0x19006C10)
#define  P6I_GPIO_002                          (0x19006C11)
#define  P6I_RTC_WKUP_1                        (0x19006C20)
#define  P6I_GPIO_003                          (0x19006C21)
#define  P6I_PWM_00a                           (0x19006C22)
#define  P6I_SDIO_LED                          (0x19006C23)
#define  P6I_RTC_WKUP_2                        (0x19006C30)
#define  P6I_GPIO_004                          (0x19006C31)
#define  P6I_PWM_01a                           (0x19006C32)
#define  P6I_USB_PWR_ON                        (0x19006C33)
#define  P6I_RTC_OUT_0                         (0x19006C40)
#define  P6I_GPIO_005                          (0x19006C41)
#define  P6I_PWM_02a                           (0x19006C42)
#define  P6I_RTC_OUT_1                         (0x19006C50)
#define  P6I_GPIO_006                          (0x19006C51)
#define  P6I_PWM_03a                           (0x19006C52)
#define  P6I_RTC_PWR_ON                        (0x19006C60)
#define  P6I_GPIO_007                          (0x19006C61)
#define  P6I_CLK_26M_EN                        (0x19006C70)
#define  P6I_GPIO_008                          (0x19006C71)
#define  P6I_UART0_RX                          (0x19006C80)
#define  P6I_GPIO_009                          (0x19006C81)
#define  P6I_UART0_TX                          (0x19006C90)
#define  P6I_GPIO_010                          (0x19006C91)
#define  P6I_UART0_RTS                         (0x19006CA0)
#define  P6I_GPIO_011                          (0x19006CA1)
#define  P6I_UART0_CTS                         (0x19006CB0)
#define  P6I_GPIO_012                          (0x19006CB1)

#define  P6I_DDR_DQS0                          (0x50D0)
#define  P6I_DDR_DQS1                          (0x50E0)

#define  P6I_NAND_nCE                          (0x50F0)
#define  P6I_PAR_CS                            (0x50F2)

#define  P6I_NAND_RnB                          (0x5400)

#define  P6I_NAND_nR                           (0x5410)
#define  P6I_PAR_nR                            (0x5412)
#define  P6I_SPI0_CLKa                         (0x5413)

#define  P6I_NAND_nW                           (0x5420)
#define  P6I_PAR_nW                            (0x5422)
#define  P6I_SPI0_SSa                          (0x5423)

#define  P6I_NAND_AL                           (0x5430)
#define  P6I_PAR_nRS                           (0x5432)
#define  P6I_SPI0_MOSIa                        (0x5433)

#define  P6I_NAND_CL                           (0x5440)
#define  P6I_SPI0_MISOa                        (0x5443)

#define  P6I_NAND_D0                           (0x5450)
#define  P6I_PAR_DB00                          (0x5452)

#define  P6I_NAND_D1                           (0x5460)
#define  P6I_PAR_DB01                          (0x5462)

#define  P6I_NAND_D2                           (0x5470)
#define  P6I_PAR_DB02                          (0x5472)

#define  P6I_NAND_D3                           (0x5480)
#define  P6I_PAR_DB03                          (0x5482)

#define  P6I_NAND_D4                           (0x5490)
#define  P6I_PAR_DB04                          (0x5492)

#define  P6I_NAND_D5                           (0x54A0)
#define  P6I_PAR_DB05                          (0x54A2)

#define  P6I_NAND_D6                           (0x54B0)
#define  P6I_PAR_DB06                          (0x54B2)

#define  P6I_NAND_D7                           (0x54C0)
#define  P6I_PAR_DB07                          (0x54C2)


#define  P6I_SD0_CLK                           (0x54E0)
#define  P6I_SPI2_CLKb                         (0x54E2)

#define  P6I_SD0_CMD                           (0x54F0)
#define  P6I_SPI2_SSb                          (0x54F2)

#define  P6I_SD0_DAT0                          (0x5800)
#define  P6I_SPI2_MOSIb                        (0x5802)

#define  P6I_SD0_DAT1                          (0x5810)
#define  P6I_SPI2_MISOb                        (0x5812)

#define  P6I_SD0_DAT2                          (0x5820)
#define  P6I_I2S_IN3                           (0x5822)

#define  P6I_SD0_DAT3                          (0x5830)
#define  P6I_I2S_SYNC3                         (0x5832)

#define  P6I_I2C0M_SCL                         (0x5840)
#define  P6I_PWM_00b                           (0x5842)
#define  P6I_I2C0S_SCL                         (0x5843)

#define  P6I_I2C0M_SDA                         (0x5850)
#define  P6I_PWM_01b                           (0x5852)
#define  P6I_I2C0S_SDA                         (0x5853)

#define  P6I_I2C1_SCL                          (0x5860)
#define  P6I_PWM_02b                           (0x5862)
#define  P6I_I2S_IN4                           (0x5863)

#define  P6I_I2C1_SDA                          (0x5870)
#define  P6I_PWM_03b                           (0x5872)
#define  P6I_I2S_SYNC4                         (0x5873)

#define  P6I_PCM_RX                            (0x5880)

#define  P6I_PCM_TX                            (0x5890)

#define  P6I_PCM_CLK                           (0x58A0)

#define  P6I_PCM_FRM                           (0x58B0)

#define  P6I_UART1_RTSa                        (0x58C0)
#define  P6I_SPI0_CLKb                         (0x58C2)
#define  P6I_PWM_03c                           (0x58C3)

#define  P6I_UART1_CTSa                        (0x58D0)
#define  P6I_SPI0_SSb                          (0x58D2)
#define  P6I_PWM_02c                           (0x58D3)

#define  P6I_UART1_TXa                         (0x58E0)
#define  P6I_SPI0_MOSIb                        (0x58E2)

#define  P6I_UART1_RXa                         (0x58F0)
#define  P6I_SPI0_MISOb                        (0x58F2)

#define  P6I_SPI1_CLK                          (0x5C00)
#define  P6I_UART2_RTS                         (0x5C02)
#define  P6I_PWM_01c                           (0x5C03)

#define  P6I_SPI1_SS                           (0x5C10)
#define  P6I_UART2_CTS                         (0x5C12)
#define  P6I_PWM_00c                           (0x5C13)

#define  P6I_SPI1_MOSI                         (0x5C20)
#define  P6I_UART2_TX                          (0x5C22)

#define  P6I_SPI1_MISO                         (0x5C30)
#define  P6I_UART2_RX                          (0x5C32)

#define  P6I_I2S_IN1                           (0x5C40)
#define  P6I_I2S_IN2                           (0x5C50)
#define  P6I_I2S_OUT0                          (0x5C60)
#define  P6I_I2S_OUT1                          (0x5C70)
#define  P6I_I2S_CLK                           (0x5C80)
#define  P6I_I2S_SYNC                          (0x5C90)

#define  P6I_MCLK                              (0x5CA0)

#define  P6I_nRSTOUT                           (0x5CB0)

#define  P6I_RAM_D00                           (0x5CC0)
#define  P6I_RAM_D01                           (0x5CD0)
#define  P6I_RAM_D02                           (0x5CE0)
#define  P6I_RAM_D03                           (0x5CF0)
#define  P6I_RAM_D04                           (0x6000)
#define  P6I_RAM_D05                           (0x6010)
#define  P6I_RAM_D06                           (0x6020)
#define  P6I_RAM_D07                           (0x6030)
#define  P6I_RAM_D08                           (0x6040)
#define  P6I_RAM_D09                           (0x6050)
#define  P6I_RAM_D10                           (0x6060)
#define  P6I_RAM_D11                           (0x6070)
#define  P6I_RAM_D12                           (0x6080)
#define  P6I_RAM_D13                           (0x6090)
#define  P6I_RAM_D14                           (0x60A0)
#define  P6I_RAM_D15                           (0x60B0)

#define  P6I_RAM_A00                           (0x60C0)
#define  P6I_RAM_A01                           (0x60D0)
#define  P6I_RAM_A02                           (0x60E0)
#define  P6I_RAM_A03                           (0x60F0)
#define  P6I_RAM_A04                           (0x6400)
#define  P6I_RAM_A05                           (0x6410)
#define  P6I_RAM_A06                           (0x6420)
#define  P6I_RAM_A07                           (0x6430)
#define  P6I_RAM_A08                           (0x6440)
#define  P6I_RAM_A09                           (0x6450)
#define  P6I_RAM_A10                           (0x6460)
#define  P6I_RAM_A11                           (0x6470)
#define  P6I_RAM_A12                           (0x6480)
#define  P6I_RAM_A13                           (0x6490)

#define  P6I_RAM_BA0                           (0x64A0)
#define  P6I_RAM_BA1                           (0x64B0)
#define  P6I_JTAG                              (0x64F0)
#define  P6I_SPI2a                             (0x64F2)
#define  P6I_UART1b                            (0x64F3)

#endif /* __ARCH_ARM_PARROT6_PINS_H */
