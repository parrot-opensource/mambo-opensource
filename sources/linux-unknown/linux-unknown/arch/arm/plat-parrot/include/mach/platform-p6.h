/**
 * @file linux/include/asm-arm/arch-parrot/platform-p6.h
 * @brief Parrot6 device mapping and registers
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     matthieu.castet@parrot.com
 * @date       2008-06-18
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
 *
 */
#ifndef __ARCH_ARM_PLATFORM_P6_H
#define __ARCH_ARM_PLATFORM_P6_H

#define PARROT6_PCLK_HS               (156000000) /* normal mode = 156 MHz */
#define PARROT6_PCLK_LS               (130000000) /* slow mode = 130 MHz */

#define PARROT6_CAN_CLK               (40000000)  /* CAN clock = 40 MHz */

/* P6i clocks */
#define PARROT6I_CLKIN                (26000000)  /* default CLKIN = 26 MHz */

/* memory mapping */

#define PARROT6_DDR_BASE              0x40000000
#define PARROT6_DDR_SIZE              0x40000000

#define PARROT6_INTRAM_BASE           0x80000000  /* 64 kB internal RAM */
#define PARROT6_INTRAM_SIZE           0x00010000

#define PARROT6_INTROM_BASE           0x90000000  /* 64 kB internal ROM */
#define PARROT6_INTROM_SIZE           0x00010000

/* AHB peripherals */

#define PARROT6_VIC                   0xc0000000 /* Interrupt controller */
#define PARROT6_MPMC                  0xc0100000 /* MPMC memory controller */
#define PARROT6_PARINT                0xc0200000 /* PARINT */
#define PARROT6_NAND_DATA             0xc0300000 /* NAND DATA */
#define PARROT6_USB0                  0xc0400000 /* USB0 */
#define PARROT6_USB1                  0xc0500000 /* USB1 */
#define PARROT6_DMAC                  0xc0600000 /* DMA controller */
#define PARROT6_SDIO                  0xc0700000 /* SDIO controller */

/* APB peripherals (note, 64 kB memory range) */

#define PARROT6_SYS                   0xd0000000 /* System controller */
#define PARROT6_AAI                   0xd0010000 /* Advanced Audio interface */
#define PARROT6_MOST                  0xd0020000 /* MediaLB int. for MOST */
#define PARROT6_CV                    0xd0030000 /* CV voice reco accel. */
#define PARROT6_CAMIF0                0xd0040000 /* Camera interface */
#define PARROT6_CAMIF1                0xd0050000 /* Camera interface */
#define PARROT6_H264                  0xd0060000 /* MMC Interface */
#define PARROT6_UART0                 0xd0070000 /* Uart 0 */
#define PARROT6_UART1                 0xd0080000 /* Uart 1 */
#define PARROT6_UART2                 0xd0090000 /* Uart 2 */
#define PARROT6_UART3                 0xd00a0000 /* Uart 3 */
#define PARROT6_SPI0                  0xd00b0000 /* SPI interface 0 */
#define PARROT6_SPI1                  0xd00c0000 /* SPI interface 1 */
#define PARROT6_SPI2                  0xd00d0000 /* SPI interface 2 */
#define PARROT6_LCDC                  0xd00e0000 /* LCD interface */
#define PARROT6_NAND                  0xd00f0000 /* NAND interface */
#define PARROT6_SIM                   0xd0100000 /* SIM Card interface */
#define PARROT6_I2CS                  0xd0110000 /* I2C slave */
#define PARROT6_PWM                   0xd0120000 /* PWM */
#define PARROT6_GPIO                  0xd0130000 /* GPIOs */
#define PARROT6_AHBMON                0xd0140000 /* AHB monitor */
#define PARROT6_I2CM0                 0xd0150000 /* I2C master 0 */
#define PARROT6_I2CM1                 0xd0160000 /* I2C master 1 */
#define PARROT6_MPSD0                 0xd0170000 /* MMC Interface 0 */
#define PARROT6_MPSD1                 0xd0180000 /* MMC Interface 1 */
#define PARROT6I_RTC                  0xd0190000 /* RTC (P6i only) */

#define PARROT6_CAN                   0xf0000000 /* C_CAN module */

#endif /* __ARCH_ARM_PLATFORM_P6_H */
