/**
 * @file linux/include/asm-arm/arch-parrot/p5-platform.h
 * @brief Parrot5/5+ device mapping and registers
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     yves.lemoine@parrot.com
 * @author     ivan.djelic@parrot.com
 * @date       2005-09-28
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
#ifndef __ARCH_ARM_P5_PLATFORM_H
#define __ARCH_ARM_P5_PLATFORM_H

/* Parrot5 AHB high-speed clock */
#define PARROT5_PCLK_HS               (104000000) /* normal mode */
#define PARROT5_PCLK_HS_SLOW          ( 78000000) /* slow mode (cpu 156 MHz) */
#define PARROT5_PCLK_HS_OVERCLOCK     (130000000) /* overclock (cpu 260 MHz) */

#define PARROT5_PCLK_LS               (PARROT5_PCLK_HS/2)

#define PARROT5_UART_CLOCK            PARROT5_PCLK_HS
#define PARROT5_SYS_CLOCK             PARROT5_PCLK_HS
#define PARROT5_SPI_CLOCK             PARROT5_PCLK_HS
#define PARROT5_CAN_CLOCK             PARROT5_PCLK_LS

#define PARROT5_INTROM_BASE           0x03000000 /* 32 kB internal ROM */
#define PARROT5_INTROM_SIZE           0x00008000

#define PARROT5_INTRAM_BASE           0x02000000 /* 96 kB internal RAM */
#define PARROT5_INTRAM_SIZE           0x00018000

#define PARROT5_CS0_PHYS_BASE         0x10000000 /* CS0 : SDRAM */
#define PARROT5_CS1_PHYS_BASE         0x20000000 /* CS1 : FLASH */
#define PARROT5_CS2_PHYS_BASE         0x30000000 /* CS2 : SRAM/FLASH */
#define PARROT5_CS3_PHYS_BASE         0x40000000 /* CS3 : SRAM/FLASH */
#define PARROT5_CS4_PHYS_BASE         0x50000000 /* CS4 : virtual CS - GPI */

#define PARROT5_VIC                   0xc0000000 /* Interrupt controller */
#define PARROT5_MPMC                  0xc0010000 /* MPMC memory controller */

#define PARROT5_USB0                  0xc0030000 /* USB0 (P5+ only) */
#define PARROT5_USB1                  0xc0040000 /* USB1 (P5+ only) */
#define PARROT5_DMAC                  0xc0050000 /* DMA controller (P5+) */

/* PARROT5 APB HS Mapping */

#define PARROT5_SYS                   0xd0000000 /* System controller */
#define PARROT5_AAI                   0xd0010000 /* Advanced Audio interface */
#define PARROT5_MOST                  0xd0020000 /* MediaLB int. for MOST */
#define PARROT5_CV                    0xd0030000 /* CV voice reco accel. */
#define PARROT5_CAMIF                 0xd0040000 /* Camera interface */
#define PARROT5_ME                    0xd0050000 /* Motion Estimation Accel. */
#define PARROT5_MPSD0                 0xd0060000 /* MMC Interface */
#define PARROT5_UART0                 0xd0070000 /* Uart 0 */
#define PARROT5_UART1                 0xd0080000 /* Uart 1 */
#define PARROT5_UART2                 0xd0090000 /* Uart 2 */
#define PARROT5_SPI                   0xd00a0000 /* SPI interface */

/* Parrot5+ additional peripherals */
#define PARROT5_DCT                   0xd00b0000 /* DCT */
#define PARROT5_MPSD1                 0xd00c0000 /* MMC Interface */

#define PARROT5_MPSD                  PARROT5_MPSD0

/* PARROT5 APB LS Mapping */

#define PARROT5_LPO                   0xe0000000 /* LPO */
#define PARROT5_CAN                   0xe0010000 /* C_CAN module */
#define PARROT5_SIM                   0xe0020000 /* SIM Card interface */
#define PARROT5_I2CS                  0xe0030000 /* I2C slave */
#define PARROT5_PWM                   0xe0040000 /* PWM */
#define PARROT5_GPIO                  0xe0050000 /* GPIOs */
#define PARROT5_AHBMON                0xe0060000 /* AHB monitor */
#define PARROT5_I2CM                  0xe0070000 /* I2C master */

#endif /* __ARCH_ARM_P5_PLATFORM_H */
