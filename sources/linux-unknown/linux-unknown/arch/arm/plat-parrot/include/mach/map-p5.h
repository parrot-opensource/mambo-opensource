/**
 * @file linux/include/asm-arm/arch-parrot/map-p5.h
 * @brief Parrot5/5+ device virtual mapping
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2007-06-11
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
#ifndef __ARCH_ARM_PARROT_MAP_P5_H
#define __ARCH_ARM_PARROT_MAP_P5_H

#include <mach/platform-p5.h>

#define PARROT5_VA_VIC    P5_IOVA(PARROT5_VIC)   /* Interrupt controller */
#define PARROT5_VA_MPMC   P5_IOVA(PARROT5_MPMC)  /* MPMC memory controller */
#define PARROT5_VA_USB0   P5_IOVA(PARROT5_USB0)  /* USB0 (P5+ only) */
#define PARROT5_VA_USB1   P5_IOVA(PARROT5_USB1)  /* USB1 (P5+ only) */
#define PARROT5_VA_DMAC   P5_IOVA(PARROT5_DMAC)  /* DMA controller (P5+) */

/* PARROT5 APB HS Mapping */
#define PARROT5_VA_SYS    P5_IOVA(PARROT5_SYS)   /* System controller */
#define PARROT5_VA_AAI    P5_IOVA(PARROT5_AAI)   /* Advanced Audio interface */
#define PARROT5_VA_MOST   P5_IOVA(PARROT5_MOST)  /* MediaLB interface */
#define PARROT5_VA_CV     P5_IOVA(PARROT5_CV)    /* CV voice reco accel. */
#define PARROT5_VA_CAMIF  P5_IOVA(PARROT5_CAMIF) /* Camera interface */
#define PARROT5_VA_ME     P5_IOVA(PARROT5_ME)    /* Motion Estimation Accel. */
#define PARROT5_VA_MPSD0  P5_IOVA(PARROT5_MPSD0) /* MMC Interface */
#define PARROT5_VA_UART0  P5_IOVA(PARROT5_UART0) /* Uart 0 */
#define PARROT5_VA_UART1  P5_IOVA(PARROT5_UART1) /* Uart 1 */
#define PARROT5_VA_UART2  P5_IOVA(PARROT5_UART2) /* Uart 2 */
#define PARROT5_VA_SPI    P5_IOVA(PARROT5_SPI)   /* SPI interface */

/* Parrot5+ additional peripherals */
#define PARROT5_VA_DCT    P5_IOVA(PARROT5_DCT)   /* DCT */
#define PARROT5_VA_MPSD1  P5_IOVA(PARROT5_MPSD1) /* MMC Interface */
#define PARROT5_VA_MPSD   P5_IOVA(PARROT5_MPSD)

/* PARROT5 APB LS Mapping */
#define PARROT5_VA_LPO    P5_IOVA(PARROT5_LPO)   /* LPO */
#define PARROT5_VA_CAN    P5_IOVA(PARROT5_CAN)   /* C_CAN module */
#define PARROT5_VA_SIM    P5_IOVA(PARROT5_SIM)   /* SIM Card interface */
#define PARROT5_VA_I2CS   P5_IOVA(PARROT5_I2CS)  /* I2C slave */
#define PARROT5_VA_PWM    P5_IOVA(PARROT5_PWM)   /* PWM */
#define PARROT5_VA_GPIO   P5_IOVA(PARROT5_GPIO)  /* GPIOs */
#define PARROT5_VA_AHBMON P5_IOVA(PARROT5_AHBMON)/* AHB monitor */
#define PARROT5_VA_I2CM   P5_IOVA(PARROT5_I2CM)  /* I2C master */

#endif /* __ARCH_ARM_PARROT_MAP_P5_H */
