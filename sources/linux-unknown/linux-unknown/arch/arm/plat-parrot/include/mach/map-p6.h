/**
 * @file linux/include/asm-arm/arch-parrot/map-p6.h
 * @brief Parrot6 device virtual mapping
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     matthieu.castet@parrot.com
 * @date       2008-06-11
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
#ifndef __ARCH_ARM_PARROT_MAP_P6_H
#define __ARCH_ARM_PARROT_MAP_P6_H

#include <mach/platform-p6.h>

/* AHB mapping */
#define PARROT6_VA_VIC       P5_IOVA(PARROT6_VIC)       /* Int controller */
#define PARROT6_VA_MPMC      P5_IOVA(PARROT6_MPMC)      /* Memory controller */
#define PARROT6_VA_PARINT    P5_IOVA(PARROT6_PARINT)    /* Parallel interface */
#define PARROT6_VA_NAND_DATA P5_IOVA(PARROT6_NAND_DATA) /* NAND data memory */
#define PARROT6_VA_DMAC      P5_IOVA(PARROT6_DMAC)      /* DMA controller */
#define PARROT6_VA_USB0      P5_IOVA(PARROT6_USB0)      /* USB0 controller */
#define PARROT6_VA_USB1      P5_IOVA(PARROT6_USB1)      /* USB1 controller */

/* APB mapping */
#define PARROT6_VA_SYS       P5_IOVA(PARROT6_SYS)       /* System controller */
#define PARROT6_VA_UART0     P5_IOVA(PARROT6_UART0)     /* Uart 0 */
#define PARROT6_VA_UART1     P5_IOVA(PARROT6_UART1)     /* Uart 1 */
#define PARROT6_VA_UART2     P5_IOVA(PARROT6_UART2)     /* Uart 2 */
#define PARROT6_VA_UART3     P5_IOVA(PARROT6_UART3)     /* Uart 3 */
#define PARROT6_VA_NAND      P5_IOVA(PARROT6_NAND)      /* NAND controller */
#define PARROT6_VA_GPIO      P5_IOVA(PARROT6_GPIO)      /* GPIO controller */
#define PARROT6I_VA_RTC      P5_IOVA(PARROT6I_RTC)      /* P6i RTC */
#define PARROT6_VA_H264      P5_IOVA(PARROT6_H264)      /* H264 controller */

#endif /* __ARCH_ARM_PARROT_MAP_P6_H */
