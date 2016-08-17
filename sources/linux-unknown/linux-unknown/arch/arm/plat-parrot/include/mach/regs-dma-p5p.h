/**
 * @file linux/include/asm-arm/arch-parrot/regs-p5p-dma.h
 * @brief Parrot5+ DMA (ARM PL081) controller registers definitions
 *
 * Copyright (C) 2007 Parrot S.A.
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
#ifndef __ASM_ARCH_P5P_DMA_REGS_H
#define __ASM_ARCH_P5P_DMA_REGS_H

#define _P5P_DMAC_NB_CHANNELS              (2)

#define _P5P_DMAC_INTSTATUS                (0x00)
#define _P5P_DMAC_INTTCSTATUS              (0x04)
#define _P5P_DMAC_INTTCCLEAR               (0x08)
#define _P5P_DMAC_INTERRORSTATUS           (0x0c)
#define _P5P_DMAC_INTERRCLR                (0x10)
#define _P5P_DMAC_RAWINTTCSTATUS           (0x14)
#define _P5P_DMAC_RAWINTERRORSTATUS        (0x18)
#define _P5P_DMAC_ENBLDCHNS                (0x1c)
#define _P5P_DMAC_SOFTBREQ                 (0x20)
#define _P5P_DMAC_SOFTSREQ                 (0x24)
#define _P5P_DMAC_SOFTLBREQ                (0x28)
#define _P5P_DMAC_SOFTLSREQ                (0x2c)
#define _P5P_DMAC_CONFIG                   (0x30)
#define _P5P_DMAC_SYNC                     (0x34)

#define _P5P_DMAC_C0OFF                    (0x100)
#define _P5P_DMAC_C1OFF                    (0x120)

#define _P5P_DMAC_CXSRCADDR                (0x00)
#define _P5P_DMAC_CXDESTADDR               (0x04)
#define _P5P_DMAC_CXLLI                    (0x08)
#define _P5P_DMAC_CXCTRL                   (0x0c)
#define _P5P_DMAC_CXCONFIG                 (0x10)

#define _P5P_DMAC_C0SRCADDR                (0x100)
#define _P5P_DMAC_C0DESTADDR               (0x104)
#define _P5P_DMAC_C0LLI                    (0x108)
#define _P5P_DMAC_C0CTRL                   (0x10c)
#define _P5P_DMAC_C0CONFIG                 (0x110)

#define _P5P_DMAC_C1SRCADDR                (0x120)
#define _P5P_DMAC_C1DESTADDR               (0x124)
#define _P5P_DMAC_C1LLI                    (0x128)
#define _P5P_DMAC_C1CTRL                   (0x12c)
#define _P5P_DMAC_C1CONFIG                 (0x130)

/* Registers bitwise definitions */

#define _P5P_DMAC_CONFIG_ENABLE            (1 << 0)

/* bit offsets and masks for DMA Channel Control Register (_P5P_DMAC_CXCTRL) */
#define _P5P_DMAC_CXCTRL_SI                (1 << 26)
#define _P5P_DMAC_CXCTRL_DI                (1 << 27)
#define _P5P_DMAC_CXCTRL_I                 (1 << 31)

#define bs_P5P_DMAC_CXCTRL_TRANSIZE        ( 0)
#define bw_P5P_DMAC_CXCTRL_TRANSIZE        (12)
#define bs_P5P_DMAC_CXCTRL_SBSIZE          (12)
#define bw_P5P_DMAC_CXCTRL_SBSIZE          ( 3)
#define bs_P5P_DMAC_CXCTRL_DBSIZE          (15)
#define bw_P5P_DMAC_CXCTRL_DBSIZE          ( 3)
#define bs_P5P_DMAC_CXCTRL_SWIDTH          (18)
#define bw_P5P_DMAC_CXCTRL_SWIDTH          ( 3)
#define bs_P5P_DMAC_CXCTRL_DWIDTH          (21)
#define bw_P5P_DMAC_CXCTRL_DWIDTH          ( 3)
#define bs_P5P_DMAC_CXCTRL_PROT            (28)
#define bw_P5P_DMAC_CXCTRL_PROT            ( 3)


/* offsets and masks for DMA Channel Control Register (_P5P_DMAC_CXCONFIG) */
#define _P5P_DMAC_CXCONFIG_ENABLE          (1 << 0)
#define _P5P_DMAC_CXCONFIG_ITERROR         (1 << 14)
#define _P5P_DMAC_CXCONFIG_ITTC            (1 << 15)
#define _P5P_DMAC_CXCONFIG_LOCK            (1 << 16)
#define _P5P_DMAC_CXCONFIG_ACTIVE          (1 << 17)
#define _P5P_DMAC_CXCONFIG_HALT            (1 << 18)

#define bs_P5P_DMAC_CXCONFIG_SRCPERIPH     ( 1)
#define bw_P5P_DMAC_CXCONFIG_SRCPERIPH     ( 4)
#define bs_P5P_DMAC_CXCONFIG_DSTPERIPH     ( 6)
#define bw_P5P_DMAC_CXCONFIG_DSTPERIPH     ( 4)
#define bs_P5P_DMAC_CXCONFIG_FLOWCNTRL     (11)
#define bw_P5P_DMAC_CXCONFIG_FLOWCNTRL     ( 3)

/* value for _P5P_DMAC_CXCONFIG_FLOWCNTRL */
#define _P5P_DMAC_CXCONFIG_FLOWCNTRL_M2M_DMAC   (0) /* mem->mem, ctrl=dma */
#define _P5P_DMAC_CXCONFIG_FLOWCNTRL_M2P_DMAC   (1) /* mem->periph, ctrl=dma */
#define _P5P_DMAC_CXCONFIG_FLOWCNTRL_P2M_DMAC   (2) /* periph->mem, ctrl=dma */
#define _P5P_DMAC_CXCONFIG_FLOWCNTRL_P2P_DMAC   (3) /* src->dst, ctrl=dma */
#define _P5P_DMAC_CXCONFIG_FLOWCNTRL_P2P_DEST   (4) /* src->dst, ctrl=dst per.*/
#define _P5P_DMAC_CXCONFIG_FLOWCNTRL_M2P_PERIPH (5) /* mem->periph, ctrl=per. */
#define _P5P_DMAC_CXCONFIG_FLOWCNTRL_P2M_PERIPH (6) /* periph->mem, ctrl=per. */
#define _P5P_DMAC_CXCONFIG_FLOWCNTRL_P2P_SRC    (7) /* src->dst, ctrl=src per.*/

#endif /* __ASM_ARCH_P5P_DMA_REGS_H */
