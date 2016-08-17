/**
 * @file linux/include/asm-arm/arch-parrot/regs-pl08x.h
 * @brief ARM PL08x DMA controller registers definitions
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
#ifndef __ASM_ARCH_DMA_PL08X_REGS_H
#define __ASM_ARCH_DMA_PL08X_REGS_H

#define _PL080_DMA_PERIPH_MAX                (16)
#define _PL080_MAX_CHANNELS                  (8)

#define _PL080_INTSTATUS                     (0x00)
#define _PL080_INTTCSTATUS                   (0x04)
#define _PL080_INTTCCLEAR                    (0x08)
#define _PL080_INTERRORSTATUS                (0x0c)
#define _PL080_INTERRCLR                     (0x10)
#define _PL080_RAWINTTCSTATUS                (0x14)
#define _PL080_RAWINTERRORSTATUS             (0x18)
#define _PL080_ENBLDCHNS                     (0x1c)
#define _PL080_SOFTBREQ                      (0x20)
#define _PL080_SOFTSREQ                      (0x24)
#define _PL080_SOFTLBREQ                     (0x28)
#define _PL080_SOFTLSREQ                     (0x2c)
#define _PL080_CONFIG                        (0x30)
#define _PL080_SYNC                          (0x34)

#define _PL080_PERIPHID0                     (0xfe0)
#define _PL080_PERIPHID1                     (0xfe4)
#define _PL080_PERIPHID2                     (0xfe8)
#define _PL080_PERIPHID3                     (0xfec)
#define _PL080_PCELLID0                      (0xff0)
#define _PL080_PCELLID1                      (0xff4)
#define _PL080_PCELLID2                      (0xff8)
#define _PL080_PCELLID3                      (0xffc)

#define _PL080_C0OFF                         (0x100)
#define _PL080_CXRANGE                       (0x20)

#define _PL080_CXSRCADDR                     (0x00)
#define _PL080_CXDESTADDR                    (0x04)
#define _PL080_CXLLI                         (0x08)
#define _PL080_CXCTRL                        (0x0c)
#define _PL080_CXCONFIG                      (0x10)

/* Registers bitwise definitions */

#define _PL080_CONFIG_ENABLE                 (1 << 0)

/* bit offsets and masks for DMA Channel Control Register (_PL080_CXCTRL) */
#define _PL080_CXCTRL_SI                     (1 << 26)
#define _PL080_CXCTRL_DI                     (1 << 27)
#define _PL080_CXCTRL_I                      (1 << 31)

#define bs_PL080_CXCTRL_TRANSIZE             ( 0)
#define bw_PL080_CXCTRL_TRANSIZE             (12)
#define bs_PL080_CXCTRL_SBSIZE               (12)
#define bw_PL080_CXCTRL_SBSIZE               ( 3)
#define bs_PL080_CXCTRL_DBSIZE               (15)
#define bw_PL080_CXCTRL_DBSIZE               ( 3)
#define bs_PL080_CXCTRL_SWIDTH               (18)
#define bw_PL080_CXCTRL_SWIDTH               ( 3)
#define bs_PL080_CXCTRL_DWIDTH               (21)
#define bw_PL080_CXCTRL_DWIDTH               ( 3)
#define bs_PL080_CXCTRL_PROT                 (28)
#define bw_PL080_CXCTRL_PROT                 ( 3)

/* offsets and masks for DMA Channel Control Register (_PL080_CXCONFIG) */
#define _PL080_CXCONFIG_ENABLE               (1 << 0)
#define _PL080_CXCONFIG_ITERROR              (1 << 14)
#define _PL080_CXCONFIG_ITTC                 (1 << 15)
#define _PL080_CXCONFIG_LOCK                 (1 << 16)
#define _PL080_CXCONFIG_ACTIVE               (1 << 17)
#define _PL080_CXCONFIG_HALT                 (1 << 18)

#define bs_PL080_CXCONFIG_SRCPERIPH          ( 1)
#define bw_PL080_CXCONFIG_SRCPERIPH          ( 4)
#define bs_PL080_CXCONFIG_DSTPERIPH          ( 6)
#define bw_PL080_CXCONFIG_DSTPERIPH          ( 4)
#define bs_PL080_CXCONFIG_FLOWCNTRL          (11)
#define bw_PL080_CXCONFIG_FLOWCNTRL          ( 3)

/* value for _PL080_CXCONFIG_FLOWCNTRL */
#define _PL080_CXCONFIG_FLOWCNTRL_M2M_DMAC   (0) /* mem->mem, ctrl=dma */
#define _PL080_CXCONFIG_FLOWCNTRL_M2P_DMAC   (1) /* mem->periph, ctrl=dma */
#define _PL080_CXCONFIG_FLOWCNTRL_P2M_DMAC   (2) /* periph->mem, ctrl=dma */
#define _PL080_CXCONFIG_FLOWCNTRL_P2P_DMAC   (3) /* src->dst, ctrl=dma */
#define _PL080_CXCONFIG_FLOWCNTRL_P2P_DEST   (4) /* src->dst, ctrl=dst per.*/
#define _PL080_CXCONFIG_FLOWCNTRL_M2P_PERIPH (5) /* mem->periph, ctrl=per. */
#define _PL080_CXCONFIG_FLOWCNTRL_P2M_PERIPH (6) /* periph->mem, ctrl=per. */
#define _PL080_CXCONFIG_FLOWCNTRL_P2P_SRC    (7) /* src->dst, ctrl=src per.*/

#define bs_PL080_PERIPHID_PARTNUMBER         (0)
#define bw_PL080_PERIPHID_PARTNUMBER         (12)
#define bs_PL080_PERIPHID_DESIGNERID         (12)
#define bw_PL080_PERIPHID_DESIGNERID         (8)
#define bs_PL080_PERIPHID_REVISION           (20)
#define bw_PL080_PERIPHID_REVISION           (4)
#define bs_PL080_PERIPHID_CONFIGURATION      (24)
#define bw_PL080_PERIPHID_CONFIGURATION      (8)

#define _PL080_PERIPHID_PARTNUMBER_PL080     (0x080)
#define _PL080_PERIPHID_PARTNUMBER_PL081     (0x081)


#endif /* __ASM_ARCH_DMA_PL08X_REGS_H */
