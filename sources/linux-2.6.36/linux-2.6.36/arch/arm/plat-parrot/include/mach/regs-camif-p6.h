/**
 * @file linux/arc/arm/plat-parrot/include/mach/regs-camif-p6.h
 * @brief Device I/O - Description of Parrot6 camera hardware
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     Farbod Ghassemi, Virginia Martin Heriz
 * @author     matthieu.castet@parrot.com
 * @date       2008-12-12
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

#ifndef __MACH_REGS_CAMIF_P6_H
#define __MACH_REGS_CAMIF_P6_H 1


/* Parrot proprietary P6_CAMIF registers */

/* register mapping */
#define _P6_CAMIF_STATUS         0x0000   /* CAMIF Status register */
#define _P6_CAMIF_ITEN           0x0004   /* CAMIF interrupt enable register */
#define _P6_CAMIF_ITACK          0x0008   /* CAMIF interrupt acknowledge register  */
#define _P6_CAMIF_CTRL           0x000C   /* CAMIF control register */
#define _P6_CAMIF_DMA            0x0010   /* CAMIF DMA register */
#define _P6_CAMIF_YBASE1ADDR     0x0014   /* CAMIF Y base 1 address register */
#define _P6_CAMIF_YBASE2ADDR     0x0018   /* CAMIF Y base 2 address register */
#define _P6_CAMIF_YBASE3ADDR     0x001C   /* CAMIF Y base 3 address register */
#define _P6_CAMIF_CBBASE1ADDR    0x0020   /* CAMIF CB base 1 address register */
#define _P6_CAMIF_CBBASE2ADDR    0x0024   /* CAMIF CB base 2 address register */
#define _P6_CAMIF_CBBASE3ADDR    0x0028   /* CAMIF CB base 3 address register */
#define _P6_CAMIF_CRBASE1ADDR    0x002C   /* CAMIF CR base 1 address register */
#define _P6_CAMIF_CRBASE2ADDR    0x0030   /* CAMIF CR base 2 address register */
#define _P6_CAMIF_CRBASE3ADDR    0x0034   /* CAMIF CR base 3 address register */
#define _P6_CAMIF_YCURRENTADDR   0x0038   /* CAMIF Y current address register */
#define _P6_CAMIF_CBCURRENTADDR  0x003C   /* CAMIF CB current address register */
#define _P6_CAMIF_CRCURRENTADDR  0x0040   /* CAMIF CR current address register */

/* Bitwise definitions */
#define P6_CAMIF_STATUS_LINE               (1 << 0)
#define P6_CAMIF_STATUS_FIFOOVER           (1 << 1)
#define P6_CAMIF_STATUS_ERROR              (1 << 2)

#define P6_CAMIF_ITEN_LINE                 (1 << 0)
#define P6_CAMIF_ITEN_OVERWRITE            (1 << 1)
#define P6_CAMIF_ITEN_ERROR                (1 << 2)

#define P6_CAMIF_ITACK_LINE                (1 << 0)
#define P6_CAMIF_ITACK_OVERWRITE           (1 << 1)
#define P6_CAMIF_ITACK_ERROR               (1 << 2)

#define P6_CAMIF_CTRL_P6_CAMIF_EN             (1 << 0)
#define P6_CAMIF_CTRL_SYNC_NEG             (1 << 1)
#define P6_CAMIF_CTRL_ORDER                (1 << 2)
#define P6_CAMIF_CTRL_HSYNCPOL             (1 << 3)
#define P6_CAMIF_CTRL_VSYNCPOL             (1 << 4)
#define P6_CAMIF_CTRL_SYNCSRC_BT656        (1 << 5)
#define P6_CAMIF_CTRL_FORMAT               (1 << 6)
#define P6_CAMIF_CTRL_INTLINE              (7 << 7)
#define P6_CAMIF_CTRL_NB_BUFFERS          (1 << 16)

#define P6_CAMIF_DMA_INCR                   0
#define P6_CAMIF_DMA_INCR4                  1
#define P6_CAMIF_DMA_INCR8                  2
#define P6_CAMIF_DMA_INCR16                 3
#define P6_CAMIF_DMA_IDLE                  (1 << 2)
#define P6_CAMIF_DMA_OFFSET                (0xf << 3)


/* P6_CAMIF register size masks */

#define P6_CAMIF_MSK_STATUS             0x00000007
#define P6_CAMIF_MSK_ITEN               0x00000007
#define P6_CAMIF_MSK_ITACK              0x00000007
#define P6_CAMIF_MSK_CTRL               0x000003ff
#define P6_CAMIF_MSK_DMA                0x000007ff
#define P6_CAMIF_MSK_YBASE1ADDR         0xfffffffc
#define P6_CAMIF_MSK_YBASE2ADDR         0xfffffffc
#define P6_CAMIF_MSK_YBASE3ADDR         0xfffffffc
#define P6_CAMIF_MSK_CBBASE1ADDR        0xfffffffc
#define P6_CAMIF_MSK_CBBASE2ADDR        0xfffffffc
#define P6_CAMIF_MSK_CBBASE3ADDR        0xfffffffc
#define P6_CAMIF_MSK_CRBASE1ADDR        0xfffffffc
#define P6_CAMIF_MSK_CRBASE2ADDR        0xfffffffc
#define P6_CAMIF_MSK_CRBASE3ADDR        0xfffffffc
#define P6_CAMIF_MSK_YCURRENTADDR       0x000ffffc
#define P6_CAMIF_MSK_CBCURRENTADDR      0x00007ffc
#define P6_CAMIF_MSK_CRCURRENTADDR      0x00007ffc

#endif
