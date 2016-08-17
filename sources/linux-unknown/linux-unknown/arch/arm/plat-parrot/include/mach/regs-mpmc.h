/**
 * @file linux/include/asm-arm/arch-parrot/regs-mpmc.h
 * @brief Parrot5/5+ MPMC registers
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     yves.lemoine@parrot.com
 * @date       2005-12-05
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
#ifndef __ARCH_ARM_PARROT_MPMC_H
#define __ARCH_ARM_PARROT_MPMC_H

/* PL172 Memory controller registers definitions */

#define _MPMC_CTRL                      0x00
#define _MPMC_STATUS                    0x04
#define _MPMC_CONFIG                    0x08

#define _MPMC_Control                   0x000
#define _MPMC_Status                    0x004
#define _MPMC_Config                    0x008
#define _MPMC_DynamicControl            0x020
#define _MPMC_DynamicRefresh            0x024
#define _MPMC_DynamicReadConfig         0x028
#define _MPMC_DynamictRP                0x030
#define _MPMC_DynamictRAS               0x034
#define _MPMC_DynamictSREX              0x038
#define _MPMC_DynamictWR                0x044
#define _MPMC_DynamictRC                0x048
#define _MPMC_DynamictRFC               0x04c
#define _MPMC_DynamictXSR               0x050
#define _MPMC_DynamictRRD               0x054
#define _MPMC_DynamictMRD               0x058
#define _MPMC_StaticExtendedWait        0x080
#define _MPMC_DynamicConfig0            0x100
#define _MPMC_DynamicRasCas0            0x104
#define _MPMC_DynamicConfig1            0x120
#define _MPMC_DynamicRasCas1            0x124
#define _MPMC_DynamicConfig2            0x140
#define _MPMC_DynamicRasCas2            0x144
#define _MPMC_DynamicConfig3            0x160
#define _MPMC_DynamicRasCas3            0x164

#define _MPMC_StaticConfig0             0x200
#define _MPMC_StaticWaitWen0            0x204
#define _MPMC_StaticWaitOen0            0x208
#define _MPMC_StaticWaitRd0             0x20C
#define _MPMC_StaticWaitPage0           0x210
#define _MPMC_StaticWaitWr0             0x214
#define _MPMC_StaticWaitTurn0           0x218

/*
 * Parrot5+ NAND wait states
 * [1:0] : 0=6 cycles max Wr, 1=5 cycles, 2=4 cycles, 3=3 cycles
 *   [2] : 0=2 cycles min Rd, 1=3 cycles min Rd
 */
#define _MPMC_StaticWaitNand0           0x21C

#define _MPMC_StaticConfig1             0x220
#define _MPMC_StaticWaitWen1            0x224
#define _MPMC_StaticWaitOen1            0x228
#define _MPMC_StaticWaitRd1             0x22C
#define _MPMC_StaticWaitPage1           0x230
#define _MPMC_StaticWaitWr1             0x234
#define _MPMC_StaticWaitTurn1           0x238

#define _MPMC_StaticConfig2             0x240
#define _MPMC_StaticWaitWen2            0x244
#define _MPMC_StaticWaitOen2            0x248
#define _MPMC_StaticWaitRd2             0x24C
#define _MPMC_StaticWaitPage2           0x250
#define _MPMC_StaticWaitWr2             0x254
#define _MPMC_StaticWaitTurn2           0x258

#define _MPMC_StaticConfig3             0x260
#define _MPMC_StaticWaitWen3            0x264
#define _MPMC_StaticWaitOen3            0x268
#define _MPMC_StaticWaitRd3             0x26C
#define _MPMC_StaticWaitPage3           0x270
#define _MPMC_StaticWaitWr3             0x274
#define _MPMC_StaticWaitTurn3           0x278

/* Registers bitwise definitions */

/* StaticConfig Register */
#define _StaticConfig_8bit              0
#define _StaticConfig_16bit             (1 << 0)
#define _StaticConfig_32bit             (1 << 1)
#define _StaticConfig_PageMode          (1 << 3)
#define _StaticConfig_CSPolarity        (1 << 6)
#define _StaticConfig_ByteLane          (1 << 7)
#define _StaticConfig_ExtendedWait      (1 << 8)
#define _StaticConfig_BufferEnable      (1 << 19)
#define _StaticConfig_WriteProtect      (1 << 20)

#endif /* __ARCH_ARM_PARROT_MPMC_H */
