/**
 *
 * Copyright (C) 2009 Parrot S.A.
 *
 * @author     matthieu.castet@parrot.com
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
#ifndef __ARCH_ARM_PARROT6_PWM_H
#define __ARCH_ARM_PARROT6_PWM_H


#define P6_PWM_CTL           0x0000       // P6_PWM Control register

#define P6_PWM_SPEED00       0x0004       // P6_PWM Speed00 register
#define P6_PWM_SPEED01       0x0008       // P6_PWM Spedd01 register
#define P6_PWM_SPEED02       0x000C       // P6_PWM Speed02 register
#define P6_PWM_SPEED03       0x0010       // P6_PWM Speed03 register
#define P6_PWM_SPEED04       0x0014       // P6_PWM Speed04 register
#define P6_PWM_SPEED05       0x0018       // P6_PWM Spedd05 register
#define P6_PWM_SPEED06       0x001C       // P6_PWM Speed06 register
#define P6_PWM_SPEED07       0x0020       // P6_PWM Speed07 register
#define P6_PWM_SPEED08       0x0024       // P6_PWM Speed08 register
#define P6_PWM_SPEED09       0x0028       // P6_PWM Speed09 register
#define P6_PWM_SPEED10       0x002C       // P6_PWM Speed10 register
#define P6_PWM_SPEED11       0x0030       // P6_PWM Speed11 register
#define P6_PWM_SPEED12       0x0034       // P6_PWM Speed12 register
#define P6_PWM_SPEED13       0x0038       // P6_PWM Speed13 register
#define P6_PWM_SPEED14       0x003C       // P6_PWM Speed14 register
#define P6_PWM_SPEED15       0x0040       // P6_PWM Speed15 register

#define P6_PWM_RATIO00       0x0044       // P6_PWM Ratio00 register
#define P6_PWM_RATIO01       0x0048       // P6_PWM Ratio01 register
#define P6_PWM_RATIO02       0x004C       // P6_PWM Ratio02 register
#define P6_PWM_RATIO03       0x0050       // P6_PWM Ratio03 register
#define P6_PWM_RATIO04       0x0054       // P6_PWM Ratio04 register
#define P6_PWM_RATIO05       0x0058       // P6_PWM Ratio05 register
#define P6_PWM_RATIO06       0x005C       // P6_PWM Ratio06 register
#define P6_PWM_RATIO07       0x0060       // P6_PWM Ratio07 register
#define P6_PWM_RATIO08       0x0064       // P6_PWM Ratio08 register
#define P6_PWM_RATIO09       0x0068       // P6_PWM Ratio09 register
#define P6_PWM_RATIO10       0x006C       // P6_PWM Ratio10 register
#define P6_PWM_RATIO11       0x0070       // P6_PWM Ratio11 register
#define P6_PWM_RATIO12       0x0074       // P6_PWM Ratio12 register
#define P6_PWM_RATIO13       0x0078       // P6_PWM Ratio13 register
#define P6_PWM_RATIO14       0x007C       // P6_PWM Ratio14 register
#define P6_PWM_RATIO15       0x0080       // P6_PWM Ratio15 register

#define P6_PWM_32K           0x0084       // P6_PWM 32KHz register

// Registers bitwise definitions

// Control register
#define P6_PWM_START00      (1<<0)        // Start P6_PWM 00
#define P6_PWM_START01      (1<<1)        // Start P6_PWM 01
#define P6_PWM_START02      (1<<2)        // Start P6_PWM 02
#define P6_PWM_START03      (1<<3)        // Start P6_PWM 03

#define P6_PWM_CLKMODE00    (1<<4)        // Set Clock mode for P6_PWM 00
#define P6_PWM_CLKMODE01    (1<<5)        // Set Clock mode for P6_PWM 01
#define P6_PWM_CLKMODE02    (1<<6)        // Set Clock mode for P6_PWM 02
#define P6_PWM_CLKMODE03    (1<<7)        // Set Clock mode for P6_PWM 03

#define P6_PWM_START04      (1<<8)        // Start P6_PWM 04
#define P6_PWM_START05      (1<<9)        // Start P6_PWM 05
#define P6_PWM_START06      (1<<10)       // Start P6_PWM 06
#define P6_PWM_START07      (1<<11)       // Start P6_PWM 07

#define P6_PWM_CLKMODE04    (1<<12)       // Set Clock mode for P6_PWM 04
#define P6_PWM_CLKMODE05    (1<<13)       // Set Clock mode for P6_PWM 05
#define P6_PWM_CLKMODE06    (1<<14)       // Set Clock mode for P6_PWM 06
#define P6_PWM_CLKMODE07    (1<<15)       // Set Clock mode for P6_PWM 07

#define P6_PWM_START08      (1<<16)        // Start P6_PWM 08
#define P6_PWM_START09      (1<<17)        // Start P6_PWM 09
#define P6_PWM_START10      (1<<18)        // Start P6_PWM 10
#define P6_PWM_START11      (1<<19)        // Start P6_PWM 11

#define P6_PWM_CLKMODE08    (1<<20)        // Set Clock mode for P6_PWM 08
#define P6_PWM_CLKMODE09    (1<<21)        // Set Clock mode for P6_PWM 09
#define P6_PWM_CLKMODE10    (1<<22)        // Set Clock mode for P6_PWM 10
#define P6_PWM_CLKMODE11    (1<<23)        // Set Clock mode for P6_PWM 11

#define P6_PWM_START12      (1<<24)        // Start P6_PWM 12
#define P6_PWM_START13      (1<<25)        // Start P6_PWM 13
#define P6_PWM_START14      (1<<26)        // Start P6_PWM 14
#define P6_PWM_START15      (1<<27)        // Start P6_PWM 15

#define P6_PWM_CLKMODE12    (1<<28)        // Set Clock mode for P6_PWM 12
#define P6_PWM_CLKMODE13    (1<<29)        // Set Clock mode for P6_PWM 13
#define P6_PWM_CLKMODE14    (1<<30)        // Set Clock mode for P6_PWM 14
#define P6_PWM_CLKMODE15    (1<<31)        // Set Clock mode for P6_PWM 15

#define P6_PWM_32K_VAL0      (1<<0)        // Start P6_PWM 00 of 32KHz 
#define P6_PWM_32K_VAL1      (1<<1)        // Start P6_PWM 01 of 32KHz
#define P6_PWM_32K_VAL2      (1<<2)        // Start P6_PWM 02 of 32KHz
#define P6_PWM_32K_VAL3      (1<<3)        // Start P6_PWM 03 of 32KHz
#define P6_PWM_32K_VAL4      (1<<4)        // Start P6_PWM 04 of 32KHz
#define P6_PWM_32K_VAL5      (1<<5)        // Start P6_PWM 05 of 32KHz
#define P6_PWM_32K_VAL6      (1<<6)        // Start P6_PWM 06 of 32KHz
#define P6_PWM_32K_VAL7      (1<<7)        // Start P6_PWM 07 of 32KHz


#endif
