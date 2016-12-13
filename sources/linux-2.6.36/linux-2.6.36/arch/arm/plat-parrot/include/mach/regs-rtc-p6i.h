/**
 * @file   regs-rtc-p6i.h
 * @brief  P6i RTC registers
 *
 * @author ivan.djelic@parrot.com
 * @date   2010-07-20
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

#ifndef P6I_RTC_H
#define P6I_RTC_H

#define _RTC_PWR_DELAY_ADDR  0x00
#define _RTC_CORE_DELAY      0x04
#define _RTC_PWR_CTRL        0x08
#define _RTC_OSC_CHOICE      0x0C
#define _RTC_OSC_CTRL        0x10
#define _RTC_INTERRUPT       0x14
#define _RTC_OUT_CTRL        0x18
#define _RTC_WU_32K_CTRL     0x1C
#define _RTC_WU_32K_DB       0x20
#define _RTC_WU_26M          0x24
#define _RTC_COMP_START      0x28
#define _RTC_COMP_N          0x2C
#define _RTC_COMP_M          0x30
#define _RTC_COMP_1HZ        0x34
#define _RTC_OUT_0_PER       0x38
#define _RTC_OUT_0_DUT       0x3C
#define _RTC_OUT_1_PER       0x40
#define _RTC_OUT_1_DUT       0x44
#define _RTC_OUT_2_PER       0x48
#define _RTC_OUT_2_DUT       0x4C
#define _RTC_OUT_3_PER       0x50
#define _RTC_OUT_3_DUT       0x54
#define _RTC_COUNTER         0x58
#define _RTC_ALARM           0x5C
#define _RTC_SLEW_SCHMITT    0x60
#define _RTC_PULLS           0x64
#define _RTC_DRIVES          0x68
#define _RTC_IO              0x6C
#define _RTC_SCRAP           0x70
#define _RTC_FUSE_A0         0x80
#define _RTC_FUSE_A1         0x84
#define _RTC_FUSE_B0         0x88
#define _RTC_FUSE_B1         0x8C
#define _RTC_FUSE_SOP        0x90
#define _RTC_FUSE_MAGIC      0x94

/* Parameters */
#define STS_RTC_ALARM       (1<<0)
#define STS_32K_COMP        (1<<1)
#define STS_WU0_32K         (1<<2)
#define STS_WU1_32K         (1<<3)
#define STS_WU2_32K         (1<<4)
#define STS_RX_32K          (1<<5)
#define STS_CTS_32K         (1<<6)
#define STS_RX_26M          (1<<7)
#define STS_CTS_26M         (1<<8)
#define MASK_RTC_ALARM      (1<<9)
#define MASK_32K_COMP       (1<<10)
#define MASK_WU0_32K        (1<<11)
#define MASK_WU1_32K        (1<<12)
#define MASK_WU2_32K        (1<<13)
#define MASK_RX_32K         (1<<14)
#define MASK_CTS_32K        (1<<15)
#define MASK_RX_26M         (1<<16)
#define MASK_CTS_26M        (1<<17)
#define ACK_RTC_ALARM       (1<<18)
#define ACK_32K_COMP        (1<<19)
#define ACK_WU0_32K         (1<<20)
#define ACK_WU1_32K         (1<<21)
#define ACK_WU2_32K         (1<<22)
#define ACK_RX_32K          (1<<23)
#define ACK_CTS_32K         (1<<24)
#define ACK_RX_26M          (1<<25)
#define ACK_CTS_26M         (1<<26)

#define RTC_PWR_CTRL_V_ADJUST_1V2_0          (1<< 0)
#define RTC_PWR_CTRL_V_ADJUST_1V2_1          (1<< 1)
#define RTC_PWR_CTRL_EN_VADJ_1V2_pmu         (1<< 2)
#define RTC_PWR_CTRL_BYP_CODEC_BG            (1<< 3)
#define RTC_PWR_CTRL_USB_LDO_EN              (1<< 4)
#define RTC_PWR_CTRL_CODEC_LDO_EN            (1<< 5)
#define RTC_PWR_CTRL_CODEC_2v8_n3v0          (1<< 6)
#define RTC_PWR_CTRL_EN_CODEC_BG             (1<< 7)
#define RTC_PWR_CTRL_LPM_EN                  (1<< 8)
#define RTC_PWR_CTRL_DLPM_EN                 (1<< 9)

#endif /* P6I_RTC_H */

