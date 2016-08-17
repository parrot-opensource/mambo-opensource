/**
 * @file linux/include/asm-arm/arch-parrot/regs-sysc-p6.h
 * @brief Parrot6 System controller registers
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2008-11-05
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
#ifndef __ARCH_ARM_PARROT6_SYSCONTROL_H
#define __ARCH_ARM_PARROT6_SYSCONTROL_H

/* System Controller Registers */
#define _P6_SYS_STATUS                0x00
#define _P6_SYS_ITEN                  0x04
#define _P6_SYS_ITACK                 0x08
#define _P6_SYS_CEN                   0x0C
#define _P6_SYS_CDIS                  0x10
#define _P6_SYS_CSTAT                 0x14
#define _P6_SYS_PSD_CEN               0x18
#define _P6_SYS_PSD_CDIS              0x1C
#define _P6_SYS_PSD_CSTAT             0x20
#define _P6_SYS_USB_CLK               0x24
#define _P6_SYS_WDOGCTL               0x28
#define _P6_SYS_RESET                 0x2C
#define _P6_SYS_SPEED                 0x30
#define _P6_SYS_REMAP                 0x34
#define _P6_SYS_SDCARD                0x38
#define _P6_SYS_CHIPID                0x3C   /* Chip ID - compat. with P5/P5+ */

/* I/O control */
#define _P6_SYS_IOCR1                 0x50
#define _P6_SYS_IOCR2                 0x54
#define _P6_SYS_IOCR3                 0x58
#define _P6_SYS_IOCR4                 0x5C
#define _P6_SYS_IOCR5                 0x60
#define _P6_SYS_IOCR6                 0x64
#define _P6_SYS_IOCR7                 0x68
#define _P6_SYS_IOCR8                 0x6C
#define _P6_SYS_IOCR9                 0x70
#define _P6_SYS_IOCR10                0x74
#define _P6_SYS_IOCR11                0x78

/* Timer 0-3 control */
#define _P6_SYS_TIM0CTL               0x100
#define _P6_SYS_TIM0LD                0x104
#define _P6_SYS_TIM1CTL               0x108
#define _P6_SYS_TIM1LD                0x10C
#define _P6_SYS_TIM2CTL               0x110
#define _P6_SYS_TIM2LD                0x114
#define _P6_SYS_TIM3CTL               0x118
#define _P6_SYS_TIM3LD                0x11C
#define _P6_SYS_TIM0CNT               0x120
#define _P6_SYS_TIM1CNT               0x124
#define _P6_SYS_TIM2CNT               0x128
#define _P6_SYS_TIM3CNT               0x12C

#define _P6_SYS_PUPD0                 0x140
#define _P6_SYS_PUPD1                 0x144
#define _P6_SYS_PUPD2                 0x148
#define _P6_SYS_PUPD3                 0x14C
#define _P6_SYS_PUPD4                 0x150

/* DDR SDRAM IO control */
#define _P6_SYS_DDR_IO0               0x160
#define _P6_SYS_DDR_IO1               0x164
#define _P6_SYS_DDR_IO2               0x168
#define _P6_SYS_DDR_IO3               0x16C

/* P6i registers */
#define _P6I_SYS_P1_0                 0x140   /*  Pull up regs config */
#define _P6I_SYS_P1_1                 0x144
#define _P6I_SYS_P2_0                 0x160   /*  Pull down regs config */
#define _P6I_SYS_P2_1                 0x164
#define _P6I_SYS_P2_2                 0x168
#define _P6I_SYS_E1_0                 0x180   /* Drive strengh config */
#define _P6I_SYS_E1_1                 0x184
#define _P6I_SYS_E1_2                 0x188
#define _P6I_SYS_E2_0                 0x1A0
#define _P6I_SYS_E2_1                 0x1A4
#define _P6I_SYS_E2_2                 0x1A8
#define _P6I_SYS_SR0                  0x1C0   /*  Slew Rate config */
#define _P6I_SYS_SR1                  0x1C4
#define _P6I_SYS_SMT0                 0x1E0   /*  Smith trigger select */
#define _P6I_SYS_SMT1                 0x1E4
#define _P6I_SYS_HVPS                 0x220   /*  Active high 3v3 selector */
#define _P6I_SYS_USB_CTL0             0x240   /*  USB registers control */
#define _P6I_SYS_USB_CTL1             0x244
#define _P6I_SYS_USB_STATUS           0x248
#define _P6I_SYS_CODEC_CTL0           0x260   /*  CODEC registers control */
#define _P6I_SYS_CODEC_CTL1           0x264
#define _P6I_SYS_CODEC_CTL2           0x268
#define _P6I_SYS_CODEC_CTL3           0x26C
#define _P6I_SYS_SD_STATUS0           0x2A0   /*  SDIO Debug status */
#define _P6I_SYS_SD_STATUS1           0x2A4
#define _P6I_SYS_SDRAM                0x2B0   /*  SDRAM selection register */

/*
 * ---------------------------------------------------------------------------
 * Register bitwise definitions
 * ---------------------------------------------------------------------------
 */

/* Timer Control register */
#define P6_SYS_TIMXCTL_PRESCALE1      0       /* 0 stage prescale, clock/1 */
#define P6_SYS_TIMXCTL_PRESCALE16     1       /* 4 stage prescale, clock/16 */
#define P6_SYS_TIMXCTL_PRESCALE256    2       /* 8 stage prescale, clock/256 */
#define P6_SYS_TIMXCTL_PRESCALE4096   3       /* 12 stage prescale, clock/4096*/
#define P6_SYS_TIMXCTL_FREERUNNING    (0<<2)  /* Periodic(1)/free running (0) */
#define P6_SYS_TIMXCTL_PERIODIC       (1<<2)  /* Periodic(1)/free running (0) */
#define P6_SYS_TIMXCTL_ENABLE         (1<<3)  /* Timer enabled */

/* Watchdog control register */
#define P6_SYS_WDOGCTL_TICKEN         (1<<28) /* Enable Wdog count & Tick irq*/
#define P6_SYS_WDOGCTL_WDOGEN         (1<<29) /* Enable Watchdog system reset */
#define P6_SYS_WDOGCTL_TICKPREDIV     (1<<30)
#define P6_SYS_WDOGCTL_SYSSOFTRESET   (1<<31) /* Apply reset and release */

/* STATUS register */
#define P6_SYS_STATUS_IT_TICK         (1<<0)
#define P6_SYS_STATUS_IT_TIM0         (1<<1)
#define P6_SYS_STATUS_IT_TIM1         (1<<2)
#define P6_SYS_STATUS_IT_TIM2         (1<<3)
#define P6_SYS_STATUS_IT_TIM3         (1<<4)
#define P6_SYS_STATUS_SOFTRESET       (1<<8)  /* Reset due to a Soft reset */
#define P6_SYS_STATUS_WDOGRESET       (1<<9)  /* Reset due to a Wdog reset */

/* Timer interrupt enable */
#define P6_SYS_ITEN_TICK              (1<<0)
#define P6_SYS_ITEN_TIM0              (1<<1)
#define P6_SYS_ITEN_TIM1              (1<<2)
#define P6_SYS_ITEN_TIM2              (1<<3)
#define P6_SYS_ITEN_TIM3              (1<<4)

/* Timer interrupt acknowledge */
#define P6_SYS_ITACK_TICK             (1<<0)
#define P6_SYS_ITACK_TIM0             (1<<1)
#define P6_SYS_ITACK_TIM1             (1<<2)
#define P6_SYS_ITACK_TIM2             (1<<3)
#define P6_SYS_ITACK_TIM3             (1<<4)

/* Clock enable/disable/status register bits */
#define P6_SYS_CLK_DMA                (1<<0)
#define P6_SYS_CLK_CAMIF0             (1<<1)
#define P6_SYS_CLK_CAMIF1             (1<<2)
#define P6_SYS_CLK_AAI                (1<<3)
#define P6_SYS_CLK_H264               (1<<4)
#define P6_SYS_CLK_LCDC               (1<<5)
#define P6_SYS_CLK_CV                 (1<<6)
#define P6_SYS_CLK_USB0               (1<<7)
#define P6_SYS_CLK_USB1               (1<<8)
#define P6_SYS_CLK_SDIO               (1<<9)
#define P6_SYS_CLK_PARINT             (1<<10)
#define P6_SYS_CLK_NAND               (1<<11)
#define P6_SYS_CLK_PWM                (1<<12)
#define P6_SYS_CLK_CAN                (1<<13)
#define P6_SYS_CLK_UARTSIM            (1<<14)
#define P6_SYS_CLK_GPIO               (1<<15)
#define P6_SYS_CLK_MOST               (1<<16)
#define P6_SYS_CLK_SPI0               (1<<17)
#define P6_SYS_CLK_SPI1               (1<<18)
#define P6_SYS_CLK_SPI2               (1<<19)
#define P6_SYS_CLK_UART0              (1<<20)
#define P6_SYS_CLK_UART1              (1<<21)
#define P6_SYS_CLK_UART2              (1<<22)
#define P6_SYS_CLK_UART3              (1<<23)
#define P6_SYS_CLK_AHBMON             (1<<24)
#define P6_SYS_CLK_I2CS               (1<<25)
#define P6_SYS_CLK_I2CM0              (1<<26)
#define P6_SYS_CLK_I2CM1              (1<<27)
#define P6_SYS_CLK_INTROM             (1<<28)
#define P6_SYS_CLK_MPMC               (1<<29)
#define P6_SYS_CLK_BUSMX              (1<<30)
#define P6_SYS_CLK_ARM                (1<<31) /* can only be disabled */

/* PSD clock enable/disable/status register bits */
#define P6_SYS_CLK_PSD0               (1<<0)
#define P6_SYS_CLK_PSD1               (1<<1)

/* REMAP register */
#define P6_SYS_REMAP_REMAP            (1<<0)  /* Boot on ROM(0) or FLASH(1) */
#define P6_SYS_REMAP_DEBUG            (1<<1)  /* DEBUG enable */

#define P6_SYS_REMAP_NBBITS           (1<<4)  /* Number of Bits for NAND */
#define P6_SYS_REMAP_GENERATION       (1<<5)  /* Generation type of NAND */
#define P6_SYS_REMAP_NBADDR           (1<<6)  /* Number of address for NAND */
#define P6_SYS_REMAP_NAND_WP          (1<<7)  /* NAND WP pin */

/* SPEED register */
#define P6_SYS_SPEED_156MHZ           (1<<0)  /* 156MHz (1) or 130MHz (0) */

/* RESET status register (An pads values on reset) */
#define P6_SYS_RESET_A0               (1<<0)
#define P6_SYS_RESET_A1               (1<<1)
#define P6_SYS_RESET_A2               (1<<2)
#define P6_SYS_RESET_A3               (1<<3)
#define P6_SYS_RESET_A4               (1<<4)
#define P6_SYS_RESET_A5               (1<<5)
#define P6_SYS_RESET_A6               (1<<6)
#define P6_SYS_RESET_A7               (1<<7)
#define P6_SYS_RESET_A8               (1<<8)
#define P6_SYS_RESET_A9               (1<<9)
#define P6_SYS_RESET_A10              (1<<10)
#define P6_SYS_RESET_A11              (1<<11)
#define P6_SYS_RESET_A12              (1<<12)
#define P6_SYS_RESET_A13              (1<<13)
#define P6_SYS_RESET_A14              (1<<14)
#define P6_SYS_RESET_A15              (1<<15)
#define P6_SYS_RESET_A16              (1<<16)
#define P6_SYS_RESET_A17              (1<<17)
#define P6_SYS_RESET_A18              (1<<18)
#define P6_SYS_RESET_A19              (1<<19)
#define P6_SYS_RESET_A20              (1<<20)
#define P6_SYS_RESET_A21              (1<<21)
#define P6_SYS_RESET_A22              (1<<22)
#define P6_SYS_RESET_A23              (1<<23)
#define P6_SYS_RESET_A24              (1<<24)
#define P6_SYS_RESET_A25              (1<<25)

/* ASIC chip identification strings */
#define P6_SYS_CHIPID_P5              (0x312D3550) /* "P5-1" */
#define P6_SYS_CHIPID_P5P             (0x31503550) /* "P5P1" */
#define P6_SYS_CHIPID_P6              (0x30523650) /* "P6R0" */
#define P6_SYS_CHIPID_P6I             (0x30693650) /* "P6i0" */

/* SDCARD Clock register */

/* with AHB clock at 130 MHZ */
#define P6_SYS_130MHZ_SDIO_43_3MHZ    (0<<0)
#define P6_SYS_130MHZ_SDIO_32_5MHZ    (1<<0)
#define P6_SYS_130MHZ_SDIO_26_0MHZ    (2<<0)
#define P6_SYS_130MHZ_SDIO_21_7MHZ    (3<<0)
#define P6_SYS_130MHZ_SDIO_18_6MHZ    (4<<0)
#define P6_SYS_130MHZ_SDIO_16_3MHZ    (5<<0)
#define P6_SYS_130MHZ_SDIO_14_4MHZ    (6<<0)
#define P6_SYS_130MHZ_SDIO_13_0MHZ    (7<<0)

/* with AHB clock at 156 MHZ */
#define P6_SYS_156MHZ_SDIO_52_0MHZ    (0<<0)
#define P6_SYS_156MHZ_SDIO_39_0MHZ    (1<<0)
#define P6_SYS_156MHZ_SDIO_31_2MHZ    (2<<0)
#define P6_SYS_156MHZ_SDIO_26_0MHZ    (3<<0)
#define P6_SYS_156MHZ_SDIO_22_3MHZ    (4<<0)
#define P6_SYS_156MHZ_SDIO_19_5MHZ    (5<<0)
#define P6_SYS_156MHZ_SDIO_17_3MHZ    (6<<0)
#define P6_SYS_156MHZ_SDIO_15_6MHZ    (7<<0)

/* PSD Clock register */

/* with AHB clock at 130 MHZ */
#define P6_SYS_130MHZ_PSD_21_7MHZ    (0<<0)
#define P6_SYS_130MHZ_PSD_18_6MHZ    (1<<0)
#define P6_SYS_130MHZ_PSD_16_3MHZ    (2<<0)
#define P6_SYS_130MHZ_PSD_14_4MHZ    (3<<0)
#define P6_SYS_130MHZ_PSD_13_0MHZ    (4<<0)
#define P6_SYS_130MHZ_PSD_11_8MHZ    (5<<0)
#define P6_SYS_130MHZ_PSD_10_8MHZ    (6<<0)
#define P6_SYS_130MHZ_PSD_10_0MHZ    (7<<0)
/* with AHB clock at 156 MHZ */
#define P6_SYS_156MHZ_PSD_26_0MHZ    (0<<0)
#define P6_SYS_156MHZ_PSD_22_3MHZ    (1<<0)
#define P6_SYS_156MHZ_PSD_19_5MHZ    (2<<0)
#define P6_SYS_156MHZ_PSD_17_3MHZ    (3<<0)
#define P6_SYS_156MHZ_PSD_15_6MHZ    (4<<0)
#define P6_SYS_156MHZ_PSD_14_2MHZ    (5<<0)
#define P6_SYS_156MHZ_PSD_13_0MHZ    (6<<0)
#define P6_SYS_156MHZ_PSD_12_0MHZ    (7<<0)

/*  _P6_SYS_IOCR11 */
#define P6_SYS_NJTAG_SPI             (1<<4)

/* P6i additional defines */

/* Codec control 0 register mask */
#define P6I_SYS_CODEC_PDDACLZ_MASK     	 (1<<1)
#define P6I_SYS_CODEC_PDDACRZ_MASK     	 (1<<2)
#define P6I_SYS_CODEC_PDAUXDRVLZ_MASK    (1<<3)
#define P6I_SYS_CODEC_PDAUXDRVRZ_MASK    (1<<4)
#define P6I_SYS_CODEC_PDADCLZ_MASK     	 (1<<5)
#define P6I_SYS_CODEC_PDADCRZ_MASK     	 (1<<6)
#define P6I_SYS_CODEC_PDLPGAZ_MASK     	 (1<<7)
#define P6I_SYS_CODEC_PDRPGAZ_MASK     	 (1<<8)
#define P6I_SYS_CODEC_PDMBIASZ_MASK      (1<<9)

/* Codec control 1 register mask */
#define P6I_SYS_CODEC_RECSEL_MASK     	 (65535)
#define P6I_SYS_CODEC_LATCH_MASK     	 (1<<16)
#define P6I_SYS_CODEC_I2SFS_MASK     	 (15<<17)

/* Codec control 2 register mask */
#define P6I_SYS_CODEC_MICVOL_MASK     	 (65535<<0)
#define P6I_SYS_CODEC_RECVOL_MASK     	 (65535<<16)

/* Codec control 3 register mask */
#define P6I_SYS_CODEC_LMMUTE_MASK     	 (3<<0)
#define P6I_SYS_CODEC_MICMUTE_MASK     	 (3<<2)
#define P6I_SYS_CODEC_RECMUTE_MASK     	 (3<<4)

/* PHY USB register */

/* USB Control 0 register (_P6I_SYS_USB_CTL0 @0x240) */
#define P6I_SYS_USB_CTL0_PHY_EXTCAL             (31<<0)
#define P6I_SYS_USB_CTL0_PHY_CHGRDETEN          (1<<5)
#define P6I_SYS_USB_CTL0_PHY_CHGRMODE           (1<<6)
#define P6I_SYS_USB_CTL0_PHY_CHGRDETON          (1<<7)
#define P6I_SYS_USB_CTL0_PHY_HSDRVTIMINGP       (1<<8)
#define P6I_SYS_USB_CTL0_PHY_HSDRVTIMINGN       (3<<9)
#define P6I_SYS_USB_CTL0_PHY_HSDRVAMPLITUDE     (3<<11)
#define P6I_SYS_USB_CTL0_PHY_HSDRVSLOPE         (15<<13)
#define P6I_SYS_USB_CTL0_PHY_HSDEDVSEL          (3<<17)
#define P6I_SYS_USB_CTL0_PHY_FSTUNEVSEL         (7<<19)
#define P6I_SYS_USB_CTL0_PHY_HSTEDVSEL          (3<<22)
#define P6I_SYS_USB_CTL0_PHY_ICPCTRL            (3<<24)
#define P6I_SYS_USB_CTL0_PHY_LSRFTSEL           (3<<26)
#define P6I_SYS_USB_CTL0_PHY_FSRFTSEL           (3<<28)
#define P6I_SYS_USB_CTL0_PHY_PREEMDEPTH         (1<<30)
#define P6I_SYS_USB_CTL0_PHY_ENPRE              (1<<31)

/* USB Control 1 register (_P6I_SYS_USB_CTL1 @0x244) */
#define P6I_SYS_USB_CTL1_COMP_ALWAYS_ON         (1<<0)
#define P6I_SYS_USB_CTL1_CONF3                  (1<<1)
#define P6I_SYS_USB_CTL1_PHY_CALBP              (1<<2)

/* DDR/SDR*/
#define P6I_SYS_SDRAM_SDR_nDDR                  (1<<0)
#define P6I_SYS_SDRAM_RUN_SDRAM                 (1<<1)

#endif /* __ARCH_ARM_PARROT6_SYSCONTROL_H */
