/**
 * @file linux/include/asm-arm/arch-parrot/regs-sysc-p5.h
 * @brief Parrot5/5+ System controller registers
 *
 * Copyright (C) 2005,2006,2007 Parrot S.A.
 *
 * @author     yves.lemoine@parrot.com
 * @author     ivan.djelic@parrot.com
 * @date       2005-11-17
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
#ifndef __ARCH_ARM_PARROT5_SYSCONTROL_H
#define __ARCH_ARM_PARROT5_SYSCONTROL_H

/* System Controller Registers */
#define _P5_SYS_TIM0CTL               0x00    /* Timer0 Control */
#define _P5_SYS_TIM0LD                0x04    /* Timer0 Load Value    (W) */
#define _P5_SYS_TIM0VAL               0x04    /* Timer0 Current Value (R) */
#define _P5_SYS_TIM1CTL               0x08    /* Timer1 Control */
#define _P5_SYS_TIM1LD                0x0C    /* Timer1 Load Value    (W) */
#define _P5_SYS_TIM1VAL               0x0C    /* Timer1 Current Value (R) */
#define _P5_SYS_TIM2CTL               0x10    /* Timer2 Control */
#define _P5_SYS_TIM2LD                0x14    /* Timer2 Load Value    (W) */
#define _P5_SYS_TIM2VAL               0x14    /* Timer2 Current Value (R) */
#define _P5_SYS_TIM3CTL               0x18    /* Timer3 Control */
#define _P5_SYS_TIM3LD                0x1C    /* Timer3 Load Value    (W) */
#define _P5_SYS_TIM3VAL               0x1C    /* Timer3 Current Value (R) */
#define _P5_SYS_WDOGCTL               0x20    /* Watchdog Control */
#define _P5_SYS_FLAGS                 0x24    /* Flags */
#define _P5_SYS_CEN                   0x28    /* Clock Enable */
#define _P5_SYS_CDIS                  0x2C    /* Clock Disable */
#define _P5_SYS_CSTAT                 0x30    /* Clock Status */
#define _P5_SYS_REMAP                 0x34    /* Remap */
#define _P5_SYS_RESET                 0x38    /* Reset status */
#define _P5_SYS_CHIPID                0x3C    /* Chip Identification */
#define _P5_SYS_IOCR1                 0x40    /* IO Configuration  1 */
#define _P5_SYS_IOCR2                 0x44    /* IO Configuration  2 */
#define _P5_SYS_IOCR3                 0x48    /* IO Configuration  3 */
#define _P5_SYS_IOCR4                 0x4C    /* IO Configuration  4 */
#define _P5_SYS_SDBYMODE              0x50    /* Standby Mode */
#define _P5_SYS_IODBG                 0x54    /* IO Debug register */
#define _P5_SYS_SDCARD                0x58    /* SDcard clock register */

/* Parrot5+ extensions */
#define _P5P_SYS_IOCR5                 0x5C    /* IO Configuration  5 */
#define _P5P_SYS_IOCR6                 0x60    /* IO Configuration  6 */
#define _P5P_SYS_NAND                  0x64    /* nReady/Busy nand on bit 0 */
#define _P5P_SYS_PULL                  0x68    /* Pullup/down Disable */
#define _P5P_SYS_USB                   0x6C    /* USB Control */

/* Register bitwise definitions */

/* Timer Control register */
#define P5_SYS_TIMXCTL_PRESCALE1      0       /* 0 stage prescale, clock/1 */
#define P5_SYS_TIMXCTL_PRESCALE16     1       /* 4 stage prescale, clock/16*/
#define P5_SYS_TIMXCTL_PRESCALE256    2       /* 8 stage prescale, clock/256*/
#define P5_SYS_TIMXCTL_PERIODIC       (1<<2)  /* Periodic(1)/free running(0) */
#define P5_SYS_TIMXCTL_ENABLE         (1<<3)  /* Timer enabled */

/* Watchdog control register */
/* Enable Watchdog counter and Tick interrupt (IT_TICK) */
#define P5_SYS_WDOGCTL_TICKEN         (1<<12)
#define P5_SYS_WDOGCTL_WDOGEN         (1<<13) /* Enable Watchdog system reset */

/* EBC is under reset (reset is then released when a 0 is written) */
#define P5_SYS_WDOGCTL_EBCSOFTRESET   (1<<14)
/* A reset is applied to the system and then released */
#define P5_SYS_WDOGCTL_SYSSOFTRESET   (1<<15)

/* Flags register */
#define P5_SYS_FLAGS_TICK             1       /* Tick interrupt status bit */
#define P5_SYS_FLAGS_TIM0             (1<<1)  /* Timer 0 interrupt status bit */
#define P5_SYS_FLAGS_TIM1             (1<<2)  /* Timer 1 interrupt status bit */
#define P5_SYS_FLAGS_TIM2             (1<<3)  /* Timer 2 interrupt status bit */
#define P5_SYS_FLAGS_TIM3             (1<<4)  /* Timer 3 interrupt status bit */
#define P5_SYS_FLAGS_RST_BRK          (1<<5)  /* Breakpoint reset bit */

/* Clock enable register */
#define P5_SYS_CEN_EBCCLK             (1<<2)  /* Enable EBC clocks (4,26,52) */
#define P5_SYS_CEN_UART0CLK           (1<<3)  /* Enable UART0 clock */
#define P5_SYS_CEN_UART1CLK           (1<<4)  /* Enable UART1 clock */
#define P5_SYS_CEN_UART2CLK           (1<<5)  /* Enable UART2 clock */
#define P5_SYS_CEN_SIM0CLK            (1<<6)  /* Enable SIM0 clock */
#define P5_SYS_CEN_SIM1CLK            (1<<7)  /* Enable SIM1 clock */
#define P5_SYS_CEN_SIM2CLK            (1<<8)  /* Enable SIM2 clock */
#define P5_SYS_CEN_CANCLK             (1<<9)  /* Enable CAN clock */
#define P5_SYS_CEN_SPICLK             (1<<10) /* Enable SPI clock */
#define P5_SYS_CEN_PWMCLK             (1<<11) /* Enable PWM clock */
#define P5_SYS_CEN_PSDCLK             (1<<12) /* Enable PSD clock */
#define P5_SYS_CEN_CAMIFCLK           (1<<13) /* Enable CAMIF/ME clock */
#define P5_SYS_CEN_I2CSCLK            (1<<14) /* Enable I2C SLAVE  clock */
#define P5_SYS_CEN_I2CMCLK            (1<<15) /* Enable I2C MASTER clock */

/* Parrot5+ extensions */
#define P5P_SYS_CEN_PSD1CLK            (1<<16) /* Enable PSD1 clock */
#define P5P_SYS_CEN_DMACLK             (1<<17) /* Enable DMAC clock */
#define P5P_SYS_CEN_USB0CLK            (1<<18) /* Enable USB0 clock */
#define P5P_SYS_CEN_USB1CLK            (1<<19) /* Enable USB1 clock */
#define P5P_SYS_CEN_PCMCLK             (1<<21) /* Enable PCM clock */


/* Clock disable register */
#define P5_SYS_CDIS_ARMCLK            (1<<0)  /* Disable ARM clock and perip. */
#define P5_SYS_CDIS_SELFREQ           (1<<1)  /* Request SDRAM self-refresh */
#define P5_SYS_CDIS_EBCCLK            (1<<2)  /* Disable EBC clocks(4,26,52) */
#define P5_SYS_CDIS_UART0CLK          (1<<3)  /* Disable UART0 clock */
#define P5_SYS_CDIS_UART1CLK          (1<<4)  /* Disable UART1 clock */
#define P5_SYS_CDIS_UART2CLK          (1<<5)  /* Disable UART2 clock */
#define P5_SYS_CDIS_SIM0CLK           (1<<6)  /* Disable SIM0 clock */
#define P5_SYS_CDIS_SIM1CLK           (1<<7)  /* Disable SIM1 clock */
#define P5_SYS_CDIS_SIM2CLK           (1<<8)  /* Disable SIM2 clock */
#define P5_SYS_CDIS_CANCLK            (1<<9)  /* Disable CAN clock */
#define P5_SYS_CDIS_SPICLK            (1<<10) /* Disable SPI clock */
#define P5_SYS_CDIS_PWMCLK            (1<<11) /* Disable PWM clock */
#define P5_SYS_CDIS_PSDCLK            (1<<12) /* Disable PSD clock */
#define P5_SYS_CDIS_CAMIFCLK          (1<<13) /* Disable CAMIF/ME clock */
#define P5_SYS_CDIS_I2CSCLK           (1<<14) /* Disable I2C SLAVE  clock */
#define P5_SYS_CDIS_I2CMCLK           (1<<15) /* Disable I2C MASTER clock */

/* Parrot5+ extensions */
#define P5P_SYS_CDIS_PSD1CLK           (1<<16) /* Disable PSD1 clock */
#define P5P_SYS_CDIS_DMACLK            (1<<17) /* Disable DMAC clock */
#define P5P_SYS_CDIS_USB0CLK           (1<<18) /* Disable USB0 clock */
#define P5P_SYS_CDIS_USB1CLK           (1<<19) /* Disable USB1 clock */
#define P5P_SYS_CDIS_PCMCLK            (1<<21) /* Disable PCM clock */

/* Clock status register */
#define P5_SYS_CSTAT_EBCCLK           (1<<2)  /* Status for EBC clks(4,26,52) */
#define P5_SYS_CSTAT_UART0CLK         (1<<3)  /* Status for UART0 clock */
#define P5_SYS_CSTAT_UART1CLK         (1<<4)  /* Status for UART1 clock */
#define P5_SYS_CSTAT_UART2CLK         (1<<5)  /* Status for UART2 clock */
#define P5_SYS_CSTAT_SIM0CLK          (1<<6)  /* Status for SIM0 clock */
#define P5_SYS_CSTAT_SIM1CLK          (1<<7)  /* Status for SIM1 clock */
#define P5_SYS_CSTAT_SIM2CLK          (1<<8)  /* Status for SIM2 clock */
#define P5_SYS_CSTAT_CANCLK           (1<<9)  /* Status for CAN clock */
#define P5_SYS_CSTAT_SPICLK           (1<<10) /* Status for SPI clock */
#define P5_SYS_CSTAT_PWMCLK           (1<<11) /* Status for PWM clock */
#define P5_SYS_CSTAT_PSDCLK           (1<<12) /* Status for PSD clock */
#define P5_SYS_CSTAT_CAMIFCLK         (1<<13) /* Status for CAMIF/ME clock */
#define P5_SYS_CSTAT_I2CSCLK          (1<<14) /* Status I2C SLAVE  clock */
#define P5_SYS_CSTAT_I2CMCLK          (1<<15) /* Status I2C MASTER clock */

/* Parrot5+ extensions */
#define P5P_SYS_CSTAT_PSD1CLK          (1<<16) /* Status for PSD1 clock */
#define P5P_SYS_CSTAT_DMACLK           (1<<17) /* Status for DMA clock */
#define P5P_SYS_CSTAT_USB0CLK          (1<<18) /* Status for USB0 clock */
#define P5P_SYS_CSTAT_USB1CLK          (1<<19) /* Status for USB1 clock */
#define P5P_SYS_CSTAT_PLLUSB           (1<<20) /* Status for USB PLL */

/* Parrot5+ USB control Register */
#define P5_SYS_USBC_SUSPEND0          (1<<0) /* USB0 controller suspend state */
#define P5_SYS_USBC_PWRDOWN0          (1<<1) /* USB0 controller suspend state */
#define P5_SYS_USBC_SUSPEND1          (1<<2) /* USB0 controller suspend state */
#define P5_SYS_USBC_PWRDOWN1          (1<<3) /* USB0 controller suspend state */

/* Remap register */
#define P5_SYS_REMAP_REMAP            (1<<0)  /* Boot on ROM (0) or FLASH (1) */
#define P5_SYS_REMAP_DEBUGEN          (1<<1)  /* Debug is allowed(1) or not(0)*/

/* Reset status register */
#define P5_SYS_RESET_A0               (1<<0)  /* Value of A0 pad at reset */
#define P5_SYS_RESET_A1               (1<<1)  /* Value of A1 pad at reset */
#define P5_SYS_RESET_A2               (1<<2)  /* Value of A2 pad at reset */
#define P5_SYS_RESET_A3               (1<<3)  /* Value of A3 pad at reset */
#define P5_SYS_RESET_A4               (1<<4)  /* Value of A4 pad at reset */
#define P5_SYS_RESET_A5               (1<<5)  /* Value of A5 pad at reset */
#define P5_SYS_RESET_A6               (1<<6)  /* Value of A6 pad at reset */
#define P5_SYS_RESET_A7               (1<<7)  /* Value of A7 pad at reset */
#define P5_SYS_RESET_A8               (1<<8)  /* Value of A8 pad at reset */
#define P5_SYS_RESET_A9               (1<<9)  /* Value of A9 pad at reset */
#define P5_SYS_RESET_A10              (1<<10) /* Value of A10 pad at reset */
#define P5_SYS_RESET_A11              (1<<11) /* Value of A11 pad at reset */
#define P5_SYS_RESET_A12              (1<<12) /* Value of A12 pad at reset */
#define P5_SYS_RESET_A13              (1<<13) /* Value of A13 pad at reset */
#define P5_SYS_RESET_A14              (1<<14) /* Value of A14 pad at reset */
#define P5_SYS_RESET_A15              (1<<15) /* Value of A15 pad at reset */
#define P5_SYS_RESET_A16              (1<<16) /* Value of A16 pad at reset */
#define P5_SYS_RESET_A17              (1<<17) /* Value of A17 pad at reset */
#define P5_SYS_RESET_SOFTRESET        (1<<18) /* Reset was due to a Soft rst */
#define P5_SYS_RESET_WDOGRESET        (1<<19) /* Reset was due to a Wdog rst*/
#define P5_SYS_RESET_EMCYRESET        (1<<20) /* Reset was due to an Emcy rst */

/* IO control register 4 */
#define P5_SYS_IOCR4_DQM              (1<<0)  /* DQM are used (0) or not (1) */
#define P5_SYS_IOCR4_RADIO_BIDIR      (1<<1)  /* BT radio, 1=bidir, 0=unidir */
#define P5_SYS_IOCR4_EDR              (1<<15) /* Use a BT EDR module */

/* Parrot5+ extension */
#define P5P_SYS_IOCR4_NAND             (1<<16)  /* NAND interface activation */

/* ASIC chip identification strings */
#define P5_SYS_CHIPID_P5              (0x312D3550) /* "P5-1" */
#define P5_SYS_CHIPID_P5P             (0x31503550) /* "P5P1" */

/* SDCARD Clock register */
#define P5_SYS_SDCARD0_13MHZ          (3<<0)
#define P5_SYS_SDCARD0_15MHZ          (2<<0)
#define P5_SYS_SDCARD0_17MHZ          (1<<0)
#define P5_SYS_SDCARD0_20MHZ          (0<<0)

#define P5_SYS_SDCARD1_13MHZ          (3<<2)
#define P5_SYS_SDCARD1_15MHZ          (2<<2)
#define P5_SYS_SDCARD1_17MHZ          (1<<2)
#define P5_SYS_SDCARD1_20MHZ          (0<<2)

/* SDcard clock freq (old macros) */
#define P5_SYS_SDCARD_20              0 /* 20.8  MHz */
#define P5_SYS_SDCARD_17              1 /* 17.33 Mhz */
#define P5_SYS_SDCARD_14              2 /* 14.85 Mhz */
#define P5_SYS_SDCARD_13              3 /* 13.0  Mhz */

#endif /* __ARCH_ARM_PARROT5_SYSCONTROL_H */
