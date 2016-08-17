/*
 *  linux/arch/arm/mach-parrot6/parrot6idev.c
 *
 *  Copyright (C) 2008 Parrot S.A.
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

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/mmc/host.h>
#include <linux/i2c.h>

/* Functions to sleep */
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include <mach/mmc.h>
#include <mach/usb.h>
#include <mach/regs-pwm-p6.h>

#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"


#define DELOS_HSIS__GPIO_FLASH_WP 29

/* GPIO numbers */
#define DELOS_HSIS__GPIO_RESET_MOTOR_FAULT   57
#define DELOS_HSIS__GPIO_MOTOR_ENABLE        53
#define DELOS_HSIS__GPIO_MOTOR_FAULT         2
#define DELOS_HSIS__GPIO_RESET_PIC           58
#define DELOS_HSIS__GPIO_PROG_PIC_DATA       56
#define DELOS_HSIS__GPIO_PROG_PIC_CLK        52

/* Corresponding PIN names */
#define DELOS_HSIS__GPIOPIN_RESET_MOTOR_FAULT        GPIO_057
#define DELOS_HSIS__GPIOPIN_MOTOR_ENABLE             GPIO_053
#define DELOS_HSIS__GPIOPIN_MOTOR_FAULT          P6I_GPIO_002
#define DELOS_HSIS__GPIOPIN_RESET_PIC                GPIO_058
#define DELOS_HSIS__GPIOPIN_PROG_PIC_DATA            GPIO_056
#define DELOS_HSIS__GPIOPIN_PROG_PIC_CLK             GPIO_052


#define DELOS_USE_USB
#define DELOS_USE_PWM
#define DELOS_USE_PIC
#define DELOS_USE_BLE
static unsigned int pins_init_p6i[] = {
        /* uart */
#ifdef DELOS_USE_BLE
        P6I_UART0_DEFAULT,        /* Delos Bluetooth Chip UART - with rts/cts - "P6I_UART1_RXTX_DEFAULT" does not work here */
#endif
        P6I_UART1_RXTX_DEFAULT,   /* Delos Console UART        - no rts/cts */
#ifdef DELOS_USE_PIC
        P6I_UART2_RXTX_DEFAULT,   /* Delos HW00 PIC UART                    */
#endif
        P6I_NAND8_DEFAULT,
        P6I_I2CM0_DEFAULT,
        //    P6I_SD0_DEFAULT,
        //    P6I_MMC_GPIO_DEFAULT,
        //    P6I_AAI_I2S_SYNC_DEFAULT,
        //    P6I_AAI_PCM0_DEFAULT,
        //    P6I_I2S_IN1,
        //    P6I_I2S_OUT0,
        /* Activate USB */
        //    P6I_GPIO_004,
        /* activate bluetooth chip */
        P6I_GPIO_005,
#ifdef DELOS_USE_PIC
        /* HW00 Navboard GPIO */
        DELOS_HSIS__GPIOPIN_RESET_PIC,
        DELOS_HSIS__GPIOPIN_PROG_PIC_DATA,
        DELOS_HSIS__GPIOPIN_PROG_PIC_CLK,
#endif
        /* INT INERT GPIO */
        GPIO_054,
        /* RTC 32K out for TI BT */
        P6I_RTC_32K_OUT,
        /* usb pwr on */
        P6I_USB_PWR_ON,
#ifdef DELOS_USE_PWM
        /* Delos motors */
        DELOS_HSIS__GPIOPIN_MOTOR_FAULT,        /*  2 */
        DELOS_HSIS__GPIOPIN_MOTOR_ENABLE,       /* 53 */
        DELOS_HSIS__GPIOPIN_RESET_MOTOR_FAULT,  /* 57 */
#endif
        GPIO_030,
        P6I_SPI1_DEFAULT, 
        0,
};

static struct platform_device *p6i_devices[] __initdata = {
#ifdef DELOS_USE_BLE
        &p6_uart0_device,
#endif
        &p6_uart1_device,
        &p6_uart2_device,
        &p6_nand_device,
        &p6_aai_device,
        &p6_i2cm0_device,
        //&p6_i2cm1_device,
        &p6_spi2_device,
        &p6_sdhci0_device,
        //&dmamem_device,
        //&p6mu_rtc,
        //&p6_acpower_device,
#ifdef DELOS_USE_USB
        &p6i_usb0_device,
#endif
        /* virtual device */
        &p6_dmac_device,
        &p6_gpio,
};

static unsigned int pins_init_p6i_pwms[] = {
#ifdef DELOS_USE_PWM
        /* PWM motors */
        P6I_PWM_00a,
        P6I_PWM_01a,
        P6I_PWM_02b,
        P6I_PWM_03b,
#endif
        0,
};


#ifdef DELOS_USE_PWM
static void delos_reset_pwms(void)
{
        char __iomem *pwm_regbase;
        const int pwm_ctrl_enable_all = 0x0f;
        unsigned int ntimer;

        pwm_regbase = ioremap(PARROT6_PWM, 256);

        /* Force all ratios to zero */
        for (ntimer=0;ntimer<4;ntimer++){
                __raw_writel(0, pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
        }
        /* Activate the PWM output to generate the 0 output */
        __raw_writel(pwm_ctrl_enable_all, pwm_regbase + P6_PWM_CTL);
}

typedef struct { unsigned int val[4]; } __attribute__ ((packed)) pwm_delos_quadruplet;
void delos_set_raw_ctrl_reg(unsigned int reg);
void delos_set_raw_ratios(pwm_delos_quadruplet* ratios);
#endif



static void __init p6idev_init(void)
{
        printk("Initializing Delos board ...\n");

        p6i_init();
        ssleep(2);
        parrot_init_pin(pins_init_p6i);

#warning **
#warning ** Delos : Delos SiP6 Dev Board with OV7740 USB companion chip
#warning **


        /* disable NAND flash write protection on P6 dev */
        printk("Setting NAND RW ...\n");
        gpio_direction_output(DELOS_HSIS__GPIO_FLASH_WP, 1);

#ifdef DELOS_USE_PIC
        /* Navboard gpio */
        printk("Initializing Navboard ICSP interface ...\n");
        gpio_direction_output(DELOS_HSIS__GPIO_RESET_PIC, 0);
        gpio_direction_output(DELOS_HSIS__GPIO_PROG_PIC_DATA, 0);
        gpio_direction_output(DELOS_HSIS__GPIO_PROG_PIC_CLK, 0);
#endif
        //gpio_direction_output(GPIO_054,0);
        gpio_direction_input(54);
        gpio_interrupt_register(54, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

        printk("Initializing USB activation pin\n");
        gpio_direction_output(30, 0);

        /* Activate USB */
        //    gpio_direction_output(4, 1);

        /* pads init */
        p6i_set_pads_i2c(0);
        p6i_set_pads_sdcard(25000000);

        printk("Initializing Ethernet JTAG ...\n");
        p6i_eth_on_jtag_init();

#ifdef DELOS_USE_PWM
        // init PWM :
        printk("Initializing PWM ...\n");
        parrot_init_pin(pins_init_p6i_pwms);

        /* Force PWM outputs to 0 */
        printk("Forcing PWM to 0 ...\n");
        delos_reset_pwms();

        printk("Initializing Motor protection pins ...\n");
        gpio_direction_input(DELOS_HSIS__GPIO_MOTOR_FAULT);
        //gpio_direction_output(DELOS_HSIS__GPIO_MOTOR_ENABLE, 0);  /* PB */
        gpio_direction_output(DELOS_HSIS__GPIO_RESET_MOTOR_FAULT, 1);
#endif

        platform_add_devices(p6i_devices, ARRAY_SIZE(p6i_devices));
}

MACHINE_START(PARROT_P6IDEV, "Delos SiP6 Parrot platform")
/* Maintainer: Parrot S.A. */
.phys_io    = PARROT6_UART0,
        .io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
        .boot_params    = PARROT6_DDR_BASE+0x100,
        .map_io     = p6i_map_io,
        .init_irq   = p6_init_irq,
        .timer      = &p6_timer,
        .init_machine   = p6idev_init,
        MACHINE_END

