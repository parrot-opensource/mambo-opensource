/*
 *  linux/arch/arm/mach-parrot6/powerup.c
 *
 *  Copyright (C) 2015 Parrot S.A.
 *
 * @author     florian.leprince.ext@parrot.com
 * @date       2015-01-23
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
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/mmc/host.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/reboot.h>
#include <linux/memblock.h>

/* Functions to sleep */
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include <mach/mmc.h>
#include <mach/usb.h>
#include <mach/regs-pwm-p6.h>
#include <mach/i2c.h>
#include <mach/spi.h>
#include <linux/spi/spi.h>
#include <mach/aai.h>
#include <mach/ultra_snd.h>

#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"
#include "board-sysfs.h"

#include <misc/dynamic_pinmux.h>

#include <mach/powerup_hwrev.h>

#ifndef CONFIG_GPIOLIB
#error GPIOLIB is required. Add it to configuration.
#endif

typedef enum _powerup_pcb_version_t
{
	POWERUP_PCB_00 = 0x00,	// HW 00
} powerup_pcb_version_t;

// ------------------------------------------------------------------------------------------

static void __init powerup_init(void);

static __init int sysfs_powerup_init(void);

static struct parrot_aai_platform_data aai_platform_data;

typedef struct {
	int hwrev;
	int pcbrev;
} struct_powerup_hsis;

typedef struct {
	int gpio_POWER_ON_OFF_on;
	int gpio_MCU_RST;
	int gpio_WiFi_RST;
	int gpio_INT_MAGNETO;
	int gpio_MOTOR_FAULT;
	int gpio_NRST_CAM;
	int gpio_RED_LED_LEFT;
	int gpio_RED_LED_RIGHT;
	int gpio_EN_BUZZER;
	int gpio_POWER_ON_OFF;
	int gpio_BUTTON;
	int gpio_VBUS_DETECT;
	int gpio_GREEN_LED_LEFT;
	int gpio_GREEN_LED_RIGHT;
	int gpio_INT_INERT;
	int gpio_USB_MUX_CMD;
	int gpio_CHARGE_STATUS;
	int gpio_DEV_1;
	int gpio_DEV_2;
	int gpio_DEV_3;
	int gpio_DEV_4;
} struct_powerup_hsis_gpio;

typedef struct {
	int pwm_sip6_left_motor;
	int pwm_sip6_right_motor;
} struct_powerup_hsis_pwm_sip6;

static const struct_powerup_hsis powerup_default_hsis =
{
	.hwrev = -1,
	.pcbrev = -1,
};

static const struct_powerup_hsis_pwm_sip6 powerup_hsis_pwm_sip6_default =
{
	.pwm_sip6_left_motor = -1,
	.pwm_sip6_right_motor = -1,
};

static const struct_powerup_hsis_gpio powerup_hsis_gpio_default =
{
	.gpio_MCU_RST = -1,
	.gpio_WiFi_RST = -1,
	.gpio_INT_MAGNETO = -1,
	.gpio_MOTOR_FAULT = -1,
	.gpio_NRST_CAM = -1,
	.gpio_RED_LED_LEFT = -1,
	.gpio_RED_LED_RIGHT = -1,
	.gpio_EN_BUZZER = -1,
	.gpio_GREEN_LED_LEFT = -1,
	.gpio_GREEN_LED_RIGHT = -1,
	.gpio_INT_INERT = -1,
	.gpio_USB_MUX_CMD = -1,
	.gpio_VBUS_DETECT = -1,
	.gpio_BUTTON = -1,
	.gpio_POWER_ON_OFF = -1,
	.gpio_POWER_ON_OFF_on = -1,
	.gpio_CHARGE_STATUS = -1,
	.gpio_DEV_1 = -1,
	.gpio_DEV_2 = -1,
	.gpio_DEV_3 = -1,
	.gpio_DEV_4 = -1,
};

static struct_powerup_hsis powerup_hsis = {0};
static struct_powerup_hsis_pwm_sip6 *powerup_hsis_pwm_sip6 = 	(struct_powerup_hsis_pwm_sip6 *)&powerup_hsis_pwm_sip6_default;
static struct_powerup_hsis_gpio 	*powerup_hsis_gpio = 	(struct_powerup_hsis_gpio *)&powerup_hsis_gpio_default;

// ************************************
static unsigned int pins_init_p6i_powerup[] __initdata = {
	/* gpios */
	P6I_GPIO_002,		/* MCU_RST */
	P6I_GPIO_011,		/* WiFi_RST */
	GPIO_038,		/* INT_MAGNETO */
	GPIO_039,		/* MOTOR_FAULT */
	GPIO_040,		/* WEBCAM_RST */
	GPIO_041,		/* LEFT_EYE_LED_RED */
	GPIO_043,		/* RIGHT_EYE_LED_RED */
	GPIO_045,		/* EN_BUZZER */
	GPIO_048,		/* SYS_OFF_SCK / SPI1_CLK */
	GPIO_049,		/* SD_CD */
	GPIO_050,		/* SWITCH_ON_MOSI */
	GPIO_051,		/* VBUSD_MISO */
	GPIO_052,		/* LEFT_EYE_LED_GREEN */
	GPIO_053,		/* RIGHT_EYE_LED_GREEN */
	GPIO_057,		/* INT_INERT */
	GPIO_058,		/* MB_USB_SWC */
	/* uart */
	P6I_UART1_RXTX_DEFAULT,	/* SIP6_UART1 (no RTS/CTS) */
	/* i2c */
	P6I_I2CM0_DEFAULT,
	/* sdio/mmc */
	P6I_SD0_DEFAULT,
	/* rtc pwr on */
	P6I_RTC_PWR_ON,
	0
};

static unsigned int pins_init_pwm_powerup[] __initdata = {
	/* PWM motors */
	P6I_PWM_00a,
	P6I_PWM_01a,
	0
};

static const unsigned int pins_reboot_pwm_powerup[] = {
	P6I_RTC_WKUP_1,
	P6I_RTC_WKUP_2,
	0
};

static int powerup_rst_notify_sys(struct notifier_block *this,
		unsigned long code, void *unused)
{
	parrot_init_pin(pins_reboot_pwm_powerup);
	return NOTIFY_DONE;
}

static struct notifier_block powerup_rst_notifier = {
	.notifier_call = powerup_rst_notify_sys,
};

static struct notifier_block powerup_panic_notifier = {
	.notifier_call = powerup_rst_notify_sys,
};

static struct platform_device *p6i_powerup_devices[] = {
	&p6_uart1_device,
	&p6_nand_device,
	&p6_us_device,
	&p6_aai_device,
	&p6_i2cm0_device,
	&p6_sdhci0_device,
	&p6i_usb0_device,
	/* virtual device */
	&p6_spi1_device,
	&p6_dmac_device,
	&p6_gpio
};

static struct p6_spi_config p6i_spi_controller_data = {
	.tsetupcs_ns = 10,
	.tholdcs_ns  = 10,
};

static struct spi_board_info p6i_spi_board_info[] =  {
	/* SPIDEV2.0 */
	{
		.modalias = "spidev",
		.max_speed_hz = 2000000,
		.bus_num = 1,
		.chip_select = 0,
		.platform_data = NULL,
		.controller_data = &p6i_spi_controller_data,
		.mode = SPI_MODE_0,
	}
};

static struct parrot_mmc_platform_data powerup_mmc_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.4V ~ 3.2V card Vdd only */
	.wp_pin = -1,
	.cd_pin = 49,
};

// I2C 0 :
static struct parrot5_i2cm_platform i2cm_platform_powerup_i2c0 = {
	.bus_freq	= 400*1000,		// use i2c at 400kHz
	.retries	= 1,
};

struct my_i2c_board_info
{
	struct i2c_board_info info;
	char version[I2C_NAME_SIZE];
	int busnum;
	int hwrev_mask;
};

static struct my_i2c_board_info powerup_i2c_devices[] = {
	{ /* IMU */
		.info = {
			I2C_BOARD_INFO("mpu6050", 0x68),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<POWERUP_HW_00)
	},
	{ /* Magneto */
		.info = {
			I2C_BOARD_INFO("ak8963c", 0x0d),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<POWERUP_HW_00)
	},
	{ /* Pressure/Temperature */
		.info = {
			I2C_BOARD_INFO("ms5607", 0x77),
			.irq = -1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<POWERUP_HW_00)
	}
};

static void  __init init_aai()
{
	printk(KERN_INFO "Initializing AAI ...\n");

	aai_platform_data.sync_freq = 48000;

	aai_platform_data.i2s_bit_clock_control = 0;
	aai_platform_data.i2s_master_clock_control = 0;

	p6_aai_device.dev.platform_data = &aai_platform_data;
}


/*
 * This function reset the P6_PWM_RATIOxx registers
 */
static __init void powerup_reset_pwms(void)
{
	char __iomem *pwm_regbase;
	const int pwm_ctrl_enable_all = P6_PWM_START00 | P6_PWM_START01;
	unsigned int ntimer;

	struct clk *pwm_clk;

	printk(KERN_INFO "Powerup Reset PWMs\n");

	pwm_clk = clk_get(NULL, "pwm");

	clk_enable(pwm_clk);

	pwm_regbase = ioremap(PARROT6_PWM, 256);

	/* Force all ratios to zero */
	for (ntimer=0; ntimer<2; ntimer++)
	{
		__raw_writel(0, pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	}
	/* Activate the PWM output to generate the 0 output */
	__raw_writel(pwm_ctrl_enable_all, pwm_regbase + P6_PWM_CTL);

	iounmap(pwm_regbase);

	clk_disable(pwm_clk);
}


/******************************/
/* Ramoops and persistent ram */
/******************************/

#ifdef CONFIG_RAMOOPS
/* reserved RAM used to store oops/panic logs */
#define RAMOOPS_SIZE  SZ_512K

struct ramoops_platform_data {
	unsigned long   mem_size;
	unsigned long   mem_address;
	int	            dump_oops;
};

static struct ramoops_platform_data ramoops_data = {
	/* size of the persistent RAM allocated to ramoop */
	.mem_size               = RAMOOPS_SIZE,
	/* set to 1 to dump oopses, 0 to only dump panics */
	.dump_oops              = 1,
};

static struct platform_device ramoops_dev = {
	.name = "ramoops",
	.dev = {
		.platform_data = &ramoops_data,
	},
};

static void __init powerup_reserve_mem(void)
{
	ramoops_data.mem_address = meminfo.bank[0].start +
		meminfo.bank[0].size - RAMOOPS_SIZE;

	memblock_reserve(&ramoops_data.mem_address,
			&ramoops_data.mem_size);
}

static void __init powerup_ramoops_init(void)
{
	int ret;

	/* Need CONFIG_RAMOOPS in kernel config */
	ret = platform_device_register(&ramoops_dev);
	if (ret)
		printk("unable to register ramoops platform device\n");
	else
		printk("ramoops registered with memory from %08lx-%08lx\n",
				(long)ramoops_data.mem_address,
				(long)(ramoops_data.mem_address +
					ramoops_data.mem_size - 1));

}
#endif

/* PowerUp pcb */
static const struct_powerup_hsis_pwm_sip6 powerup_hsis_pwm_sip6_pcb00 =
{
	.pwm_sip6_left_motor = 0,	/* PWM_SIP6_0 */
	.pwm_sip6_right_motor = 1,	/* PWM_SIP6_1 */
};

static const struct_powerup_hsis_gpio powerup_hsis_gpio_pcb00 = {
	.gpio_POWER_ON_OFF_on = 0,
	.gpio_MCU_RST = 2,
	.gpio_WiFi_RST = 11,
	.gpio_INT_MAGNETO = 38,		/* INT_MAGNETO */
	.gpio_MOTOR_FAULT = 39,		/* nFAULT_WHEEL */
	.gpio_NRST_CAM = 40,		/* WEBCAM_RST */
	.gpio_RED_LED_LEFT = 41,	/* LEFT_EYE_LED_RED */
	.gpio_RED_LED_RIGHT = 43,	/* RIGHT_EYE_LED_RED */
	.gpio_EN_BUZZER = 45,		/* EN_BUZZER */
	.gpio_POWER_ON_OFF = 48,	/* SYS_OFF_SCK */
	.gpio_BUTTON = 50,			/* SWITCH_ON_MOSI */
	.gpio_VBUS_DETECT = 51,		/* VBUSD_MISO */
	.gpio_GREEN_LED_LEFT = 52,	/* LEFT_EYE_LED_GREEN */
	.gpio_GREEN_LED_RIGHT = 53,	/* RIGHT_EYE_LED_GREEN */
	.gpio_INT_INERT = 57,		/* INT_INERT */
	.gpio_USB_MUX_CMD = 58,		/* MB_USB_SWC */
	.gpio_CHARGE_STATUS = -1,	/* Not Connected */
	.gpio_DEV_1 = -1,
	.gpio_DEV_2 = -1,
	.gpio_DEV_3 = -1,
	.gpio_DEV_4 = -1,
};

static int pic_programming_mode_on[] = {
	P6I_SPI1_MISO,
	P6I_SPI1_MOSI,
	P6I_SPI1_CLK,
	0,
};

static int pic_programming_mode_off[] = {
	GPIO_051,
	GPIO_050,
	GPIO_048,
	0,
};

static struct dynamic_pinmux_platform_data dynamic_pinmux_pic_prog_pdata = {
	.name = "pic_programming",
	.mux_mode_0 = pic_programming_mode_off,
	.mux_mode_1 = pic_programming_mode_on,
	.current_mode = 0,
};

static struct platform_device p6_dynamic_pinmux_pic_prog = {
	.name          = "dynamic_pinmux",
	.id            = 0,
	.dev = {
		.platform_data = &dynamic_pinmux_pic_prog_pdata,
	},
};

static void powerup_power_off(void)
{
	int gpio = powerup_hsis_gpio->gpio_POWER_ON_OFF;
	int val;
	val = gpio_get_value(gpio);
	gpio_set_value(gpio, val ? 0 : 1);
}

static void __init powerup_init(void)
{
	int i;

	sip6_init(0);
	platform_device_register(&user_gpio);

	powerup_hsis = powerup_default_hsis;
	powerup_hsis.pcbrev = parrot_board_rev();

#ifdef CONFIG_RAMOOPS
	powerup_ramoops_init();
#endif
	printk(KERN_INFO " ==== PowerUp (PCB rev : %d) ====\n", powerup_hsis.pcbrev);

	switch (powerup_hsis.pcbrev) {
		default:
			printk(KERN_INFO "WARNING : unknown and/or not supported board version\n");
			printk(KERN_INFO "value of pcb version : %x\n", powerup_hsis.pcbrev);
			printk(KERN_INFO "We're assuming that hardware version of this board is similar to the last hardware version managed by this BSP\n");
			/* There is no break, it's normal... */
		case POWERUP_PCB_00:
			powerup_hsis.hwrev = POWERUP_HW_00;
			printk(KERN_INFO "hardware version 00 used (%d detected)\n", powerup_hsis.pcbrev);
			powerup_hsis_gpio = (struct_powerup_hsis_gpio *)&powerup_hsis_gpio_pcb00;
			powerup_hsis_pwm_sip6 = (struct_powerup_hsis_pwm_sip6 *)&powerup_hsis_pwm_sip6_pcb00;
			parrot_init_pin(pins_init_p6i_powerup);
			platform_device_register(&p6_dynamic_pinmux_pic_prog);

			// Pull Up on UART1 RX:
			p6i_set_pads_uart1_pullup();
			break;
	}

	// OUTPUT HIGH => Motors blocked.
	if (powerup_hsis_gpio->gpio_MOTOR_FAULT >= 0) {
		p6i_export_gpio(powerup_hsis_gpio->gpio_MOTOR_FAULT, GPIOF_OUT_INIT_HIGH, "MOTOR_FAULT", 0);
	}

	powerup_reset_pwms();
	parrot_init_pin(pins_init_pwm_powerup);
	register_reboot_notifier(&powerup_rst_notifier);
	atomic_notifier_chain_register(&panic_notifier_list,
			&powerup_panic_notifier);

	/* ON/OFF P6i output GPIO */
	if (powerup_hsis_gpio->gpio_POWER_ON_OFF >= 0) {
		if (powerup_hsis_gpio->gpio_POWER_ON_OFF_on==1)
			p6i_export_gpio(powerup_hsis_gpio->gpio_POWER_ON_OFF, GPIOF_OUT_INIT_HIGH, "POWER_ON_OFF", 0);
		else
			p6i_export_gpio(powerup_hsis_gpio->gpio_POWER_ON_OFF, GPIOF_OUT_INIT_LOW, "POWER_ON_OFF", 0);

		/* Enable power off if product has a GPIO for that. */
		pm_power_off = powerup_power_off;
        }

	// Turn On and export RED LEDs
	if (powerup_hsis_gpio->gpio_RED_LED_LEFT >= 0)
		p6i_export_gpio(powerup_hsis_gpio->gpio_RED_LED_LEFT, GPIOF_OUT_INIT_HIGH, "RED_LED_LEFT", 0);
	if (powerup_hsis_gpio->gpio_RED_LED_RIGHT >= 0)
		p6i_export_gpio(powerup_hsis_gpio->gpio_RED_LED_RIGHT, GPIOF_OUT_INIT_HIGH, "RED_LED_RIGHT", 0);

	// Turn Off and export GREEN LEDs
	if (powerup_hsis_gpio->gpio_GREEN_LED_LEFT >= 0)
		p6i_export_gpio(powerup_hsis_gpio->gpio_GREEN_LED_LEFT, GPIOF_OUT_INIT_LOW, "GREEN_LED_LEFT", 0);
	if (powerup_hsis_gpio->gpio_GREEN_LED_RIGHT >= 0)
		p6i_export_gpio(powerup_hsis_gpio->gpio_GREEN_LED_RIGHT, GPIOF_OUT_INIT_LOW, "GREEN_LED_RIGHT", 0);

	if (powerup_hsis_gpio->gpio_CHARGE_STATUS >= 0) {
		gpio_interrupt_register(powerup_hsis_gpio->gpio_CHARGE_STATUS, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(powerup_hsis_gpio->gpio_CHARGE_STATUS, GPIOF_IN, "CHARGE_STATUS", 0);
	}

	if (powerup_hsis_gpio->gpio_BUTTON >= 0) {
		gpio_interrupt_register(powerup_hsis_gpio->gpio_BUTTON, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(powerup_hsis_gpio->gpio_BUTTON, GPIOF_IN, "BUTTON", 0);
	}

	/* Interrupt pin for pal_gpio_irq */
	if (powerup_hsis_gpio->gpio_INT_INERT >= 0) {
		gpio_interrupt_register(powerup_hsis_gpio->gpio_INT_INERT, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(powerup_hsis_gpio->gpio_INT_INERT, GPIOF_IN, "INT_INERT", 1);
	}

	/* Interrupt pin for pal_gpio_irq */
	if (powerup_hsis_gpio->gpio_INT_MAGNETO >= 0) {
		gpio_interrupt_register(powerup_hsis_gpio->gpio_INT_MAGNETO, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(powerup_hsis_gpio->gpio_INT_MAGNETO, GPIOF_IN, "INT_MAGNETO", 1);
	}

	/* Speaker amplifier */
	if (powerup_hsis_gpio->gpio_EN_BUZZER >= 0) {
		p6i_export_gpio(powerup_hsis_gpio->gpio_EN_BUZZER, GPIOF_OUT_INIT_LOW, "EN_BUZZER", 0);
	}

	/* Etron /reset */
	if (powerup_hsis_gpio->gpio_NRST_CAM >= 0)
		p6i_export_gpio(powerup_hsis_gpio->gpio_NRST_CAM, GPIOF_OUT_INIT_LOW, "NRST_CAM", 0);

	/* INIT AAI */
	init_aai();

	/* Detection of plug/unplug usb */
	if (powerup_hsis_gpio->gpio_VBUS_DETECT >= 0) {
		gpio_interrupt_register(powerup_hsis_gpio->gpio_VBUS_DETECT, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(powerup_hsis_gpio->gpio_VBUS_DETECT, GPIOF_IN, "VBUS_DETECT", 0);
	}

	// Wifi Reset
	if (powerup_hsis_gpio->gpio_WiFi_RST >= 0) {
		p6i_export_gpio(powerup_hsis_gpio->gpio_WiFi_RST, GPIOF_OUT_INIT_LOW, "WIFI_RST", 0);
	}

	// MCU Reset
	if (powerup_hsis_gpio->gpio_MCU_RST >= 0) {
		p6i_export_gpio(powerup_hsis_gpio->gpio_MCU_RST, GPIOF_OUT_INIT_LOW, "MCU_RST", 0);
		printk("Now registering p6i_spi_board_info, nb spi = %d\n", ARRAY_SIZE(p6i_spi_board_info));
		spi_register_board_info(p6i_spi_board_info, ARRAY_SIZE(p6i_spi_board_info));
	}

	// Init USB switch (0: USB Connector, 1: USB hub)
	if (powerup_hsis_gpio->gpio_USB_MUX_CMD >= 0) {
		if (parrot_force_usb_device)
			p6i_export_gpio(powerup_hsis_gpio->gpio_USB_MUX_CMD, GPIOF_OUT_INIT_LOW, "USB_MUX_CMD", 0);
		else
			p6i_export_gpio(powerup_hsis_gpio->gpio_USB_MUX_CMD, GPIOF_OUT_INIT_HIGH, "USB_MUX_CMD", 0);
	}

	if (powerup_hsis_gpio->gpio_DEV_1 >= 0) {
		gpio_interrupt_register(powerup_hsis_gpio->gpio_DEV_1, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(powerup_hsis_gpio->gpio_DEV_1, GPIOF_IN, "DEV_1", 1);
	}
	if (powerup_hsis_gpio->gpio_DEV_2 >= 0) {
		gpio_interrupt_register(powerup_hsis_gpio->gpio_DEV_2, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(powerup_hsis_gpio->gpio_DEV_2, GPIOF_IN, "DEV_2", 1);
	}
	if (powerup_hsis_gpio->gpio_DEV_3 >= 0) {
		gpio_interrupt_register(powerup_hsis_gpio->gpio_DEV_3, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(powerup_hsis_gpio->gpio_DEV_3, GPIOF_IN, "DEV_3", 1);
	}
	if (powerup_hsis_gpio->gpio_DEV_4>= 0) {
		gpio_interrupt_register(powerup_hsis_gpio->gpio_DEV_4, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(powerup_hsis_gpio->gpio_DEV_4, GPIOF_IN, "DEV_4", 1);
	}

	// I2C :
	p6_i2cm0_device.dev.platform_data = &i2cm_platform_powerup_i2c0;
	p6i_set_pads_i2c(0);
	p6i_set_i2c_drive_strength(0, 0x0);

	// PowerUp : eMMC - SDIO
	p6i_set_pads_sdcard(50000000);	// Set SDIO frequency at 50MHz & do set drive strength

	/* set drive strengh for usb */
	p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_PREEMDEPTH,0x01);
	p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_ENPRE,0x01);
	p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVAMPLITUDE,0x03);
	p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVSLOPE,0x0f);

	p6_sdhci0_device.dev.platform_data = &powerup_mmc_platform_data;

	platform_add_devices( p6i_powerup_devices, ARRAY_SIZE(p6i_powerup_devices) );

	p6i_export_uart_hw_infos(1, 0, "");

	for (i = 0; i < ARRAY_SIZE(powerup_i2c_devices); i++)
	{
		struct my_i2c_board_info *ptr=&powerup_i2c_devices[i];
		if (ptr->hwrev_mask & (1<<powerup_hsis.hwrev))
			p6i_export_i2c_hw_infos(ptr->busnum,
					ptr->info.addr,
					ptr->version,
					ptr->info.type);
	}

	sysfs_powerup_init();
}

#define CREATE_BSP_FILE_INT(_name, _var) \
	static ssize_t _name##_show(struct kobject *kobj, \
			struct kobj_attribute *attr, char *buf) \
{ \
	return sprintf(buf, "%d\n", _var); \
} \
static ssize_t _name##_store(struct kobject *kobj, \
		struct kobj_attribute *attr, \
		const char *buf, size_t count) \
{ \
	return count; \
} \
static struct kobj_attribute _name##_attr = \
__ATTR(_name, 0644, _name##_show, _name##_store)

#define MAP_BSP_VALUE_INT_NP(x, _hsis) CREATE_BSP_FILE_INT(x, _hsis.x)
#define MAP_BSP_VALUE_INT(x, _hsis) CREATE_BSP_FILE_INT(x, _hsis->x)

#define CREATE_BSP_FILE_STRING(_name, _var) \
	static ssize_t _name##_show(struct kobject *kobj, \
			struct kobj_attribute *attr, char *buf) \
{ \
	return sprintf(buf, "%s\n", _var); \
} \
static ssize_t _name##_store(struct kobject *kobj, \
		struct kobj_attribute *attr, \
		const char *buf, size_t count) \
{ \
	return count; \
} \
static struct kobj_attribute _name##_attr = \
__ATTR(_name, 0644, _name##_show, _name##_store)

#define MAP_BSP_VALUE_STRING(x, _hsis) CREATE_BSP_FILE_STRING(x, _hsis.x)

#define BSP_ATTR(_name) &_name##_attr.attr

MAP_BSP_VALUE_INT_NP(pcbrev, powerup_hsis);
MAP_BSP_VALUE_INT_NP(hwrev, powerup_hsis);

MAP_BSP_VALUE_INT(pwm_sip6_left_motor, powerup_hsis_pwm_sip6);
MAP_BSP_VALUE_INT(pwm_sip6_right_motor, powerup_hsis_pwm_sip6);


MAP_BSP_VALUE_INT(gpio_INT_INERT, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_INT_MAGNETO, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_VBUS_DETECT, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_BUTTON, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_NRST_CAM, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_RED_LED_LEFT, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_RED_LED_RIGHT, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_GREEN_LED_LEFT, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_GREEN_LED_RIGHT, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_EN_BUZZER, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_MOTOR_FAULT, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_POWER_ON_OFF, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_POWER_ON_OFF_on, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_USB_MUX_CMD, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_WiFi_RST, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_MCU_RST, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_DEV_1, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_DEV_2, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_DEV_3, powerup_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_DEV_4, powerup_hsis_gpio);

static const struct attribute *powerup_bsp_attrs[] = {
	BSP_ATTR(hwrev),
	BSP_ATTR(pcbrev),
	BSP_ATTR(pwm_sip6_left_motor),
	BSP_ATTR(pwm_sip6_right_motor),
	BSP_ATTR(gpio_INT_INERT),
	BSP_ATTR(gpio_INT_MAGNETO),
	BSP_ATTR(gpio_VBUS_DETECT),
	BSP_ATTR(gpio_BUTTON),
	BSP_ATTR(gpio_NRST_CAM),
	BSP_ATTR(gpio_RED_LED_LEFT),
	BSP_ATTR(gpio_RED_LED_RIGHT),
	BSP_ATTR(gpio_GREEN_LED_LEFT),
	BSP_ATTR(gpio_GREEN_LED_RIGHT),
	BSP_ATTR(gpio_EN_BUZZER),
	BSP_ATTR(gpio_MOTOR_FAULT),
	BSP_ATTR(gpio_POWER_ON_OFF),
	BSP_ATTR(gpio_POWER_ON_OFF_on),
	BSP_ATTR(gpio_USB_MUX_CMD),
	BSP_ATTR(gpio_WiFi_RST),
	BSP_ATTR(gpio_MCU_RST),
	BSP_ATTR(gpio_DEV_1),
	BSP_ATTR(gpio_DEV_2),
	BSP_ATTR(gpio_DEV_3),
	BSP_ATTR(gpio_DEV_4),
	NULL
};

static const struct attribute_group powerup_bsp_attr_group = {
	.attrs = (struct attribute **)powerup_bsp_attrs,
};

static __init int sysfs_powerup_init(void)
{
	struct kobject *powerup_kobject;
	int ret;

	printk(KERN_INFO "Powerup: exporting HSIS to userspace in /sys/kernel/hsis");

	powerup_kobject = kobject_create_and_add("hsis", kernel_kobj);
	if (!powerup_kobject)
		return -ENOMEM;

	ret = sysfs_create_group(powerup_kobject, &powerup_bsp_attr_group);
	if (ret) {
		kobject_put(powerup_kobject);
		return ret;
	}

	return 0;
}

MACHINE_START(PARROT_POWERUP, "PowerUp sip6 board")
/* Maintainer: Parrot S.A. */
#ifdef CONFIG_RAMOOPS
	.reserve    = powerup_reserve_mem,
#endif
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io	 = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer	  = &p6_timer,
	.init_machine   = powerup_init,
MACHINE_END

MACHINE_START(PARROT_JPSUMO_EVO, "PowerUp sip6 board")
	/* Maintainer: Parrot S.A. */
#ifdef CONFIG_RAMOOPS
	.reserve    = powerup_reserve_mem,
#endif
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io	 = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer	  = &p6_timer,
	.init_machine   = powerup_init,
MACHINE_END


