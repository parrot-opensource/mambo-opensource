/*
 *  linux/arch/arm/mach-parrot6/delos.c
 *
 *  Copyright (C) 2015 Parrot S.A.
 *
 * @author     didier.leymarie.ext@parrot.com
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
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/pm.h>
#include <linux/mmc/host.h>
#include <linux/i2c.h>
#include <linux/reboot.h>
#include <linux/clk.h>
#include <linux/memblock.h>

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

#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"
#include "board-sysfs.h"

#include <mach/i2c.h>
#include <mach/spi.h>
#include <linux/spi/spi.h>

#include <mach/delos.h>
#include <mach/delos_hwrev.h>

#include <mach/regs-rtc-p6i.h>

#include <misc/dynamic_pinmux.h>
#include <../drivers/parrot/i2c/smsc-82514-usb-hub.h>
#include <../drivers/parrot/spi/mcu_minidrones3.h>

#include <../drivers/parrot/pressure/lps22hb.h>

#ifndef CONFIG_GPIOLIB
#error GPIOLIB is required. Add it to configuration.
#endif

/* *************** Globals ... ********************* */

typedef enum _delos_type_t
{
	DELOS_CLASSIC = 0,
	DELOS_EVO,
	DELOS_V3,
	DELOS_WINGX,
} delos_type_t;

typedef struct {
	int gpio_BT_ENABLE;
	int gpio_INT_INERT;
	int gpio_INT_BARO;
	int gpio_VBUS_DETECT;
	int gpio_BUTTON;
	int gpio_US_POWER;
	int gpio_HRESET_CAM;
	int gpio_MOTOR_FAULT;
	int gpio_POWER_ON_OFF;
	int gpio_RED_LED_LEFT;
	int gpio_RED_LED_RIGHT;
	int gpio_GREEN_LED_LEFT;
	int gpio_GREEN_LED_RIGHT;
	int gpio_PWMGEN_nOE;
	int gpio_USB_MUX_CMD;
	int gpio_POWER_ON_OFF_on;
	int gpio_MCU_RST;
	int gpio_MCU_IT;
	int gpio_BT_ANTENNA_EXTERN;
	int gpio_BT_ANTENNA_EVO;

} struct_delos_hsis_gpio;

typedef struct {
	int pwm_gen_headlight_left;
	int pwm_gen_headlight_right;
	int pwm_gen_outdrv;
	int pwm_gen_headlight_complement;
} struct_delos_hsis_pwm_gen;

typedef struct {
	int pwm_sip6_motor_1;
	int pwm_sip6_motor_2;
	int pwm_sip6_motor_3;
	int pwm_sip6_motor_4;
} struct_delos_hsis_pwm_sip6;

typedef struct {
	char *model;
	int hwrev;
	int pcbrev;
} struct_delos_hsis;


/* *************** Prototypes ********************* */
static __init void delos_init(delos_type_t delos_type);
static __init int sysfs_delos_init(void);



/* ************************************************ */
/**
 * Array containing harware information to export to userspace
 */

static const struct_delos_hsis delos_hsis_default =
{
	.model = "Unknown",
	.hwrev = -1,
	.pcbrev = -1,
};

static const struct_delos_hsis_pwm_gen delos_hsis_pwm_gen_default =
{
	.pwm_gen_headlight_left = -1,
	.pwm_gen_headlight_right = -1,
	.pwm_gen_outdrv = -1,
	.pwm_gen_headlight_complement = -1,
};

static const struct_delos_hsis_pwm_sip6 delos_hsis_pwm_sip6_default =
{
	.pwm_sip6_motor_1 = 2,
	.pwm_sip6_motor_2 = 1,
	.pwm_sip6_motor_3 = 3,
	.pwm_sip6_motor_4 = 0,
};

static const struct_delos_hsis_gpio delos_hsis_gpio_default =
{
	.gpio_BT_ENABLE = -1,
	.gpio_INT_INERT = -1,
	.gpio_VBUS_DETECT = -1,
	.gpio_BUTTON = -1,
	.gpio_US_POWER = -1,
	.gpio_HRESET_CAM = -1,
	.gpio_MOTOR_FAULT = -1,
	.gpio_POWER_ON_OFF = -1,
	.gpio_RED_LED_LEFT = -1,
	.gpio_RED_LED_RIGHT = -1,
	.gpio_GREEN_LED_LEFT = -1,
	.gpio_GREEN_LED_RIGHT = -1,
	.gpio_PWMGEN_nOE = -1,
	.gpio_USB_MUX_CMD = -1,
	.gpio_POWER_ON_OFF_on = -1,
	.gpio_BT_ANTENNA_EXTERN = -1,
	.gpio_BT_ANTENNA_EVO = -1,
	.gpio_INT_BARO = -1
};

static struct_delos_hsis delos_hsis = {0};
static struct_delos_hsis_pwm_gen *delos_hsis_pwm_gen = (struct_delos_hsis_pwm_gen *)&delos_hsis_pwm_gen_default;
static struct_delos_hsis_pwm_sip6 *delos_hsis_pwm_sip6 = (struct_delos_hsis_pwm_sip6 *)&delos_hsis_pwm_sip6_default;
static struct_delos_hsis_gpio *delos_hsis_gpio = (struct_delos_hsis_gpio *)&delos_hsis_gpio_default;

// ************************************

static unsigned int pins_init_p6i[] __initdata = {
	/* uart */
	P6I_UART1_RXTX_DEFAULT,
	P6I_I2CM0_DEFAULT,
	P6I_GPIO_005, /* activate bluetooth chip */
	P6I_SPI1_MOSI,	/* PWM_US	: Init MOSI spi pin for ultrasound driver */
	P6I_USB_PWR_ON, /* usb pwr on */
	0,
};

static unsigned int pins_init_pwm_classic_evo[] __initdata = {
	/* PWM motors */
	P6I_PWM_00a,
	P6I_PWM_01a,
	P6I_PWM_02b,
	P6I_PWM_03a,
	0,
};

static unsigned int pins_init_pwm_v3[] __initdata = {
	/* PWM motors */
	P6I_PWM_00a,
	P6I_PWM_01a,
	P6I_PWM_02c,
	P6I_PWM_03a,
	0,
};

static unsigned int pins_init_p6i_hw05[] __initdata = {
	DELOS_HSIS__GPIOPIN_HRESET_CAM, /* Enable (1) or Disable (0) USB Camera */
	DELOS_HSIS__GPIOPIN_US_POWER, /* When US_POWER is set, amplitude of ultrasound "pwm" moves from 3.3V to 0.8V */
	DELOS_HSIS__GPIOPIN_ONOFF_HW05, /*Control of power device */
	DELOS_HSIS__GPIOPIN_BUTTON_HW05, /* Input Button to monitor : 0 when this button is released; 1 when someone presses it */
	DELOS_HSIS__GPIOPIN_MOTOR_FAULT_HW05, /* Motors are "blocked" when output MOTOR_FAULT is set */
	DELOS_HSIS__GPIOPIN_RED_LED_L_HW05, /* Red left led gpio */
	DELOS_HSIS__GPIOPIN_GREEN_LED_R_HW05, /* Green right led gpio */
	DELOS_HSIS__GPIOPIN_GREEN_LED_L_HW05, /* Green left led gpio */
	DELOS_HSIS__GPIOPIN_RED_LED_R_HW05, /* Red right led gpio */
	DELOS_HSIS__GPIOPIN_INT_INERT_HW05, /* Input data ready signal of sensor mpu6050 */
	DELOS_HSIS__GPIOPIN_VBUS_HW05, /*When delos is connected with usb interface, vbus is set*/
	0,
};

static unsigned int pins_init_p6i_hw07[] __initdata = {
	DELOS_HSIS__GPIOPIN_HRESET_CAM, /* Enable (1) or Disable (0) USB Camera */
	DELOS_HSIS__GPIOPIN_US_POWER, /* When US_POWER is set, amplitude of ultrasound "pwm" moves from 3.3V to 0.8V */
	DELOS_HSIS__GPIOPIN_ONOFF_HW07, /*Control of power device */
	DELOS_HSIS__GPIOPIN_BUTTON_HW07, /* Input Button to monitor : 0 when this button is released; 1 when someone presses it */
	DELOS_HSIS__GPIOPIN_MOTOR_FAULT_HW07, /* Motors are "blocked" when output MOTOR_FAULT is set */
	DELOS_HSIS__GPIOPIN_RED_LED_L_HW07, /* Red left led gpio */
	DELOS_HSIS__GPIOPIN_GREEN_LED_R_HW07, /* Green right led gpio */
	DELOS_HSIS__GPIOPIN_GREEN_LED_L_HW07, /* Green left led gpio */
	DELOS_HSIS__GPIOPIN_RED_LED_R_HW07, /* Red right led gpio */
	DELOS_HSIS__GPIOPIN_INT_INERT_HW07, /* Input data ready signal of sensor mpu6050 */
	DELOS_HSIS__GPIOPIN_VBUS_HW07, /*When delos is connected with usb interface, vbus is set*/
	0,
};

static unsigned int pins_init_delos_evo_hw00[] __initdata = {
	DELOS_HSIS__GPIOPIN_HRESET_CAM, /* Enable (1) or Disable (0) USB Camera */
	DELOS_HSIS__GPIOPIN_US_POWER, /* When US_POWER is set, amplitude of ultrasound "pwm" moves from 3.3V to 0.8V */
	DELOS_HSIS_EVO__GPIOPIN_ONOFF_HW00, /*Control of power device */
	DELOS_HSIS_EVO__GPIOPIN_BUTTON_HW00, /* Input Button to monitor : 0 when this button is released; 1 when someone presses it */
	DELOS_HSIS_EVO__GPIOPIN_MOTOR_FAULT_HW00, /* Motors are "blocked" when output MOTOR_FAULT is set */
	DELOS_HSIS_EVO__GPIOPIN_RED_LED_L_HW00, /* Red left led gpio */
	DELOS_HSIS_EVO__GPIOPIN_GREEN_LED_R_HW00, /* Green right led gpio */
	DELOS_HSIS_EVO__GPIOPIN_GREEN_LED_L_HW00, /* Green left led gpio */
	DELOS_HSIS_EVO__GPIOPIN_RED_LED_R_HW00, /* Red right led gpio */
	DELOS_HSIS_EVO__GPIOPIN_INT_INERT_HW00, /* Input data ready signal of sensor mpu6050 */
	DELOS_HSIS_EVO__GPIOPIN_VBUS_DETECT_HW00, /*When delos is connected with usb interface, vbus_detect is set*/
	DELOS_HSIS_EVO__GPIOPIN_USB_MUX_CMD,
	DELOS_HSIS_EVO__GPIOPIN_NLED_DRIVER_OE,
	0,
};

static unsigned int pins_init_delos_evo_hw01[] __initdata = {
	GPIO_058,	/* HRESET_CAM		: Enable (1) or Disable (0) USB Camera */
	GPIO_056,	/* US_POWER		: When US_POWER is set, amplitude of ultrasound "pwm" moves from 3.3V to 0.8V */
	GPIO_030,	/* SYS_OFF__CLK		: Control of power device */
	GPIO_032,	/* SWITCH_ON__MOSI	: Input Button to monitor : 0 when this button is released; 1 when someone presses it */
	GPIO_039,	/* MOTOR_FAULT		: Motors are "blocked" when output MOTOR_FAULT is set */
	GPIO_048,	/* RED_L		: Red left led, active high led signal */
	GPIO_031,	/* GREEN_R		: Green right led gpio */
	GPIO_049,	/* GREEN_L		: Green left led, active high led signal */
	GPIO_051,	/* RED_R		: Red right led, active high led signal */
	GPIO_052,	/* INT_INERT		: Input data ready signal of sensor mpu6050 */
	GPIO_033,	/* VBUSD_MISO		: When delos is connected with usb interface, vbus_detect is set */
//	GPIO_034,	/* CHRG_STATUS		: NC */
	GPIO_055,	/* USB_SW_CMD		: USB switch select */
	GPIO_044,	/* RESET_TINY		: AT-tiny reset (active high) */
	0,
};

static unsigned int pins_init_delos_v3_hw00[] __initdata = {
	GPIO_058,	/* HRESET_CAM		: Enable (1) or Disable (0) USB Camera */
//	GPIO_056,	/* US_POWER		: When US_POWER is set, amplitude of ultrasound "pwm" moves from 3.3V to 0.8V */

	/* SPI for communication between SIP6 and Power Management At-tiny */
	P6I_SPI2_MISOb,	/* SPI_MISO		: At-tiny spi MISO */
	P6I_SPI2_MOSIb,	/* SPI_MOSI		: At-tiny spi MOSI */
	P6I_SPI2_CLKb,	/* SPI_CLK		: At-tiny spi clk */

//	/* MOTOR_FAULT				: NO MORE MOTOR FAULT ON THIS VERSION. */
	GPIO_048,	/* RED_L		: Red left led, active high led signal */
	GPIO_031,	/* GREEN_R		: Green right led gpio */
	GPIO_049,	/* GREEN_L		: Green left led, active high led signal */
	GPIO_051,	/* RED_R		: Red right led, active high led signal */
	GPIO_052,	/* INT_INERT		: Input data ready signal of sensor mpu6050 */
	GPIO_055,	/* USB_SW_CMD		: USB switch select */
	GPIO_044,	/* RESET_TINY		: AT-tiny reset (active high) */
	GPIO_056,	/* MCU_IT		: IRQ from power management AT-tiny to inform about new infos. */
	GPIO_053,	/* BT_SW_CTRL_IO2	: BT antenna external wire*/
	GPIO_057,	/* BT_SW_CTRL_IO1	: BT antenna delos evo*/
	0,
};

static unsigned int pins_init_delos_v3_hw01[] __initdata = {
	GPIO_058,	/* HRESET_CAM		: Enable (1) or Disable (0) USB Camera */
//	GPIO_056,	/* US_POWER		: When US_POWER is set, amplitude of ultrasound "pwm" moves from 3.3V to 0.8V */

	/* SPI for communication between SIP6 and Power Management At-tiny */
	P6I_SPI2_MISOb,	/* SPI_MISO		: At-tiny spi MISO */
	P6I_SPI2_MOSIb,	/* SPI_MOSI		: At-tiny spi MOSI */
	P6I_SPI2_CLKb,	/* SPI_CLK		: At-tiny spi clk */

//	/* MOTOR_FAULT				: NO MORE MOTOR FAULT ON THIS VERSION. */
	GPIO_048,	/* RED_L		: Red left led, active high led signal */
	GPIO_031,	/* GREEN_R		: Green right led gpio */
	GPIO_049,	/* GREEN_L		: Green left led, active high led signal */
	GPIO_051,	/* RED_R		: Red right led, active high led signal */
	GPIO_052,	/* INT_INERT		: Input data ready signal of sensor mpu6050 */
	GPIO_055,	/* USB_SW_CMD		: USB switch select */
	GPIO_044,	/* RESET_TINY		: AT-tiny reset (active high) */
	GPIO_056,	/* MCU_IT		: IRQ from power management AT-tiny to inform about new infos. */
	GPIO_038,       /* INT_BARO             : IRQ from lps22hb barometer */
	0,
};

static const unsigned int pins_reboot_p6i[] = {
	P6I_RTC_WKUP_1,
	P6I_RTC_WKUP_2,
	P6I_RTC_OUT_1,
	0,
};

static int delos_rst_notify_sys(struct notifier_block *this,
		unsigned long code, void *unused)
{
	parrot_init_pin(pins_reboot_p6i);
	return 0;
}

static struct notifier_block delos_rst_notifier = {
	.notifier_call = delos_rst_notify_sys,
};

static struct platform_device *p6i_devices[] __initdata = {
	&p6_uart0_device,/* BT*/
	&p6_uart1_device, /* uart console */
	&p6_nand_device,
	&p6_i2cm0_device, /* i2c for reading sensor datas and for configuring camera */
	&p6i_usb0_device,
	&p6_aai_device,
	&p6_us_device,
	&p6_spi2_device,
	&p6_dmac_device,
	&p6_gpio,
};

static struct parrot5_i2cm_platform i2cm_platform_delos = {
	/*i2c frequency at 400kHz*/
	.bus_freq	= 400*1000,
	.retries        = 1,
};

static struct smsc82514_pdata hub_init = {
	.us_port   = DS_HIGH,
	.ds_port_1 = DS_HIGH,
	.ds_port_2 = DS_HIGH,
	.reset_pin = 0,		/* No Reset Pin */
};
static struct lps22_prs_platform_data baro_init = {
	.poll_interval	= 13,
	.min_interval	= LPS22_PRS_MIN_POLL_PERIOD_MS,
};



struct my_i2c_board_info
{
	struct i2c_board_info info;
	char version[I2C_NAME_SIZE];
	int busnum;
	int hwrev_mask;
};

static struct my_i2c_board_info __initdata delos_i2c_devices[] = {
	{ /* Pressure/Temperature : MS5607 */
		.info = {
			I2C_BOARD_INFO("ms5607", 0x77),
			.irq = -1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<DELOS_CLASSIC_HW_07)|
			(1<<DELOS_CLASSIC_HW_06)|
			(1<<DELOS_CLASSIC_HW_05)|
			(1<<DELOS_CLASSIC_HW_04)|
			(1<<DELOS_CLASSIC_HW_03)|
			(1<<DELOS_CLASSIC_HW_02)|
			(1<<DELOS_CLASSIC_HW_01)|
			(1<<DELOS_CLASSIC_HW_00)|
			(1<<DELOS_EVO_HW_00)|
			(1<<DELOS_EVO_HW_01)|
			(1<<DELOS_V3_HW_00)|
			(1<<DELOS_WINGX_HW_00)
	},
	{ /* Pressure/Temperature : LPS22HB */
		.info = {
			I2C_BOARD_INFO("lps22hb", 0x5C),
			.platform_data = &baro_init,
			.irq = -1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<DELOS_V3_HW_01)|
			(1<<DELOS_V3_HW_02)|
			(1<<DELOS_WINGX_HW_01)|
			(1<<DELOS_WINGX_HW_02)
	},
	{ /* IMU */
		.info = {
			I2C_BOARD_INFO("mpu6050", 0x68),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<DELOS_CLASSIC_HW_07)|
			(1<<DELOS_CLASSIC_HW_06)|
			(1<<DELOS_CLASSIC_HW_05)|
			(1<<DELOS_CLASSIC_HW_04)|
			(1<<DELOS_CLASSIC_HW_03)|
			(1<<DELOS_CLASSIC_HW_02)|
			(1<<DELOS_CLASSIC_HW_01)|
			(1<<DELOS_CLASSIC_HW_00)|
			(1<<DELOS_EVO_HW_00)|
			(1<<DELOS_EVO_HW_01)|
			(1<<DELOS_V3_HW_01)|
			(1<<DELOS_V3_HW_02)|
			(1<<DELOS_WINGX_HW_01)|
			(1<<DELOS_WINGX_HW_02)
	},
	{ /* IMU */
		.info = {
			I2C_BOARD_INFO("icm20608", 0x68),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<DELOS_V3_HW_00)|
			(1<<DELOS_WINGX_HW_00)
	},
	{ /* PCA9633 on EVO HW00 only */
		.info = {
			I2C_BOARD_INFO("pca9633", 0x15),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask = (1<<DELOS_EVO_HW_00)
	},
	{ /* PCA9633 on EVO HW01 and more */
		.info = {
			I2C_BOARD_INFO("pca9633", 0x62),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask = (1<<DELOS_EVO_HW_01)
	},
	{ /* SMB1358 (qualcomm charger) */
		.info = {
			I2C_BOARD_INFO("smb1358", 0x1c),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<DELOS_EVO_HW_01)|
			(1<<DELOS_V3_HW_00)|
			(1<<DELOS_V3_HW_01)|
			(1<<DELOS_V3_HW_02)|
			(1<<DELOS_WINGX_HW_00)|
			(1<<DELOS_WINGX_HW_01)|
			(1<<DELOS_WINGX_HW_02)
	},
	{ /* USB hub */
		.info = {
			/* Our version is not smsc82512 but is works the same
			 * way and the driver is made for smsc82512. */
			I2C_BOARD_INFO("smsc82512", 0x2c),
			.platform_data = &hub_init,
			.irq = -1,
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<DELOS_V3_HW_00)|
			(1<<DELOS_V3_HW_01)|
			(1<<DELOS_V3_HW_02)|
			(1<<DELOS_WINGX_HW_00)|
			(1<<DELOS_WINGX_HW_01)
			/* WingX HW02 does not have USB hub (no need) */
	},
};

/*
 * Disable motors using MOTOR_FAULT signal
 * This function doesn't reset pwm registers
 * Parameter gpio_status must be passed in order to know after execution of this function if motors are already disabled (gpio_status = 1)
 * @param gpio_status : address of variable gpio_status
 */
void delos_motor_disable(unsigned int* gpio_status)
{
	if (delos_hsis_gpio->gpio_MOTOR_FAULT >= 0) {
		printk(KERN_INFO "%s()\n", __func__);
		*gpio_status = gpio_get_value(delos_hsis_gpio->gpio_MOTOR_FAULT);
		gpio_set_value(delos_hsis_gpio->gpio_MOTOR_FAULT, 1); /* CLR# */
	}
	else
		printk(KERN_INFO "%s() No motor fault ...\n", __func__);
}
EXPORT_SYMBOL(delos_motor_disable);

/*
 * Enable Delos motors if gpio_status is cleared
 * @param gpio_status : parameter gpio_status
 */
void delos_motor_enable(unsigned int gpio_status)
{
	if (delos_hsis_gpio->gpio_MOTOR_FAULT >= 0) {
		printk(KERN_INFO "%s(%d)\n", __func__, gpio_status);
		/* If gpio_status is cleared, i.e. motors are not disabled before
		 * using delos_motor_disable(), we have to "reload" them */
		if(!gpio_status)
		{
			/* After this line, MOTOR_FAULT is cleared*/
			gpio_set_value(delos_hsis_gpio->gpio_MOTOR_FAULT, 0); /* CLR# */
		}
	}
	else
		printk(KERN_INFO "%s(%d) No motor fault ...\n", __func__, gpio_status);
}
EXPORT_SYMBOL(delos_motor_enable);


/*
 * This function reset the P6_PWM_RATIOxx registers
 */
static __init void delos_reset_pwms(void)
{
	char __iomem *pwm_regbase;
	const int pwm_ctrl_enable_all = 0x0f;
	unsigned int ntimer;

	struct clk *pwm_clk;

	printk(KERN_INFO "Delos Reset PWMs\n");

	pwm_clk = clk_get(NULL, "pwm");

	clk_enable(pwm_clk);

	pwm_regbase = ioremap(PARROT6_PWM, 256);

	/* Force all ratios to zero */
	for (ntimer=0;ntimer<4;ntimer++)
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

static void __init delos_reserve_mem(void)
{
	ramoops_data.mem_address = meminfo.bank[0].start +
		meminfo.bank[0].size - RAMOOPS_SIZE;

	memblock_reserve(&ramoops_data.mem_address,
			&ramoops_data.mem_size);
}

static void __init delos_ramoops_init(void)
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

/************ DELOS Classic ***********************/
static const struct_delos_hsis_gpio delos_hsis_gpio_classic_hw05 =
{
	.gpio_BT_ENABLE = DELOS_HSIS__GPIO_BT_ENABLE,
	.gpio_INT_INERT = DELOS_HSIS__GPIO_INT_INERT_HW05,
	.gpio_VBUS_DETECT = DELOS_HSIS__GPIO_VBUS_HW05,
	.gpio_BUTTON = DELOS_HSIS__GPIO_BUTTON_HW05,
	.gpio_US_POWER = DELOS_HSIS__GPIO_US_POWER,
	.gpio_HRESET_CAM = DELOS_HSIS__GPIO_HRESET_CAM,
	.gpio_MOTOR_FAULT = DELOS_HSIS__GPIO_MOTOR_FAULT_HW05,
	.gpio_POWER_ON_OFF = DELOS_HSIS__GPIO_ONOFF_HW05,
	.gpio_POWER_ON_OFF_on = 0,
	.gpio_RED_LED_LEFT = DELOS_HSIS__GPIO_RED_LED_L_HW05,
	.gpio_RED_LED_RIGHT = DELOS_HSIS__GPIO_RED_LED_R_HW05,
	.gpio_GREEN_LED_LEFT = DELOS_HSIS__GPIO_GREEN_LED_L_HW05,
	.gpio_GREEN_LED_RIGHT = DELOS_HSIS__GPIO_GREEN_LED_R_HW05,
	.gpio_PWMGEN_nOE = -1,
	.gpio_USB_MUX_CMD = -1,
	.gpio_MCU_RST = -1,
	.gpio_MCU_IT = -1,
	.gpio_BT_ANTENNA_EXTERN = -1,
	.gpio_BT_ANTENNA_EVO = -1,
	.gpio_INT_BARO = -1

};

static const struct_delos_hsis_gpio delos_hsis_gpio_classic_hw07 =
{
	.gpio_BT_ENABLE = DELOS_HSIS__GPIO_BT_ENABLE,
	.gpio_INT_INERT = DELOS_HSIS__GPIO_INT_INERT_HW07,
	.gpio_VBUS_DETECT = DELOS_HSIS__GPIO_VBUS_HW07,
	.gpio_BUTTON = DELOS_HSIS__GPIO_BUTTON_HW07,
	.gpio_US_POWER = DELOS_HSIS__GPIO_US_POWER,
	.gpio_HRESET_CAM = DELOS_HSIS__GPIO_HRESET_CAM,
	.gpio_MOTOR_FAULT = DELOS_HSIS__GPIO_MOTOR_FAULT_HW07,
	.gpio_POWER_ON_OFF = DELOS_HSIS__GPIO_ONOFF_HW07,
	.gpio_POWER_ON_OFF_on = 0,
	.gpio_RED_LED_LEFT = DELOS_HSIS__GPIO_RED_LED_L_HW07,
	.gpio_RED_LED_RIGHT = DELOS_HSIS__GPIO_RED_LED_R_HW07,
	.gpio_GREEN_LED_LEFT = DELOS_HSIS__GPIO_GREEN_LED_L_HW07,
	.gpio_GREEN_LED_RIGHT = DELOS_HSIS__GPIO_GREEN_LED_R_HW07,
	.gpio_PWMGEN_nOE = -1,
	.gpio_USB_MUX_CMD = -1,
	.gpio_MCU_RST = -1,
	.gpio_MCU_IT = -1,
	.gpio_BT_ANTENNA_EXTERN = -1,
	.gpio_BT_ANTENNA_EVO = -1,
	.gpio_INT_BARO = -1
};

static const struct_delos_hsis_pwm_sip6 delos_hsis_pwm_sip6_hw02 =
{
	.pwm_sip6_motor_1 = 1,
	.pwm_sip6_motor_2 = 2,
	.pwm_sip6_motor_3 = 0,
	.pwm_sip6_motor_4 = 3,
};

static const struct_delos_hsis_pwm_sip6 delos_hsis_pwm_sip6_old =
{
	.pwm_sip6_motor_1 = 2,
	.pwm_sip6_motor_2 = 1,
	.pwm_sip6_motor_3 = 3,
	.pwm_sip6_motor_4 = 0,
};

static void __init delos_classic_init(void)
{
	delos_init(DELOS_CLASSIC);
}

/************ DELOS EVO ***********************/
static const struct_delos_hsis_pwm_gen delos_hsis_pwm_gen_evo_hw00 =
{
	.pwm_gen_headlight_left  = DELOS_HSIS_EVO__PWM_GEN_HEADLIGHT_LEFT,
	.pwm_gen_headlight_right = DELOS_HSIS_EVO__PWM_GEN_HEADLIGHT_RIGHT,
	.pwm_gen_outdrv = 1,/* use totem pole structure on PCA9633 */
	.pwm_gen_headlight_complement = 1, /* complement on PWM LED command on PCA9633 */
};
static const struct_delos_hsis_pwm_gen delos_hsis_pwm_gen_evo_hw01 =
{
	.pwm_gen_headlight_left  = DELOS_HSIS_EVO__PWM_GEN_HEADLIGHT_LEFT,
	.pwm_gen_headlight_right = DELOS_HSIS_EVO__PWM_GEN_HEADLIGHT_RIGHT,
	.pwm_gen_outdrv = 0,/* use open drain on PCA9633 */
	.pwm_gen_headlight_complement = 0, /* no complement on PWM LED command on PCA9633 */
};

static const struct_delos_hsis_gpio delos_hsis_gpio_evo_hw00 =
{
	.gpio_BT_ENABLE = DELOS_HSIS__GPIO_BT_ENABLE,
	.gpio_INT_INERT = DELOS_HSIS_EVO__GPIO_INT_INERT_HW00,
	.gpio_VBUS_DETECT = DELOS_HSIS_EVO__GPIO_VBUS_DETECT_HW00,
	.gpio_BUTTON = DELOS_HSIS_EVO__GPIO_BUTTON_HW00,
	.gpio_US_POWER = DELOS_HSIS__GPIO_US_POWER,
	.gpio_HRESET_CAM = DELOS_HSIS__GPIO_HRESET_CAM,
	.gpio_MOTOR_FAULT = DELOS_HSIS_EVO__GPIO_MOTOR_FAULT_HW00,
	.gpio_POWER_ON_OFF = DELOS_HSIS_EVO__GPIO_ONOFF_HW00,
	.gpio_POWER_ON_OFF_on = 0,
	.gpio_RED_LED_LEFT = DELOS_HSIS_EVO__GPIO_RED_LED_L_HW00,
	.gpio_RED_LED_RIGHT = DELOS_HSIS_EVO__GPIO_RED_LED_R_HW00,
	.gpio_GREEN_LED_LEFT = DELOS_HSIS_EVO__GPIO_GREEN_LED_L_HW00,
	.gpio_GREEN_LED_RIGHT = DELOS_HSIS_EVO__GPIO_GREEN_LED_R_HW00,
	.gpio_PWMGEN_nOE = DELOS_HSIS_EVO__GPIO_NLED_DRIVER_OE,
	.gpio_USB_MUX_CMD = DELOS_HSIS_EVO__GPIO_USB_MUX_CMD,
	.gpio_MCU_RST = -1,
	.gpio_MCU_IT = -1,
	.gpio_BT_ANTENNA_EXTERN = -1,
	.gpio_BT_ANTENNA_EVO = -1,
	.gpio_INT_BARO = -1
};

static const struct_delos_hsis_gpio delos_hsis_gpio_evo_hw01 =
{
	.gpio_BT_ENABLE = 5,		/* RESET_BT		: Bluetooth reset */
	.gpio_INT_INERT = 52,		/* INT_INERT		: Input data ready signal of sensor mpu6050 */
	.gpio_VBUS_DETECT = 33,		/* VBUSD_MISO		: When delos is connected with usb interface, vbus_detect is set */
	.gpio_BUTTON = 32,		/* SWITCH_ON__MOSI	: Input Button to monitor : 0 when this button is released; 1 when someone presses it */
	.gpio_US_POWER = 56,		/* US_POWER		: activ low (0 = max voltage US, 1 = minimum voltage US POWER) */
	.gpio_HRESET_CAM = 58,		/* HRESET_CAM		: active LOW Camera RESET signal */
	.gpio_MOTOR_FAULT = 39,		/* MOTOR_FAULT		: Motors are "blocked" when output MOTOR_FAULT is set */
	.gpio_POWER_ON_OFF = 30,	/* SYS_OFF__CLK		: Control of power device */
	.gpio_POWER_ON_OFF_on = 0,	/* value of SYS_OFF__CLK that keeps the system ON. */
	.gpio_RED_LED_LEFT = 48,	/* RED_L		: Red left led, active high led signal */
	.gpio_RED_LED_RIGHT = 51,	/* RED_R		: Red right led, active high led signal */
	.gpio_GREEN_LED_LEFT = 49,	/* GREEN_L		: Green left led, active high led signal */
	.gpio_GREEN_LED_RIGHT = 31,	/* GREEN_R		: Green right led gpio */
	.gpio_PWMGEN_nOE = -1,
	.gpio_USB_MUX_CMD = 55,		/* USB_SW_CMD		: USB switch select */
	.gpio_MCU_RST = 44,
	.gpio_MCU_IT = -1,
	.gpio_BT_ANTENNA_EXTERN = -1,	/* BT_SW_CTRL_IO2	: BT antenna external wire*/
	.gpio_BT_ANTENNA_EVO = -1,	/* BT_SW_CTRL_IO1	: BT antenna delos evo*/
	.gpio_INT_BARO = -1
};

static const struct_delos_hsis_gpio delos_hsis_gpio_v3_hw00 =
{
	.gpio_BT_ENABLE = 5,		/* RESET_BT		: Bluetooth reset */
	.gpio_INT_INERT = 52,		/* INT_INERT		: Input data ready signal of sensor mpu6050 */
	.gpio_VBUS_DETECT = -1,		/* VBUSD_MISO		: When delos is connected with usb interface, vbus_detect is set */
	.gpio_BUTTON = -1,		/* SWITCH_ON__MOSI	: Input Button to monitor : 0 when this button is released; 1 when someone presses it */
	.gpio_US_POWER = -1,		/* US_POWER		: activ low (0 = max voltage US, 1 = minimum voltage US POWER) */
	.gpio_HRESET_CAM = 58,		/* HRESET_CAM		: active LOW Camera RESET signal */
	.gpio_MOTOR_FAULT = -1,		/* MOTOR_FAULT		: Motors are "blocked" when output MOTOR_FAULT is set */
	.gpio_POWER_ON_OFF = -1,	/* SYS_OFF__CLK		: Control of power device */
	.gpio_POWER_ON_OFF_on = -1,	/* value of SYS_OFF__CLK that keeps the system ON. */
	.gpio_RED_LED_LEFT = 48,	/* RED_L		: Red left led, active high led signal */
	.gpio_RED_LED_RIGHT = 51,	/* RED_R		: Red right led, active high led signal */
	.gpio_GREEN_LED_LEFT = 49,	/* GREEN_L		: Green left led, active high led signal */
	.gpio_GREEN_LED_RIGHT = 31,	/* GREEN_R		: Green right led gpio */
	.gpio_PWMGEN_nOE = -1,
	.gpio_USB_MUX_CMD = 55,		/* USB_SW_CMD		: USB switch select */
	.gpio_MCU_RST = 44,
	.gpio_MCU_IT = 56,
	.gpio_BT_ANTENNA_EXTERN = 53,	/* BT_SW_CTRL_IO2	: BT antenna external wire*/
	.gpio_BT_ANTENNA_EVO = 57,	/* BT_SW_CTRL_IO1	: BT antenna delos evo*/
	.gpio_INT_BARO = -1
};

static const struct_delos_hsis_gpio delos_hsis_gpio_v3_hw01 =
{
	.gpio_BT_ENABLE = 5,		/* RESET_BT		: Bluetooth reset */
	.gpio_INT_INERT = 52,		/* INT_INERT		: Input data ready signal of sensor mpu6050 */
	.gpio_INT_BARO = 38,		/* INT_BARO		: Input data ready signal of sensor lps22hb */
	.gpio_VBUS_DETECT = -1,		/* VBUSD_MISO		: When delos is connected with usb interface, vbus_detect is set */
	.gpio_BUTTON = -1,		/* SWITCH_ON__MOSI	: Input Button to monitor : 0 when this button is released; 1 when someone presses it */
	.gpio_US_POWER = -1,		/* US_POWER		: activ low (0 = max voltage US, 1 = minimum voltage US POWER) */
	.gpio_HRESET_CAM = 58,		/* HRESET_CAM		: active LOW Camera RESET signal */
	.gpio_MOTOR_FAULT = -1,		/* MOTOR_FAULT		: Motors are "blocked" when output MOTOR_FAULT is set */
	.gpio_POWER_ON_OFF = -1,	/* SYS_OFF__CLK		: Control of power device */
	.gpio_POWER_ON_OFF_on = -1,	/* value of SYS_OFF__CLK that keeps the system ON. */
	.gpio_RED_LED_LEFT = 48,	/* RED_L		: Red left led, active high led signal */
	.gpio_RED_LED_RIGHT = 51,	/* RED_R		: Red right led, active high led signal */
	.gpio_GREEN_LED_LEFT = 49,	/* GREEN_L		: Green left led, active high led signal */
	.gpio_GREEN_LED_RIGHT = 31,	/* GREEN_R		: Green right led gpio */
	.gpio_PWMGEN_nOE = -1,
	.gpio_USB_MUX_CMD = 55,		/* USB_SW_CMD		: USB switch select */
	.gpio_MCU_RST = 44,
	.gpio_MCU_IT = 56,
	.gpio_BT_ANTENNA_EXTERN = -1,	/* BT_SW_CTRL_IO2	: BT antenna external wire*/
	.gpio_BT_ANTENNA_EVO = -1,	/* BT_SW_CTRL_IO1	: BT antenna delos evo*/
};

static struct p6_spi_config p6i_spi_controller_data = {
	.tsetupcs_ns = 15,
	.tholdcs_ns  = 15,
};

static struct mcu_minidrones3_key spi_mcu_keys[] = {
	{ .bmask = 0x01, .input_key = KEY_COMPUTER },
	{ .bmask = 0x02, .input_key = KEY_POWER },
};

static struct mcu_minidrones3_platform_data spi_mcu_data  = {
	.spi_read_delay_ms = 2,
	.firmware_name = "pm_mcu/firmware.bin",
	.keys_nb = ARRAY_SIZE(spi_mcu_keys),
	.keys = spi_mcu_keys,
};

static struct spi_board_info spi_mcu_board_info[] __initconst = {
	{
		.modalias        = "mcu_minidrones3",
		.max_speed_hz    = 67709,
		.bus_num         = 2,
		.chip_select     = 0,
		.platform_data   = &spi_mcu_data,
		.controller_data = &p6i_spi_controller_data,
		.mode            = SPI_MODE_0,
	},
};

static struct spi_board_info p6i_spi_board_info[] =  {
	/* SPIDEV2.0 */
	{
		.modalias = "spidev",
		.max_speed_hz = 2000000,
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = NULL,
		.controller_data = &p6i_spi_controller_data,
		.mode = SPI_MODE_0,
	}
};

static int pic_programming_mode_on[] = {
	P6I_SPI2_MISOb,
	P6I_SPI2_MOSIb,
	P6I_SPI2_CLKb,
	0,
};

static int pic_programming_mode_off[] = {
	GPIO_033,
	GPIO_032,
	GPIO_030,
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

static void __init delos_evo_init(void)
{
	delos_init(DELOS_EVO);
}

static void __init delos_v3_init(void)
{
	delos_init(DELOS_V3);
}

static void __init delos_wingx_init(void)
{
	delos_init(DELOS_WINGX);
}

static void delos_power_off(void)
{
	int gpio = delos_hsis_gpio->gpio_POWER_ON_OFF;
	int val;
	val = gpio_get_value(gpio);
	gpio_set_value(gpio, val ? 0 : 1);
}

/************** INIT *****************/
static __init void delos_init(delos_type_t delos_type)
{
	u32 tmp;
	int i;
	const unsigned int *pins_init = NULL;
	const unsigned int *pins_init_pwm = NULL;

	sip6_init(1);
	platform_device_register(&user_gpio);

	delos_hsis = delos_hsis_default;
	delos_hsis.pcbrev = parrot_board_rev();

#ifdef CONFIG_RAMOOPS
	delos_ramoops_init();
#endif
	printk(KERN_INFO " ==== Delos type : %d (PCB rev : %d) ====\n", delos_type, delos_hsis.pcbrev);

	switch (delos_type) {
	case DELOS_EVO:
		/* EVO */
		delos_hsis.model = "EVO";
		pins_init_pwm = pins_init_pwm_classic_evo;

		switch(delos_hsis.pcbrev) {
		default :
			printk(KERN_INFO "WARNING : unknown and/or not supported EVO board version\n");
			printk(KERN_INFO "value of pcb version : %x\n", parrot_board_rev());
			printk(KERN_INFO "We're assuming that hardware version of this board is similar to the last hardware version managed by this BSP\n");
			/* There is no break, it's normal... */
		case 0x1 :
			printk(KERN_INFO "EVO hardware version 01 detected\n");
			delos_hsis_gpio = (struct_delos_hsis_gpio *)&delos_hsis_gpio_evo_hw01;
			delos_hsis_pwm_gen = (struct_delos_hsis_pwm_gen *)&delos_hsis_pwm_gen_evo_hw01;
			delos_hsis_pwm_sip6 = (struct_delos_hsis_pwm_sip6 *)&delos_hsis_pwm_sip6_hw02;

			delos_hsis.hwrev = DELOS_EVO_HW_01;
			break;
		case 0x0 :
			printk(KERN_INFO "EVO hardware version 00 detected\n");
			delos_hsis_gpio = (struct_delos_hsis_gpio *)&delos_hsis_gpio_evo_hw00;
			delos_hsis_pwm_gen = (struct_delos_hsis_pwm_gen *)&delos_hsis_pwm_gen_evo_hw00;
			delos_hsis_pwm_sip6 = (struct_delos_hsis_pwm_sip6 *)&delos_hsis_pwm_sip6_hw02;

			delos_hsis.hwrev = DELOS_EVO_HW_00;
			break;
		}
		break; /* case DELOS_EVO end */

	case DELOS_CLASSIC:
		/* Classic */
		delos_hsis.model = "Classic";
		delos_hsis_pwm_gen = (struct_delos_hsis_pwm_gen *)&delos_hsis_pwm_gen_default;
		pins_init_pwm = pins_init_pwm_classic_evo;

		switch(delos_hsis.pcbrev) {
		case 0x0 :
			//printk(KERN_INFO "Classic hardware version 00 detected\n");
		case 0x4 :
			//printk(KERN_INFO "Classic hardware version 01 detected\n");
		case 0x2 :
			//printk(KERN_INFO "Classic hardware version 02 detected\n");
		case 0x6 :
			//printk(KERN_INFO "Classic hardware version 03 detected\n");
		case 0x1 :
			//printk(KERN_INFO "Classic hardware version 04 detected\n");
			{
				int cpt=0;
				int tmp_real_hw_num = 0;
				tmp_real_hw_num |= ( delos_hsis.pcbrev>>cpt & 0x1 ) << (2-cpt);
				cpt++;
				tmp_real_hw_num |= ( delos_hsis.pcbrev>>cpt & 0x1 ) << (2-cpt);
				cpt++;
				tmp_real_hw_num |= ( delos_hsis.pcbrev>>cpt & 0x1 ) << (2-cpt);
				printk(KERN_ERR "XXX Classic hardware version less than 5 detected (%d) XXX\n", tmp_real_hw_num);
				printk(KERN_ERR "XXX This HW is not supported anymore!!! XXX\n");
				BUG();
				return;
			}

			break;

		default :
			printk(KERN_INFO "WARNING : unknown and/or not supported Classic board version\n");
			printk(KERN_INFO "value of pcb version : %x\n", parrot_board_rev());
			printk(KERN_INFO "We're assuming that hardware version of this board is similar to the last hardware version managed by this BSP\n");
			/* There is no break, it's normal... */
		case 0x05:
			printk(KERN_INFO "Classic hardware version 05 detected\n");
			delos_hsis_gpio = (struct_delos_hsis_gpio *)&delos_hsis_gpio_classic_hw05;
			delos_hsis_pwm_sip6 = (struct_delos_hsis_pwm_sip6 *)&delos_hsis_pwm_sip6_hw02;
			delos_hsis.hwrev = DELOS_CLASSIC_HW_05;
			break;

		case 0x03:
			printk(KERN_INFO "Classic hardware version 06 detected\n");
			delos_hsis_gpio = (struct_delos_hsis_gpio *)&delos_hsis_gpio_classic_hw07;
			delos_hsis_pwm_sip6 = (struct_delos_hsis_pwm_sip6 *)&delos_hsis_pwm_sip6_hw02;
			delos_hsis.hwrev = DELOS_CLASSIC_HW_06;
			break;

		case 0x07:
			printk(KERN_INFO "Classic hardware version 07 detected\n");
			delos_hsis_gpio = (struct_delos_hsis_gpio *)&delos_hsis_gpio_classic_hw07;
			delos_hsis_pwm_sip6 = (struct_delos_hsis_pwm_sip6 *)&delos_hsis_pwm_sip6_hw02;
			delos_hsis.hwrev = DELOS_CLASSIC_HW_07;
			break;
		}
		break; /* case DELOS_CLASSIC end */

	case DELOS_V3:
	case DELOS_WINGX:
		delos_hsis_pwm_gen = (struct_delos_hsis_pwm_gen *)&delos_hsis_pwm_gen_default;
		pins_init_pwm = pins_init_pwm_v3;

		switch (delos_type) {
		case DELOS_V3:
			delos_hsis.model = "V3";
			switch(delos_hsis.pcbrev) {
			default :
				printk(KERN_WARNING "WARNING : unknown and/or not supported %s board version\n",
						delos_hsis.model);
				printk(KERN_INFO "value of pcb version : 0x%X\n",
						parrot_board_rev());
				printk(KERN_INFO "  We're assuming that hardware version of this board\n");
				printk(KERN_INFO "  is similar to the last hardware version managed by this BSP\n");
				/* NO BREAK */
			case 0x2: delos_hsis.hwrev = DELOS_V3_HW_02; break;
			case 0x1: delos_hsis.hwrev = DELOS_V3_HW_01; break;
			case 0x0: delos_hsis.hwrev = DELOS_V3_HW_00; break;
			}
			break;
		case DELOS_WINGX:
			delos_hsis.model = "WingX";
			switch(delos_hsis.pcbrev) {
			default :
				printk(KERN_WARNING "WARNING : unknown and/or not supported %s board version\n",
						delos_hsis.model);
				printk(KERN_INFO "value of pcb version : 0x%X\n",
						parrot_board_rev());
				printk(KERN_INFO "  We're assuming that hardware version of this board\n");
				printk(KERN_INFO "  is similar to the last hardware version managed by this BSP\n");
				/* NO BREAK */
			case 0x2: delos_hsis.hwrev = DELOS_WINGX_HW_02; break;
			case 0x1: delos_hsis.hwrev = DELOS_WINGX_HW_01; break;
			case 0x0: delos_hsis.hwrev = DELOS_WINGX_HW_00; break;
			}
			break;
		default:
			printk(KERN_ERR "%s:%d It should be impossible to be here.\n",
					__FUNCTION__, __LINE__);
			dump_stack();
			break;
		}

		switch (delos_hsis.hwrev) {
		case DELOS_WINGX_HW_00:	/* This HW does not really exist
					 * WingX board version 00 had
					 * HW id delos3. */
		case DELOS_V3_HW_00:
			printk(KERN_INFO "%s HW version 00 used in BSP (%d)\n",
					delos_hsis.model, delos_hsis.pcbrev);
			delos_hsis_gpio = (struct_delos_hsis_gpio *)&delos_hsis_gpio_v3_hw00;
			delos_hsis_pwm_sip6 = (struct_delos_hsis_pwm_sip6 *)&delos_hsis_pwm_sip6_hw02;
			pins_init = pins_init_delos_v3_hw00;
			break;

		case DELOS_WINGX_HW_01:	/* This HW does not really exist
					 * WingX board version 01 had
					 * HW id delos3. */
		case DELOS_WINGX_HW_02:
		case DELOS_V3_HW_01:
		case DELOS_V3_HW_02:
			printk(KERN_INFO "%s HW version 01/02 used in BSP (%d)\n",
					delos_hsis.model, delos_hsis.pcbrev);
			delos_hsis_gpio = (struct_delos_hsis_gpio *)&delos_hsis_gpio_v3_hw01;
			delos_hsis_pwm_sip6 = (struct_delos_hsis_pwm_sip6 *)&delos_hsis_pwm_sip6_hw02;
			pins_init = pins_init_delos_v3_hw01;
			break;

		default:
			printk(KERN_ERR "%s:%d It should be impossible to be here.\n",
					__FUNCTION__, __LINE__);
			dump_stack();
		}
		break; /* case DELOS_V3/DELOS_WINGX end */

	default:
		printk(KERN_ERR "%s:%d It should be impossible to be here.\n",
				__FUNCTION__, __LINE__);
		dump_stack();
		break;
	}

	if (delos_hsis_gpio->gpio_MOTOR_FAULT >= 0) {
		/*
		 * MOTOR_FAULT must be set before enabling pins on pwm mode
		 */
		p6i_export_gpio(delos_hsis_gpio->gpio_MOTOR_FAULT,
				GPIOF_OUT_INIT_HIGH,
				"MOTOR_FAULT", 0);
	}
	/*
	 *  Identical to :
	 *  gpio_direction_output(DELOS_HSIS__GPIO_MOTOR_FAULT_HW03, 1);
	 *  for hardware 03 and superior versions
	 */

	/* Reset pwms before using pins as pwms */
	delos_reset_pwms();

	parrot_init_pin(pins_init_p6i);
	if (pins_init_pwm == NULL)
		printk(KERN_ERR "PWM pins init array not filled => PWM pins not set!\n");
	else
		parrot_init_pin(pins_init_pwm);

	// Pull Up on UART1 RX:
	p6i_set_pads_uart1_pullup();

	/* pads init */
	p6_i2cm0_device.dev.platform_data = &i2cm_platform_delos;
	p6i_set_pads_i2c(0);

	//p6i_eth_on_jtag_init();
	register_reboot_notifier(&delos_rst_notifier);

	printk(KERN_INFO "Initializing USB activation pin\n");

	switch(delos_hsis.hwrev) {
	case DELOS_CLASSIC_HW_00:
	case DELOS_CLASSIC_HW_01:
	case DELOS_CLASSIC_HW_02:
	case DELOS_CLASSIC_HW_03:
	case DELOS_CLASSIC_HW_04:
		printk(KERN_ERR "OLD HW => We should not get this far...\n");
		BUG();
		return;
		break;

	case DELOS_CLASSIC_HW_05:
		parrot_init_pin(pins_init_p6i_hw05);

		/* Set gpio2 (button) in high impendance */
		tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0xff07);
		__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

		break;

	case DELOS_CLASSIC_HW_06:
	case DELOS_CLASSIC_HW_07:
		parrot_init_pin(pins_init_p6i_hw07);

		/* Set gpio2 (button) in high impendance */
		tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0xff07);
		__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

		break;

	case DELOS_EVO_HW_00:
		parrot_init_pin(pins_init_delos_evo_hw00);

		/* Set gpio2 (button) in high impendance */
		tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0xff07);
		__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

		/* set drive strengh for usb */
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_PREEMDEPTH,0x01);
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_ENPRE,0x01);
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVAMPLITUDE,0x03);
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVSLOPE,0x0f);

		break;
	case DELOS_EVO_HW_01:
		parrot_init_pin(pins_init_delos_evo_hw01);
		platform_device_register(&p6_dynamic_pinmux_pic_prog);

		/* Set gpio2 (button) in high impendance */
		tmp = __raw_readl(PARROT6I_VA_RTC+_RTC_PULLS) & ~(0xff07);
		__raw_writel(tmp, PARROT6I_VA_RTC+_RTC_PULLS);

		/* set drive strengh for usb */
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_PREEMDEPTH,0x01);
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_ENPRE,0x01);
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVAMPLITUDE,0x03);
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVSLOPE,0x0f);

		break;

	case DELOS_V3_HW_00:
	case DELOS_V3_HW_01:
	case DELOS_V3_HW_02:
	case DELOS_WINGX_HW_00:
	case DELOS_WINGX_HW_01:
	case DELOS_WINGX_HW_02:
	default:
		parrot_init_pin(pins_init);

		/* set drive strengh for usb */
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_PREEMDEPTH,0x01);
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_ENPRE,0x01);
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVAMPLITUDE,0x03);
		p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVSLOPE,0x0f);
		break;
	}

	/* Register power off function */
	if (delos_hsis_gpio->gpio_POWER_ON_OFF >= 0)
		pm_power_off = delos_power_off;

	/* Takes alive power device by (RE)setting this gpio */
	if (delos_hsis_gpio->gpio_POWER_ON_OFF >= 0) {
		p6i_export_gpio(delos_hsis_gpio->gpio_POWER_ON_OFF,
				(delos_hsis_gpio->gpio_POWER_ON_OFF_on==1)?GPIOF_OUT_INIT_HIGH:GPIOF_OUT_INIT_LOW,
				"POWER_ON_OFF", 0);
	}
	/* Antenna wire */
	/* For wingX HW00 antenna has been connected to delos evo pad*/
	if (delos_hsis_gpio->gpio_BT_ANTENNA_EXTERN >= 0)
		p6i_export_gpio(delos_hsis_gpio->gpio_BT_ANTENNA_EXTERN, GPIOF_OUT_INIT_LOW,  "BT_ANTENNA_EXTERN", 1);
	if (delos_hsis_gpio->gpio_BT_ANTENNA_EVO >= 0)
		p6i_export_gpio(delos_hsis_gpio->gpio_BT_ANTENNA_EVO,   GPIOF_OUT_INIT_HIGH,  "BT_ANTENNA_EVO",   1);

	/* USB Camera is disabled */
	if (delos_hsis_gpio->gpio_HRESET_CAM >= 0)
		p6i_export_gpio(delos_hsis_gpio->gpio_HRESET_CAM, GPIOF_OUT_INIT_LOW,  "HRESET_CAM", 0);
	if (delos_hsis_gpio->gpio_US_POWER >= 0)
		p6i_export_gpio(delos_hsis_gpio->gpio_US_POWER,   GPIOF_OUT_INIT_LOW,  "US_POWER",   1);

	/* bluetooth */
	p6i_export_gpio(delos_hsis_gpio->gpio_BT_ENABLE,  GPIOF_OUT_INIT_HIGH, "BT_ENABLE",  0);


	/* Initialisation for detection of press button */
	if (delos_hsis_gpio->gpio_BUTTON >= 0) {
		gpio_interrupt_register(delos_hsis_gpio->gpio_BUTTON, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(delos_hsis_gpio->gpio_BUTTON, GPIOF_IN, "BUTTON", 0);
	}

	/* leds */
	/* "Eyes" of Delos are orange at boot */
	p6i_export_gpio(delos_hsis_gpio->gpio_GREEN_LED_LEFT,  GPIOF_OUT_INIT_HIGH,  "GREEN_LED_LEFT", 0);
	p6i_export_gpio(delos_hsis_gpio->gpio_GREEN_LED_RIGHT, GPIOF_OUT_INIT_HIGH,  "GREEN_LED_RIGHT", 0);
	p6i_export_gpio(delos_hsis_gpio->gpio_RED_LED_RIGHT,   GPIOF_OUT_INIT_HIGH, "RED_LED_RIGHT", 0);
	p6i_export_gpio(delos_hsis_gpio->gpio_RED_LED_LEFT,    GPIOF_OUT_INIT_HIGH, "RED_LED_LEFT", 0);

	/* We have to monitor the interrupt pin of mpu6050 which raises each 5 ms */
	gpio_interrupt_register(delos_hsis_gpio->gpio_INT_INERT, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
	p6i_export_gpio(delos_hsis_gpio->gpio_INT_INERT, GPIOF_IN, "INT_INERT", 1);

	/* Detection of plug/unplug usb */
	if (delos_hsis_gpio->gpio_VBUS_DETECT >= 0) {
		gpio_interrupt_register(delos_hsis_gpio->gpio_VBUS_DETECT, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(delos_hsis_gpio->gpio_VBUS_DETECT, GPIOF_IN, "VBUS_DETECT", 1);
	}

	if (delos_hsis_gpio->gpio_PWMGEN_nOE >= 0)
		p6i_export_gpio(delos_hsis_gpio->gpio_PWMGEN_nOE, GPIOF_OUT_INIT_HIGH, "PWMGEN_nOE", 0);/* turn off headlights */

	if (delos_hsis_gpio->gpio_USB_MUX_CMD >= 0)
		p6i_export_gpio(delos_hsis_gpio->gpio_USB_MUX_CMD,    GPIOF_OUT_INIT_HIGH, "USB_MUX_CMD",    0);/* select Etron chip */

	if (delos_hsis_gpio->gpio_MCU_RST >= 0) {
		p6i_export_gpio(delos_hsis_gpio->gpio_MCU_RST,    GPIOF_OUT_INIT_LOW, "MCU_RST",    0);
		if (delos_hsis_gpio->gpio_MCU_IT >= 0 &&
				mcu_minidrones3_driver_enabled()) {
			/* Use spi_gpio driver. */
			gpio_request(delos_hsis_gpio->gpio_MCU_IT, "MCU_IT");
			gpio_direction_input(delos_hsis_gpio->gpio_MCU_IT);
			printk(KERN_INFO "Now registering PM MCU interface: "
			                 "irq gpio %d, nb spi = %d\n",
			                 delos_hsis_gpio->gpio_MCU_IT,
			                 ARRAY_SIZE(spi_mcu_board_info));
			spi_mcu_data.gpio_rst = delos_hsis_gpio->gpio_MCU_RST;
			spi_mcu_board_info->irq = gpio_interrupt_register(
					delos_hsis_gpio->gpio_MCU_IT,
					GPIO_IRQ_POS,
					GPIO_DEBOUNCE_NONE);
			spi_register_board_info(spi_mcu_board_info,
					ARRAY_SIZE(spi_mcu_board_info));
		} else {
			/* Old-style spidev interface for MCU flashing. */
			printk(KERN_INFO "Registering old-style PM MCU interface: nb spi = %d\n",
					ARRAY_SIZE(p6i_spi_board_info));
			spi_register_board_info(p6i_spi_board_info, ARRAY_SIZE(p6i_spi_board_info));
		}
	}

	switch(delos_hsis.hwrev) {
	case DELOS_CLASSIC_HW_01:
	case DELOS_CLASSIC_HW_02:
	case DELOS_CLASSIC_HW_00:
	case DELOS_CLASSIC_HW_03:
	case DELOS_CLASSIC_HW_04:
		printk(KERN_EMERG "OLD HW => We really should not get this far...\n");
		BUG();
		return;
		break;
	default:
		/* Nothing to do */
		break;
	}

	platform_add_devices(p6i_devices, ARRAY_SIZE(p6i_devices));

	p6i_export_uart_hw_infos(1, 0, "");

	for (i = 0; i < ARRAY_SIZE(delos_i2c_devices); i++)
	{
		struct my_i2c_board_info *ptr=&delos_i2c_devices[i];
		if (ptr->hwrev_mask & (1<<delos_hsis.hwrev)) {
			if (ptr->info.platform_data == &baro_init)
				ptr->info.irq = gpio_interrupt_register(delos_hsis_gpio->gpio_INT_BARO, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);

			p6i_export_i2c_hw_infos(ptr->busnum,
					ptr->info.addr,
					ptr->version,
					ptr->info.type);
			i2c_register_board_info(0, &(ptr->info),
					1);
		}
	}

	sysfs_delos_init();
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

MAP_BSP_VALUE_STRING(model, delos_hsis);
MAP_BSP_VALUE_INT_NP(hwrev, delos_hsis);
MAP_BSP_VALUE_INT_NP(pcbrev, delos_hsis);

MAP_BSP_VALUE_INT(pwm_gen_headlight_left, delos_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_headlight_right, delos_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_outdrv, delos_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_headlight_complement, delos_hsis_pwm_gen);

MAP_BSP_VALUE_INT(pwm_sip6_motor_1, delos_hsis_pwm_sip6);
MAP_BSP_VALUE_INT(pwm_sip6_motor_2, delos_hsis_pwm_sip6);
MAP_BSP_VALUE_INT(pwm_sip6_motor_3, delos_hsis_pwm_sip6);
MAP_BSP_VALUE_INT(pwm_sip6_motor_4, delos_hsis_pwm_sip6);

MAP_BSP_VALUE_INT(gpio_BT_ENABLE, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_INT_INERT, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_INT_BARO, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_VBUS_DETECT, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_BUTTON, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_US_POWER, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_HRESET_CAM, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_MOTOR_FAULT, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_POWER_ON_OFF, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_POWER_ON_OFF_on, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_RED_LED_LEFT, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_RED_LED_RIGHT, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_GREEN_LED_LEFT, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_GREEN_LED_RIGHT, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_PWMGEN_nOE, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_USB_MUX_CMD, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_MCU_RST, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_MCU_IT, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_BT_ANTENNA_EXTERN, delos_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_BT_ANTENNA_EVO, delos_hsis_gpio);


static const struct attribute *delos_bsp_attrs[] = {
	BSP_ATTR(model),
	BSP_ATTR(hwrev),
	BSP_ATTR(pcbrev),
	BSP_ATTR(pwm_gen_headlight_left),
	BSP_ATTR(pwm_gen_headlight_right),
	BSP_ATTR(pwm_gen_outdrv),
	BSP_ATTR(pwm_gen_headlight_complement),
	BSP_ATTR(pwm_sip6_motor_1),
	BSP_ATTR(pwm_sip6_motor_2),
	BSP_ATTR(pwm_sip6_motor_3),
	BSP_ATTR(pwm_sip6_motor_4),
	BSP_ATTR(gpio_BT_ENABLE),
	BSP_ATTR(gpio_INT_INERT),
	BSP_ATTR(gpio_INT_BARO),
	BSP_ATTR(gpio_VBUS_DETECT),
	BSP_ATTR(gpio_BUTTON),
	BSP_ATTR(gpio_US_POWER),
	BSP_ATTR(gpio_HRESET_CAM),
	BSP_ATTR(gpio_MOTOR_FAULT),
	BSP_ATTR(gpio_POWER_ON_OFF),
	BSP_ATTR(gpio_RED_LED_LEFT),
	BSP_ATTR(gpio_RED_LED_RIGHT),
	BSP_ATTR(gpio_GREEN_LED_LEFT),
	BSP_ATTR(gpio_GREEN_LED_RIGHT),
	BSP_ATTR(gpio_PWMGEN_nOE),
	BSP_ATTR(gpio_USB_MUX_CMD),
	BSP_ATTR(gpio_MCU_RST),
	BSP_ATTR(gpio_MCU_IT),
	BSP_ATTR(gpio_POWER_ON_OFF_on),
	BSP_ATTR(gpio_BT_ANTENNA_EXTERN),
	BSP_ATTR(gpio_BT_ANTENNA_EVO),

	NULL
};

static const struct attribute_group delos_bsp_attr_group = {
	.attrs = (struct attribute **)delos_bsp_attrs,
};

static __init int sysfs_delos_init(void)
{
	struct kobject *js_kobject;
	int ret;

	printk(KERN_INFO "Delos %s: exporting HSIS to userspace in /sys/kernel/hsis",
			delos_hsis.model);

	js_kobject = kobject_create_and_add("hsis", kernel_kobj);
	if (!js_kobject)
		return -ENOMEM;

	ret = sysfs_create_group(js_kobject, &delos_bsp_attr_group);
	if (ret) {
		kobject_put(js_kobject);
		return ret;
	}

	return 0;
}

MACHINE_START(PARROT_DELOS, "Delos Classic sip6 board")
	/* Maintainer: Parrot S.A. */
#ifdef CONFIG_RAMOOPS
	.reserve    = delos_reserve_mem,
#endif
	.phys_io    = PARROT6_UART0,
	.io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params    = PARROT6_DDR_BASE+0x100,
	.map_io     = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer      = &p6_timer,
	.init_machine   = delos_classic_init,
MACHINE_END

MACHINE_START(PARROT_DELOS_EVO, "Delos EVO sip6 board")
	/* Maintainer: Parrot S.A. */
#ifdef CONFIG_RAMOOPS
	.reserve    = delos_reserve_mem,
#endif
	.phys_io    = PARROT6_UART0,
	.io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params    = PARROT6_DDR_BASE+0x100,
	.map_io     = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer      = &p6_timer,
	.init_machine   = delos_evo_init,
MACHINE_END

MACHINE_START(PARROT_DELOS3, "Delos3 board")
	/* Maintainer: Parrot S.A. */
#ifdef CONFIG_RAMOOPS
	.reserve    = delos_reserve_mem,
#endif
	.phys_io    = PARROT6_UART0,
	.io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params    = PARROT6_DDR_BASE+0x100,
	.map_io     = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer      = &p6_timer,
	.init_machine   = delos_v3_init,
MACHINE_END

MACHINE_START(PARROT_WINGX, "WingX board")
	/* Maintainer: Parrot S.A. */
#ifdef CONFIG_RAMOOPS
	.reserve    = delos_reserve_mem,
#endif
	.phys_io    = PARROT6_UART0,
	.io_pg_offst    = (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params    = PARROT6_DDR_BASE+0x100,
	.map_io     = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer      = &p6_timer,
	.init_machine   = delos_wingx_init,
MACHINE_END

