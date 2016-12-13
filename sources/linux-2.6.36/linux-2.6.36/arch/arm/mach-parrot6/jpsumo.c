/*
 *  linux/arch/arm/mach-parrot6/jpsumo.c
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
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/mmc/host.h>
#include <linux/i2c.h>
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
#include <../drivers/parrot/spi/mcu_minidrones3.h>

#include <misc/dynamic_pinmux.h>

#include <mach/jpsumo_hwrev.h>

#ifndef CONFIG_GPIOLIB
#error GPIOLIB is required. Add it to configuration.
#endif

typedef enum _jpsumo_type_t
{
	JPSUMO_CLASSIC = 0,
	JPSUMO_EVO,
	JPSUMO_V3,
} jpsumo_type_t;

// ----------------------------------- JPSUMO pins etc... -----------------------------------
typedef enum _jpsumo_classic_pcb_version_t
{
	JPSUMO_CLASSIC_PCB_A1 = 0x04,	// HW 01	( The 3 bits have been inverted => HW Bin 001 => Bin 100 => 4 )
	JPSUMO_CLASSIC_PCB_A2 = 0x02,	// HW 02
	JPSUMO_CLASSIC_PCB_A3 = 0x06,	// HW 03
	JPSUMO_CLASSIC_PCB_B1 = 0x01,	// HW 04
	JPSUMO_CLASSIC_PCB_B2 = 0x05,	// HW 05
	JPSUMO_CLASSIC_PCB_06 = 0x05,	// HW 06	// HW 6 board, were used fir DV1, but the version number was still 0x05
	JPSUMO_CLASSIC_PCB_07 = 0x07,	// HW 07
} jpsumo_classic_hw_version_t;

typedef enum _jpsumo_evo_pcb_version_t
{
	JPSUMO_EVO_PCB_00 = 0x00,	// HW 00
	JPSUMO_EVO_PCB_01,	        // HW 01
	JPSUMO_EVO_PCB_02,	        // HW 02
} jpsumo_evo_hw_version_t;

typedef enum _jpsumo_v3_pcb_version_t
{
	JPSUMO_V3_PCB_00 = 0x00,	// HW 00
} jpsumo_v3_pcb_version_t;


// --------------------------------
// -------  JPSUMO_PROTO_A1  ------
// --------------------------------

// JUMPING SUMO GPIOs:
#define JS_HSIS_A1__GPIOPIN_LED_DEBUG			   GPIO_043
#define JS_HSIS_A1__GPIO_LED_DEBUG				  43

#define JS_HSIS_A1__GPIOPIN_SWITCH_POSITION_1	   GPIO_050
#define JS_HSIS_A1__GPIO_SWITCH_POSITION_1		  50
#define JS_HSIS_A1__GPIOPIN_SWITCH_POSITION_2	   GPIO_051
#define JS_HSIS_A1__GPIO_SWITCH_POSITION_2		  51

#define JS_HSIS_A1__GPIOPIN_1V8_P6i_EN			  P6I_GPIO_007
#define JS_HSIS_A1__GPIO_1V8_P6i_EN				 7
#define JS_HSIS_A1__GPIOPIN_ALIM_OFF				GPIO_058
#define JS_HSIS_A1__GPIO_ALIM_OFF				   58

#define JS_HSIS_A1__GPIOPIN_JUMP_CTRL_1			 GPIO_038
#define JS_HSIS_A1__GPIO_JUMP_CTRL_1				38
#define JS_HSIS_A1__GPIOPIN_JUMP_CTRL_2			 GPIO_039
#define JS_HSIS_A1__GPIO_JUMP_CTRL_2				39

// CHARGE_STATUS		Input... Charge status...
#define JS_HSIS_A1__GPIOPIN_CHARGE_STATUS		   GPIO_053
#define JS_HSIS_A1__GPIO_CHARGE_STATUS			  53

// wifi...
#define JS_HSIS_A1__GPIOPIN_WLAN_nPWD			   GPIO_040
#define JS_HSIS_A1__GPIO_WLAN_nPWD				  40
#define JS_HSIS_A1__GPIOPIN_WLAN_WARM_RST		   GPIO_041
#define JS_HSIS_A1__GPIO_WLAN_WARM_RST			  41

#define JS_HSIS_A1__GPIOPIN_nFAULT_WHEEL			GPIO_054	// input from wheel driver
#define JS_HSIS_A1__GPIO_nFAULT_WHEEL			   54
#define JS_HSIS_A1__GPIOPIN_nSLEEP_WHEEL			GPIO_055	// output to kill wheel driver
#define JS_HSIS_A1__GPIO_nSLEEP_WHEEL			   55

#define JS_HSIS_A1__GPIOPIN_PWMGEN_nOE			  P6I_GPIO_004	// XXX This is a patch that should be on all Proto A1 boards.
#define JS_HSIS_A1__GPIO_PWMGEN_nOE				 4


// INT_MAGNETO	  GPIO_056		// interupt from magneto	// unused

// INT_INERT : interupt from MPU6050
#define JS_HSIS_A1__GPIOPIN_INT_INERT			   GPIO_057
#define JS_HSIS_A1__GPIO_INT_INERT				  57

// Unused on proto A1
#define JS_HSIS_A1__PWMPIN_PWMSIP6_0				P6I_PWM_00a
//#define JS_HSIS_A1__PWMPIN_PWMSIP6_1			  P6I_PWM_01a		// Right now, we use this GPIO for something else. // XXX replaced by JS_HSIS_A1__GPIOPIN_PWMGEN_nOE on the same pin.
#define JS_HSIS_A1__PWMPIN_PWMSIP6_2				P6I_PWM_02a
#define JS_HSIS_A1__PWMPIN_PWMSIP6_3				P6I_PWM_03a

#define JS_HSIS_A1__PWM_SIP6_WHEEL_LEFT_A      0
#define JS_HSIS_A1__PWM_SIP6_WHEEL_LEFT_B      -1
#define JS_HSIS_A1__PWM_SIP6_WHEEL_RIGHT_A     2
#define JS_HSIS_A1__PWM_SIP6_WHEEL_RIGHT_B     3

#define JS_HSIS_A1__GPIOPIN_nAMP_PWDN			   GPIO_052
#define JS_HSIS_A1__GPIO_nAMP_PWDN				  52

/* PWM generator on PCA96n */
#define JS_HSIS_A1__PWM_GEN_JUMP_A      0
#define JS_HSIS_A1__PWM_GEN_JUMP_B      1

// --------------------------------
// -------  JPSUMO_PROTO_A2  ------
// --------------------------------

// JUMPING SUMO GPIOs:
#define JS_HSIS_A2__GPIOPIN_LED_IR				  P6I_GPIO_010
#define JS_HSIS_A2__GPIO_LED_IR					 10

#define JS_HSIS_A2__GPIOPIN_NRST_CAM_1			  GPIO_040
#define JS_HSIS_A2__GPIO_NRST_CAM_1				 40
#define JS_HSIS_A2__GPIOPIN_NRST_CAM				GPIO_043
#define JS_HSIS_A2__GPIO_NRST_CAM				   43

#define JS_HSIS_A2__GPIOPIN_DEV_GPIO_1			  GPIO_048
#define JS_HSIS_A2__GPIO_DEV_GPIO_1				 48
#define JS_HSIS_A2__GPIOPIN_DEV_GPIO_2			  GPIO_049
#define JS_HSIS_A2__GPIO_DEV_GPIO_2				 49
#define JS_HSIS_A2__GPIOPIN_DEV_GPIO_3			  GPIO_051
#define JS_HSIS_A2__GPIO_DEV_GPIO_3				 51
#define JS_HSIS_A2__GPIOPIN_DEV_GPIO_4			  GPIO_050
#define JS_HSIS_A2__GPIO_DEV_GPIO_4				 50

#define JS_HSIS_A2__GPIOPIN_1V8_P6i_EN			  P6I_GPIO_007
#define JS_HSIS_A2__GPIO_1V8_P6i_EN				 7

#define JS_HSIS_A2__GPIOPIN_JUMP_CTRL_1			 GPIO_038
#define JS_HSIS_A2__GPIO_JUMP_CTRL_1				38
#define JS_HSIS_A2__GPIOPIN_JUMP_CTRL_2			 GPIO_039
#define JS_HSIS_A2__GPIO_JUMP_CTRL_2				39

#define JS_HSIS_A2__GPIOPIN_CHARGE_STATUS		   GPIO_053	// XXX Test point, should not be necessary
#define JS_HSIS_A2__GPIO_CHARGE_STATUS			  53

#define JS_HSIS_A2__GPIOPIN_nFAULT_WHEEL			GPIO_054	// input from wheel driver
#define JS_HSIS_A2__GPIO_nFAULT_WHEEL			   54
#define JS_HSIS_A2__GPIOPIN_nSLEEP_WHEEL			GPIO_055	// output to kill wheel driver
#define JS_HSIS_A2__GPIO_nSLEEP_WHEEL			   55

#define JS_HSIS_A2__GPIOPIN_PWMGEN_nOE			  GPIO_042
#define JS_HSIS_A2__GPIO_PWMGEN_nOE				 42

// INT_INERT : interupt from MPU6050
#define JS_HSIS_A2__GPIOPIN_INT_INERT			   GPIO_057
#define JS_HSIS_A2__GPIO_INT_INERT				  57


#define JS_HSIS_A2__PWMPIN_PWMSIP6_0				P6I_PWM_00a
#define JS_HSIS_A2__PWMPIN_PWMSIP6_1				P6I_PWM_01a
#define JS_HSIS_A2__PWMPIN_PWMSIP6_2				P6I_PWM_02a
#define JS_HSIS_A2__PWMPIN_PWMSIP6_3				P6I_PWM_03a

#define JS_HSIS_A2__PWM_SIP6_WHEEL_LEFT_A      0
#define JS_HSIS_A2__PWM_SIP6_WHEEL_LEFT_B      1
#define JS_HSIS_A2__PWM_SIP6_WHEEL_RIGHT_A     2
#define JS_HSIS_A2__PWM_SIP6_WHEEL_RIGHT_B     3

#define JS_HSIS_A2__GPIOPIN_nAMP_PWDN			   GPIO_052
#define JS_HSIS_A2__GPIO_nAMP_PWDN				  52

/* PWM generator on PCA96n */
#define JS_HSIS_A2__PWM_GEN_JUMP_A      0
#define JS_HSIS_A2__PWM_GEN_JUMP_B      1

// -------------------------------------------
// -------  JPSUMO_PROTO_A3 (and HW>=3) ------
// -------------------------------------------

// JUMPING SUMO GPIOs:
#define JS_HSIS_A3__GPIOPIN_LED_IR				  P6I_GPIO_010
#define JS_HSIS_A3__GPIO_LED_IR					 10

#define JS_HSIS_A3__GPIOPIN_LEFT_EYE_LED			P6I_GPIO_011
#define JS_HSIS_A3__GPIO_LEFT_EYE_LED			   11
#define JS_HSIS_A3__GPIOPIN_RIGHT_EYE_LED		   P6I_GPIO_012
#define JS_HSIS_A3__GPIO_RIGHT_EYE_LED			  12

#define JS_HSIS_A3__GPIOPIN_NRST_CAM				GPIO_040
#define JS_HSIS_A3__GPIO_NRST_CAM				   40

#define JS_HSIS_A3__GPIOPIN_DEV_GPIO_1			  GPIO_048
#define JS_HSIS_A3__GPIO_DEV_GPIO_1				 48
#define JS_HSIS_A3__GPIOPIN_DEV_GPIO_2			  GPIO_049
#define JS_HSIS_A3__GPIO_DEV_GPIO_2				 49
#define JS_HSIS_A3__GPIOPIN_DEV_GPIO_3			  GPIO_051
#define JS_HSIS_A3__GPIO_DEV_GPIO_3				 51

#define JS_HSIS_A3__GPIOPIN_1V8_P6i_EN			  P6I_GPIO_007
#define JS_HSIS_A3__GPIO_1V8_P6i_EN				 7

#define JS_HSIS_A3__GPIOPIN_JUMP_CTRL_1			 GPIO_038
#define JS_HSIS_A3__GPIO_JUMP_CTRL_1				38
#define JS_HSIS_A3__GPIOPIN_JUMP_CTRL_2			 GPIO_039
#define JS_HSIS_A3__GPIO_JUMP_CTRL_2				39

#define JS_HSIS_A3__GPIOPIN_CHARGE_STATUS		   GPIO_053	// XXX Test point, should not be necessary
#define JS_HSIS_A3__GPIO_CHARGE_STATUS			  53

#define JS_HSIS_A3__GPIOPIN_nFAULT_WHEEL			GPIO_054	// input from wheel driver
#define JS_HSIS_A3__GPIO_nFAULT_WHEEL			   54
#define JS_HSIS_A3__GPIOPIN_nSLEEP_WHEEL			GPIO_055	// output to kill wheel driver
#define JS_HSIS_A3__GPIO_nSLEEP_WHEEL			   55

#define JS_HSIS_A3__GPIOPIN_PWMGEN_nOE			  GPIO_042
#define JS_HSIS_A3__GPIO_PWMGEN_nOE				 42

// INT_INERT : interupt from MPU6050
#define JS_HSIS_A3__GPIOPIN_INT_INERT			   GPIO_057
#define JS_HSIS_A3__GPIO_INT_INERT				  57


#define JS_HSIS_A3__PWMPIN_PWMSIP6_0				P6I_PWM_00a
#define JS_HSIS_A3__PWMPIN_PWMSIP6_1				P6I_PWM_01a
#define JS_HSIS_A3__PWMPIN_PWMSIP6_2				P6I_PWM_02a
#define JS_HSIS_A3__PWMPIN_PWMSIP6_3				P6I_PWM_03a

#define JS_HSIS_A3__PWM_SIP6_WHEEL_LEFT_A      0
#define JS_HSIS_A3__PWM_SIP6_WHEEL_LEFT_B      1
#define JS_HSIS_A3__PWM_SIP6_WHEEL_RIGHT_A     2
#define JS_HSIS_A3__PWM_SIP6_WHEEL_RIGHT_B     3

#define JS_HSIS_A3__GPIOPIN_nAMP_PWDN			   GPIO_035
#define JS_HSIS_A3__GPIO_nAMP_PWDN				  35

#define JS_HSIS_A3__GPIOPIN_MUX_GPIO_ADC_IN0		GPIO_033
#define JS_HSIS_A3__GPIO_MUX_GPIO_ADC_IN0		   33
#define JS_HSIS_A3__GPIOPIN_MUX_GPIO_ADC_IN1_2	  GPIO_034
#define JS_HSIS_A3__GPIO_MUX_GPIO_ADC_IN1_2		 34

/* PWM generator on PCA96n */
#define JS_HSIS_A3__PWM_GEN_JUMP_A      0
#define JS_HSIS_A3__PWM_GEN_JUMP_B      1
#define JS_HSIS_A3__PWM_GEN_LEFT_EYE_LED_GREEN 2
#define JS_HSIS_A3__PWM_GEN_RIGHT_EYE_LED_GREEN 3

// -------------------------------------------
// -------  JPSUMO_PROTO_EVO (and HW>=3) ------
// -------------------------------------------

// JUMPING SUMO GPIOs:
#define	JS_HSIS_EVO__GPIOPIN_ON_OFF                 P6I_GPIO_002
#define JS_HSIS_EVO__GPIO_ON_OFF                    2

#define	JS_HSIS_EVO__GPIOPIN_LEFT_EYE_LED_RED       GPIO_043
#define JS_HSIS_EVO__GPIO_LEFT_EYE_LED_RED          43
#define	JS_HSIS_EVO__GPIOPIN_RIGHT_EYE_LED_RED      GPIO_041
#define JS_HSIS_EVO__GPIO_RIGHT_EYE_LED_RED         41

#define	JS_HSIS_EVO__GPIOPIN_LEFT_EYE_LED_GREEN     GPIO_053
#define JS_HSIS_EVO__GPIO_LEFT_EYE_LED_GREEN        53
#define	JS_HSIS_EVO__GPIOPIN_RIGHT_EYE_LED_GREEN    GPIO_052
#define JS_HSIS_EVO__GPIO_RIGHT_EYE_LED_GREEN       52

#define	JS_HSIS_EVO__GPIOPIN_NRST_CAM		    GPIO_040
#define JS_HSIS_EVO__GPIO_NRST_CAM                  40
#define JS_HSIS_EVO__GPIOPIN_BUTTON                 JS_HSIS_EVO__GPIOPIN_NRST_CAM
#define JS_HSIS_EVO__GPIO_BUTTON                    JS_HSIS_EVO__GPIO_NRST_CAM

// #define	JS_HSIS_EVO__GPIOPIN_DEV_GPIO_1             GPIO_048
// #define JS_HSIS_EVO__GPIO_DEV_GPIO_1                48
// #define	JS_HSIS_EVO__GPIOPIN_DEV_GPIO_2             GPIO_049
// #define JS_HSIS_EVO__GPIO_DEV_GPIO_2                49
// #define	JS_HSIS_EVO__GPIOPIN_DEV_GPIO_3             GPIO_051
// #define JS_HSIS_EVO__GPIO_DEV_GPIO_3                51

#define JS_HSIS_EVO__GPIOPIN_1V8_P6i_EN             P6I_GPIO_007
#define JS_HSIS_EVO__GPIO_1V8_P6i_EN                7

#define	JS_HSIS_EVO__GPIOPIN_JUMP_CTRL_1            GPIO_038
#define JS_HSIS_EVO__GPIO_JUMP_CTRL_1               38
#define	JS_HSIS_EVO__GPIOPIN_JUMP_CTRL_2            GPIO_039
#define JS_HSIS_EVO__GPIO_JUMP_CTRL_2               39

#define	JS_HSIS_EVO__GPIOPIN_nFAULT_WHEEL           GPIO_054	// input from wheel driver
#define JS_HSIS_EVO__GPIO_nFAULT_WHEEL              54
#define	JS_HSIS_EVO__GPIOPIN_nSLEEP_WHEEL           GPIO_055	// output to kill wheel driver
#define JS_HSIS_EVO__GPIO_nSLEEP_WHEEL              55

#define	JS_HSIS_EVO__GPIOPIN_PWMGEN_nOE             GPIO_042
#define JS_HSIS_EVO__GPIO_PWMGEN_nOE                42

// INT_INERT : interrupt from MPU6050
#define	JS_HSIS_EVO__GPIOPIN_INT_INERT              GPIO_057
#define JS_HSIS_EVO__GPIO_INT_INERT                 57

#define	JS_HSIS_EVO__GPIOPIN_nAMP_PWDN              GPIO_045
#define JS_HSIS_EVO__GPIO_nAMP_PWDN                 45

#define	JS_HSIS_EVO__GPIOPIN_MUX_GPIO_ADC_IN0       GPIO_050
#define JS_HSIS_EVO__GPIO_MUX_GPIO_ADC_IN0	    50
#define	JS_HSIS_EVO__GPIOPIN_MUX_GPIO_ADC_IN1_2     GPIO_044
#define JS_HSIS_EVO__GPIO_MUX_GPIO_ADC_IN1_2	    44

#define	JS_HSIS_EVO__GPIOPIN_MUX_GPIO_VBUS_DETECT   GPIO_056
#define	JS_HSIS_EVO__GPIO_MUX_GPIO_VBUS_DETECT      56

#define	JS_HSIS_EVO__GPIOPIN_MUX_GPIO_USB_MUX_CMD       GPIO_058
#define	JS_HSIS_EVO__GPIO_MUX_GPIO_USB_MUX_CMD          58

#define	JS_HSIS_EVO__PWMPIN_PWMSIP6_0               P6I_PWM_00a
#define	JS_HSIS_EVO__PWMPIN_PWMSIP6_1               P6I_PWM_01a
#define	JS_HSIS_EVO__PWMPIN_PWMSIP6_2               P6I_PWM_02a
#define	JS_HSIS_EVO__PWMPIN_PWMSIP6_3               P6I_PWM_03a

#define JS_HSIS_EVO__PWM_SIP6_WHEEL_LEFT_A      0
#define JS_HSIS_EVO__PWM_SIP6_WHEEL_LEFT_B      1
#define JS_HSIS_EVO__PWM_SIP6_WHEEL_RIGHT_A     2
#define JS_HSIS_EVO__PWM_SIP6_WHEEL_RIGHT_B     3

/* PWM generator on PCA96n */
#define JS_HSIS_EVO__PWM_GEN_JUMP_A      0
#define JS_HSIS_EVO__PWM_GEN_JUMP_B      1
#define JS_HSIS_EVO__PWM_GEN_HEADLIGHT_LEFT      2
#define JS_HSIS_EVO__PWM_GEN_HEADLIGHT_RIGHT     3

// ------------------------------------------------------------------------------------------

static __init int sysfs_jpsumo_init(void);
static __init void jpsumo_sys_set_bit(u32 reg, u32 bit, u32 val);

static struct parrot_aai_platform_data aai_platform_data;
static struct parrot_ultra_snd_platform_data ultra_snd_platform_data = {
	.gpio_mux_vbat_jump = -1,
	.gpio_mux_wheels = -1
};

typedef struct {
	char *model;
	int hwrev;
	int pcbrev;
} struct_jumpingsumo_hsis;

typedef struct {
	int gpio_INT_INERT;
	int gpio_VBUS_DETECT;
	int gpio_BUTTON;
	int gpio_NRST_CAM;
	int gpio_RED_LED_LEFT;
	int gpio_RED_LED_RIGHT;
	int gpio_GREEN_LED_LEFT;
	int gpio_GREEN_LED_RIGHT;
	int gpio_JUMP_CTRL_1;
	int gpio_JUMP_CTRL_2;
	int gpio_PWMGEN_nOE;
	int gpio_nAMP_PWDN;
	int gpio_MUX_VBAT_JUMP;
	int gpio_MUX_WHEELS;
	int gpio_nSLEEP_WHEEL;
	int gpio_nFAULT_WHEEL;
	int gpio_POWER_ON_OFF;
	int gpio_POWER_ON_OFF_on;
	int gpio_USB_MUX_CMD;
	int gpio_CHARGE_STATUS;
	int gpio_LED_IR;
	int gpio_DEV_1;
	int gpio_DEV_2;
	int gpio_DEV_3;
	int gpio_DEV_4;
	int gpio_WLAN_nPWD;
	int gpio_WLAN_WARM_RST;
	int gpio_JMP_SWITCH;
	int gpio_WiFi_RST;
	int gpio_MCU_RST;
	int gpio_MCU_IT;
} struct_jumpingsumo_hsis_gpio;

typedef struct {
	int pwm_sip6_wheel_left_a;
	int pwm_sip6_wheel_left_b;
	int pwm_sip6_wheel_right_a;
	int pwm_sip6_wheel_right_b;
} struct_jumpingsumo_hsis_pwm_sip6;

typedef struct {
	int pwm_gen_jump_a;
	int pwm_gen_jump_b;
	int pwm_gen_green_led_left;
	int pwm_gen_green_led_right;
	int pwm_gen_headlight_left;
	int pwm_gen_headlight_right;
	int pwm_gen_outdrv;
	int pwm_gen_headlight_complement;
} struct_jumpingsumo_hsis_pwm_gen;


static const struct_jumpingsumo_hsis jumpingsumo_default_hsis =
{
	.model = "Unknown",
	.hwrev = -1,
	.pcbrev = -1,
};

static const struct_jumpingsumo_hsis_pwm_gen jumpingsumo_hsis_pwm_gen_default =
{
	.pwm_gen_jump_a = -1,
	.pwm_gen_jump_b = -1,
	.pwm_gen_green_led_left = -1,
	.pwm_gen_green_led_right = -1,
	.pwm_gen_headlight_left = -1,
	.pwm_gen_headlight_right = -1,
	.pwm_gen_outdrv = -1,
	.pwm_gen_headlight_complement = -1,
};

static const struct_jumpingsumo_hsis_pwm_sip6 jumpingsumo_hsis_pwm_sip6_default =
{
	.pwm_sip6_wheel_left_a = -1,
	.pwm_sip6_wheel_left_b = -1,
	.pwm_sip6_wheel_right_a = -1,
	.pwm_sip6_wheel_right_b = -1,
};

static const struct_jumpingsumo_hsis_gpio jumpingsumo_hsis_gpio_default =
{
	.gpio_INT_INERT = -1,
	.gpio_VBUS_DETECT = -1,
	.gpio_BUTTON = -1,
	.gpio_NRST_CAM = -1,
	.gpio_RED_LED_LEFT = -1,
	.gpio_RED_LED_RIGHT = -1,
	.gpio_GREEN_LED_LEFT = -1,
	.gpio_GREEN_LED_RIGHT = -1,
	.gpio_JUMP_CTRL_1 = -1,
	.gpio_JUMP_CTRL_2 = -1,
	.gpio_PWMGEN_nOE = -1,
	.gpio_nAMP_PWDN = -1,
	.gpio_MUX_VBAT_JUMP = -1,
	.gpio_MUX_WHEELS = -1,
	.gpio_nSLEEP_WHEEL = -1,
	.gpio_nFAULT_WHEEL = -1,
	.gpio_POWER_ON_OFF = -1,
	.gpio_POWER_ON_OFF_on = -1,
	.gpio_USB_MUX_CMD = -1,
	.gpio_CHARGE_STATUS = -1,
	.gpio_LED_IR = -1,
	.gpio_DEV_1 = -1,
	.gpio_DEV_2 = -1,
	.gpio_DEV_3 = -1,
	.gpio_DEV_4 = -1,
	.gpio_WLAN_nPWD = -1,
	.gpio_WLAN_WARM_RST = -1,
	.gpio_JMP_SWITCH = -1,
	.gpio_WiFi_RST = -1,
	.gpio_MCU_RST = -1,
	.gpio_MCU_IT = -1,
};

static struct_jumpingsumo_hsis jumpingsumo_hsis = {0};
static struct_jumpingsumo_hsis_pwm_gen 	*jumpingsumo_hsis_pwm_gen = 	(struct_jumpingsumo_hsis_pwm_gen *)&jumpingsumo_hsis_pwm_gen_default;
static struct_jumpingsumo_hsis_pwm_sip6 *jumpingsumo_hsis_pwm_sip6 = 	(struct_jumpingsumo_hsis_pwm_sip6 *)&jumpingsumo_hsis_pwm_sip6_default;
static struct_jumpingsumo_hsis_gpio 	*jumpingsumo_hsis_gpio = 	(struct_jumpingsumo_hsis_gpio *)&jumpingsumo_hsis_gpio_default;

// ************************************

static const unsigned int pins_init_p6i_jpsumo_A1[] __initdata = {
	/* gpios */
	JS_HSIS_A1__GPIOPIN_LED_DEBUG,
	JS_HSIS_A1__GPIOPIN_SWITCH_POSITION_1   ,
	JS_HSIS_A1__GPIOPIN_SWITCH_POSITION_2   ,
	JS_HSIS_A1__GPIOPIN_JUMP_CTRL_1		 ,
	JS_HSIS_A1__GPIOPIN_JUMP_CTRL_2		 ,
	JS_HSIS_A1__GPIOPIN_CHARGE_STATUS	   ,
	JS_HSIS_A1__GPIOPIN_WLAN_nPWD		   ,
	JS_HSIS_A1__GPIOPIN_WLAN_WARM_RST	   ,
	JS_HSIS_A1__GPIOPIN_nFAULT_WHEEL		,
	JS_HSIS_A1__GPIOPIN_nSLEEP_WHEEL		,
	JS_HSIS_A1__GPIOPIN_PWMGEN_nOE		  ,
	JS_HSIS_A1__GPIOPIN_INT_INERT		   ,
	JS_HSIS_A1__GPIOPIN_nAMP_PWDN		   ,
	/* uart */
	P6I_UART1_DEFAULT					   ,
	/* i2c */
	P6I_I2CM0_DEFAULT					   ,
	/* sdio */
	P6I_SD0_DEFAULT						 ,
	/* rtc pwr on */
	P6I_RTC_PWR_ON						  ,
	0									   ,
};

static const unsigned int pins_init_p6i_jpsumo_A2[] __initdata = {
	/* gpios */
	JS_HSIS_A2__GPIOPIN_LED_IR		  ,
	JS_HSIS_A2__GPIOPIN_NRST_CAM_1	  ,
	JS_HSIS_A2__GPIOPIN_NRST_CAM		,
	JS_HSIS_A2__GPIOPIN_DEV_GPIO_1	  ,
	JS_HSIS_A2__GPIOPIN_DEV_GPIO_2	  ,
	JS_HSIS_A2__GPIOPIN_DEV_GPIO_3	  ,
	JS_HSIS_A2__GPIOPIN_DEV_GPIO_4	  ,
	JS_HSIS_A2__GPIOPIN_JUMP_CTRL_1	 ,
	JS_HSIS_A2__GPIOPIN_JUMP_CTRL_2	 ,
	JS_HSIS_A2__GPIOPIN_CHARGE_STATUS   ,
	JS_HSIS_A2__GPIOPIN_nFAULT_WHEEL	,
	JS_HSIS_A2__GPIOPIN_nSLEEP_WHEEL	,
	JS_HSIS_A2__GPIOPIN_PWMGEN_nOE	  ,
	JS_HSIS_A2__GPIOPIN_INT_INERT	   ,
	JS_HSIS_A2__GPIOPIN_nAMP_PWDN	   ,
	/* pwms */
	JS_HSIS_A2__PWMPIN_PWMSIP6_0		,
	JS_HSIS_A2__PWMPIN_PWMSIP6_1		,
	JS_HSIS_A2__PWMPIN_PWMSIP6_2		,
	JS_HSIS_A2__PWMPIN_PWMSIP6_3		,
	/* uart */
	P6I_UART1_DEFAULT				   ,
	/* i2c */
	P6I_I2CM0_DEFAULT				   ,
	/* rtc pwr on */
	P6I_RTC_PWR_ON					  ,
	0								   ,
};

static const unsigned int pins_init_p6i_jpsumo_A3[] __initdata = {
	/* gpios */
	JS_HSIS_A3__GPIOPIN_LED_IR			  ,
	JS_HSIS_A3__GPIOPIN_LEFT_EYE_LED		,
	JS_HSIS_A3__GPIOPIN_RIGHT_EYE_LED	   ,
	JS_HSIS_A3__GPIOPIN_NRST_CAM			,
	JS_HSIS_A3__GPIOPIN_DEV_GPIO_1		  ,
	JS_HSIS_A3__GPIOPIN_DEV_GPIO_2		  ,
	JS_HSIS_A3__GPIOPIN_DEV_GPIO_3		  ,
	JS_HSIS_A3__GPIOPIN_JUMP_CTRL_1		 ,
	JS_HSIS_A3__GPIOPIN_JUMP_CTRL_2		 ,
	JS_HSIS_A3__GPIOPIN_CHARGE_STATUS	   ,
	JS_HSIS_A3__GPIOPIN_nFAULT_WHEEL		,
	JS_HSIS_A3__GPIOPIN_nSLEEP_WHEEL		,
	JS_HSIS_A3__GPIOPIN_PWMGEN_nOE		  ,
	JS_HSIS_A3__GPIOPIN_INT_INERT		   ,
	JS_HSIS_A3__GPIOPIN_nAMP_PWDN		   ,
	JS_HSIS_A3__GPIOPIN_MUX_GPIO_ADC_IN0,
	JS_HSIS_A3__GPIOPIN_MUX_GPIO_ADC_IN1_2,
	/* pwms */
	JS_HSIS_A3__PWMPIN_PWMSIP6_0			,
	JS_HSIS_A3__PWMPIN_PWMSIP6_1			,
	JS_HSIS_A3__PWMPIN_PWMSIP6_2			,
	JS_HSIS_A3__PWMPIN_PWMSIP6_3			,
	/* uart */
	P6I_UART1_DEFAULT					   ,
	/* i2c */
	P6I_I2CM0_DEFAULT					   ,
	/* rtc pwr on */
	P6I_RTC_PWR_ON						  ,
	0									   ,
};

static const unsigned int pins_init_p6i_jpsumo_EVO_pcb00[] __initdata = {
	/* gpios */
	JS_HSIS_EVO__GPIOPIN_ON_OFF ,
	JS_HSIS_EVO__GPIOPIN_LEFT_EYE_LED_RED		,
	JS_HSIS_EVO__GPIOPIN_RIGHT_EYE_LED_RED	   ,
	JS_HSIS_EVO__GPIOPIN_LEFT_EYE_LED_GREEN		,
	JS_HSIS_EVO__GPIOPIN_RIGHT_EYE_LED_GREEN	   ,
	JS_HSIS_EVO__GPIOPIN_BUTTON			,
// 	JS_HSIS_EVO__GPIOPIN_DEV_GPIO_1		  ,
// 	JS_HSIS_EVO__GPIOPIN_DEV_GPIO_2		  ,
// 	JS_HSIS_EVO__GPIOPIN_DEV_GPIO_3		  ,
	JS_HSIS_EVO__GPIOPIN_JUMP_CTRL_1		 ,
	JS_HSIS_EVO__GPIOPIN_JUMP_CTRL_2		 ,
	JS_HSIS_EVO__GPIOPIN_nFAULT_WHEEL		,
	JS_HSIS_EVO__GPIOPIN_nSLEEP_WHEEL		,
	JS_HSIS_EVO__GPIOPIN_PWMGEN_nOE		  ,
	JS_HSIS_EVO__GPIOPIN_INT_INERT		   ,
	/* pwms */
	JS_HSIS_EVO__PWMPIN_PWMSIP6_0			,
	JS_HSIS_EVO__PWMPIN_PWMSIP6_1			,
	JS_HSIS_EVO__PWMPIN_PWMSIP6_2			,
	JS_HSIS_EVO__PWMPIN_PWMSIP6_3			,
	/* ... */
	JS_HSIS_EVO__GPIOPIN_MUX_GPIO_ADC_IN0,
	JS_HSIS_EVO__GPIOPIN_MUX_GPIO_ADC_IN1_2,
	JS_HSIS_EVO__GPIOPIN_MUX_GPIO_VBUS_DETECT,
	JS_HSIS_EVO__GPIOPIN_MUX_GPIO_USB_MUX_CMD,
	JS_HSIS_EVO__GPIOPIN_nAMP_PWDN,
	/* uart */
	P6I_UART1_DEFAULT					   ,
	/* i2c */
	P6I_I2CM0_DEFAULT					   ,
	/* sdio/mmc */
	P6I_SD0_DEFAULT						 ,
	/* rtc pwr on */
	P6I_RTC_PWR_ON						  ,
	0
};

static const unsigned int pins_init_p6i_jpsumo_EVO_pcb01[] __initdata = {
	/* gpios */
	GPIO_057,		/* INT_INERT */
	GPIO_051,		/* VBUSD_MISO */
	GPIO_050,		/* SWITCH_ON_MOSI */
	GPIO_040,		/* WEBCAM_RST */
	GPIO_043,		/* LEFT_EYE_LED_RED */
	GPIO_041,		/* RIGHT_EYE_LED_RED */
	GPIO_053,		/* LEFT_EYE_LED_GREEN */
	GPIO_052,		/* RIGHT_EYE_LED_GREEN */
	GPIO_038,		/* JUMP_CTRL_1 */
	GPIO_039,		/* JUMP_CTRL_2 */
	GPIO_042,		/* PWM_GEN_nOE */
	GPIO_045,		/* nAMP_PWDN */
	GPIO_049,		/* ADC_IN0_GPIO */
	GPIO_044,		/* ADC_IN1_2_GPIO */
	GPIO_055,		/* nSLEEP_WHEEL */
	GPIO_054,		/* nFAULT_WHEEL */
	GPIO_048,		/* SYS_OFF_SCK / SPI1_CLK */
	GPIO_058,		/* MB_USB_SWC */
	P6I_GPIO_011,		/* WiFi_RST */
	P6I_GPIO_012,		/* JMP_SWITCH */
	P6I_GPIO_002,		/* MCU_RST */
	/* pwms */
	P6I_PWM_00a,		/* PWM_SIP6_0 */
	P6I_PWM_01a,		/* PWM_SIP6_1 */
	P6I_PWM_02a,		/* PWM_SIP6_2 */
	P6I_PWM_03a,		/* PWM_SIP6_3 */
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

static const unsigned int pins_init_p6i_jpsumo_EVO_pcb02[] __initdata = {
	/* gpios */
	GPIO_057,		/* INT_INERT */
	GPIO_051,		/* VBUSD_MISO */
	GPIO_050,		/* SWITCH_ON_MOSI */
	GPIO_040,		/* WEBCAM_RST */
	GPIO_043,		/* LEFT_EYE_LED_RED */
	GPIO_041,		/* RIGHT_EYE_LED_RED */
	GPIO_053,		/* LEFT_EYE_LED_GREEN */
	GPIO_052,		/* RIGHT_EYE_LED_GREEN */
	GPIO_038,		/* JUMP_CTRL_1 */
	GPIO_039,		/* JUMP_CTRL_2 */
	GPIO_042,		/* PWM_GEN_nOE */
	GPIO_045,		/* nAMP_PWDN */
	GPIO_049,		/* ADC_IN0_GPIO */
	GPIO_044,		/* ADC_IN1_2_GPIO */
	GPIO_055,		/* nSLEEP_WHEEL */
	GPIO_056,		/* nFAULT_WHEEL */
	GPIO_048,		/* SYS_OFF_SCK / SPI1_CLK */
	GPIO_058,		/* MB_USB_SWC */
	P6I_GPIO_011,		/* WiFi_RST */
	P6I_GPIO_012,		/* JMP_SWITCH */
	P6I_GPIO_002,		/* MCU_RST */
	/* pwms */
	P6I_PWM_00a,		/* PWM_SIP6_0 */
	P6I_PWM_01a,		/* PWM_SIP6_1 */
	P6I_PWM_02a,		/* PWM_SIP6_2 */
	P6I_PWM_03a,		/* PWM_SIP6_3 */
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

static const unsigned int pins_init_p6i_jpsumo_v3_pcb00[] __initdata = {
	/* gpios */
	/* NA - ADC_IN0_GPIO */
	/* NA - ADC_IN1_2_GPIO */
	/* NA - VBUSD */
	/* NA - SWITCH_ON */
	/* NA - SYS_OFF */
	GPIO_057,		/* INT_INERT */
	GPIO_040,		/* WEBCAM_RST */
	GPIO_041,		/* LEFT_EYE_LED_RED */
	GPIO_043,		/* RIGHT_EYE_LED_RED */
	GPIO_052,		/* LEFT_EYE_LED_GREEN */
	GPIO_053,		/* RIGHT_EYE_LED_GREEN */
	GPIO_038,		/* JUMP_CTRL_1 */
	GPIO_039,		/* JUMP_CTRL_2 */
	GPIO_042,		/* PWM_GEN_nOE */
	GPIO_045,		/* nAMP_PWDN */
	GPIO_055,		/* nSLEEP_WHEEL */
	GPIO_056,		/* nFAULT_WHEEL */
	GPIO_058,		/* SIP6_USB_SWC */
	P6I_GPIO_011,		/* WiFi_RST */
	P6I_GPIO_012,		/* JMP_SWITCH */
	P6I_GPIO_002,		/* MCU_RST */
	P6I_GPIO_009,		/* MCU_IT */
	/* spi (comm with mcu) */
	P6I_SPI1_MISO,
	P6I_SPI1_MOSI,
	P6I_SPI1_CLK,
	/* pwms */
	P6I_PWM_00a,		/* PWM_SIP6_0 */
	P6I_PWM_01a,		/* PWM_SIP6_1 */
	P6I_PWM_02a,		/* PWM_SIP6_2 */
	P6I_PWM_03a,		/* PWM_SIP6_3 */
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

static struct platform_device *p6i_jpsumo_A1_devices[] = {
	&p6_uart1_device,
	&p6_nand_device,
	&p6_aai_device,
	&p6_i2cm0_device,
	&p6_sdhci0_device,
	&p6i_usb0_device,
	/* virtual device */
	&p6_dmac_device,
	&p6_gpio,
};

static struct platform_device *p6i_jpsumo_A2plus_devices[] = {
	&p6_uart1_device,
	&p6_nand_device,
	&p6_aai_device,
	&p6_us_device,
	&p6_i2cm0_device,
	&p6i_usb0_device,
	/* virtual device */
	&p6_dmac_device,
	&p6_gpio,
};

static struct platform_device *p6i_jpsumo_EVO_devices[] = {
	&p6_uart1_device,
	&p6_nand_device,
	&p6_aai_device,
	&p6_us_device,
	&p6_i2cm0_device,
	&p6_sdhci0_device,
	&p6i_usb0_device,
	/* virtual device */
	&p6_spi1_device,
	&p6_dmac_device,
	&p6_gpio
};

static struct platform_device *p6i_jpsumo_v3_devices[] = {
	&p6_uart1_device,
	&p6_nand_device,
	&p6_aai_device,
	&p6_us_device,
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
	.disable_dma = 1,
};

static struct mcu_minidrones3_key mcu_minidrones3_keys[] = {
	{ .bmask = 0x01, .input_key = KEY_0 },
	{ .bmask = 0x02, .input_key = KEY_1 },
};

static struct mcu_minidrones3_platform_data mcu_minidrones3_data  = {
	.spi_read_delay_ms = 2,
	.firmware_name = "attiny24a_fw.bin",
	.keys_nb = ARRAY_SIZE(mcu_minidrones3_keys),
	.keys = mcu_minidrones3_keys,
};

static struct spi_board_info spi_mcu_minidrones3_board_info[] __initconst = {
	{
		.modalias        = "mcu_minidrones3",
		.max_speed_hz    = 150 * 1000,
		.bus_num         = 1,
		.chip_select     = 0,
		.platform_data   = &mcu_minidrones3_data,
		.controller_data = &p6i_spi_controller_data,
		.mode            = SPI_MODE_0,
	},
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

static struct parrot_mmc_platform_data jpsumo_mmc_wifi_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	/* wifi : not cd and wp pins */
};

static struct parrot_mmc_platform_data jpsumo_mmc_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.4V ~ 3.2V card Vdd only */
	.wp_pin = -1,
	.cd_pin = -1,
};

// I2C 0 :
static struct parrot5_i2cm_platform i2cm_platform_js_i2c0 = {
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

static struct my_i2c_board_info jpsumo_i2c_devices[] = {
	{ /* IMU */
		.info = {
			I2C_BOARD_INFO("mpu6050", 0x68),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<JPSUMO_CLASSIC_HW_07)|
			(1<<JPSUMO_CLASSIC_HW_06)|
			(1<<JPSUMO_CLASSIC_HW_B2)|
			(1<<JPSUMO_CLASSIC_HW_B1)|
			(1<<JPSUMO_CLASSIC_HW_A3)|
			(1<<JPSUMO_CLASSIC_HW_A2)|
			(1<<JPSUMO_CLASSIC_HW_A1)|
			(1<<JPSUMO_EVO_HW_00)|
			(1<<JPSUMO_EVO_HW_01)|
			(1<<JPSUMO_EVO_HW_02)
	},
	{ /* IMU */
		.info = {
			I2C_BOARD_INFO("icm20608", 0x68),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<JPSUMO_V3_HW_00)
	},
	{ /* PCA9633 - pwm generator */
		.info = {
			I2C_BOARD_INFO("pca9633", 0x15),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<JPSUMO_CLASSIC_HW_07)|
			(1<<JPSUMO_CLASSIC_HW_06)|
			(1<<JPSUMO_CLASSIC_HW_B2)|
			(1<<JPSUMO_CLASSIC_HW_B1)|
			(1<<JPSUMO_CLASSIC_HW_A3)|
			(1<<JPSUMO_CLASSIC_HW_A2)|
			(1<<JPSUMO_CLASSIC_HW_A1)|
			(1<<JPSUMO_EVO_HW_00)|
			(1<<JPSUMO_EVO_HW_01)|
			(1<<JPSUMO_EVO_HW_02)
	},
	{ /* PCA9633 - pwm generator */
		.info = {
			I2C_BOARD_INFO("pca9633", 0x62),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<JPSUMO_V3_HW_00)
	},
	{ /* SMB1358 (qualcomm charger) */
		.info = {
			I2C_BOARD_INFO("smb1358", 0x1c),
			.irq=-1
		},
		.busnum = 0,
		.hwrev_mask =
			(1<<JPSUMO_EVO_HW_01)|
			(1<<JPSUMO_EVO_HW_02)|
			(1<<JPSUMO_V3_HW_00)
	},
};

static void  __init init_aai(void)
{
	printk(KERN_INFO "Initializing AAI ...\n");

	aai_platform_data.sync_freq = 48000;

	aai_platform_data.i2s_bit_clock_control = 0;
	aai_platform_data.i2s_master_clock_control = 0;

	p6_aai_device.dev.platform_data = &aai_platform_data;
}

static const struct_jumpingsumo_hsis_pwm_sip6 jumpingsumo_hsis_pwm_sip6_classic_pcbA1 =
{
	.pwm_sip6_wheel_left_a = JS_HSIS_A1__PWM_SIP6_WHEEL_LEFT_A,
	.pwm_sip6_wheel_left_b = JS_HSIS_A1__PWM_SIP6_WHEEL_LEFT_B,
	.pwm_sip6_wheel_right_a = JS_HSIS_A1__PWM_SIP6_WHEEL_RIGHT_A,
	.pwm_sip6_wheel_right_b = JS_HSIS_A1__PWM_SIP6_WHEEL_RIGHT_B,
};

static const struct_jumpingsumo_hsis_pwm_gen jumpingsumo_hsis_pwm_gen_classic_pcbA1 =
{
	.pwm_gen_jump_a = JS_HSIS_A1__PWM_GEN_JUMP_A,
	.pwm_gen_jump_b = JS_HSIS_A1__PWM_GEN_JUMP_B,
	.pwm_gen_headlight_left = -1,
	.pwm_gen_headlight_right = -1,
	.pwm_gen_green_led_left = -1,
	.pwm_gen_green_led_right = -1,
	.pwm_gen_outdrv = 0,
	.pwm_gen_headlight_complement = -1,
};

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

static void __init jpsumo_reserve_mem(void)
{
	ramoops_data.mem_address = meminfo.bank[0].start +
		meminfo.bank[0].size - RAMOOPS_SIZE;

	memblock_reserve(&ramoops_data.mem_address,
			&ramoops_data.mem_size);
}

static void __init jpsumo_ramoops_init(void)
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

static const struct_jumpingsumo_hsis_gpio jumpingsumo_hsis_gpio_classic_pcbA1 =
{
	.gpio_INT_INERT = JS_HSIS_A1__GPIO_INT_INERT,
	.gpio_VBUS_DETECT = -1,
	.gpio_BUTTON = -1,
	.gpio_NRST_CAM = -1,
	.gpio_RED_LED_LEFT = -1,
	.gpio_RED_LED_RIGHT = -1,
	.gpio_GREEN_LED_LEFT = -1,
	.gpio_GREEN_LED_RIGHT = -1,
	.gpio_JUMP_CTRL_1 = JS_HSIS_A1__GPIO_JUMP_CTRL_1,
	.gpio_JUMP_CTRL_2 = JS_HSIS_A1__GPIO_JUMP_CTRL_2,
	.gpio_PWMGEN_nOE = JS_HSIS_A1__GPIO_PWMGEN_nOE,
	.gpio_nAMP_PWDN = JS_HSIS_A1__GPIO_nAMP_PWDN,
	.gpio_MUX_VBAT_JUMP = -1,
	.gpio_MUX_WHEELS = -1,
	.gpio_nSLEEP_WHEEL = JS_HSIS_A1__GPIO_nSLEEP_WHEEL,
	.gpio_nFAULT_WHEEL = JS_HSIS_A1__GPIO_nFAULT_WHEEL,
	.gpio_POWER_ON_OFF = -1,
	.gpio_POWER_ON_OFF_on = -1,
	.gpio_USB_MUX_CMD = -1,
	.gpio_CHARGE_STATUS = -1,
	.gpio_LED_IR = -1,
	.gpio_DEV_1 = -1,
	.gpio_DEV_2 = -1,
	.gpio_DEV_3 = -1,
	.gpio_DEV_4 = -1,
	.gpio_WLAN_nPWD = JS_HSIS_A1__GPIO_WLAN_nPWD,
	.gpio_WLAN_WARM_RST = JS_HSIS_A1__GPIO_WLAN_WARM_RST,
	.gpio_JMP_SWITCH = -1,
	.gpio_WiFi_RST = -1,
	.gpio_MCU_RST = -1,
	.gpio_MCU_IT = -1,
};

/* Classic PCB A2 */
static const struct_jumpingsumo_hsis_pwm_sip6 jumpingsumo_hsis_pwm_sip6_classic_pcbA2 =
{
	.pwm_sip6_wheel_left_a = JS_HSIS_A2__PWM_SIP6_WHEEL_LEFT_A,
	.pwm_sip6_wheel_left_b = JS_HSIS_A2__PWM_SIP6_WHEEL_LEFT_B,
	.pwm_sip6_wheel_right_a = JS_HSIS_A2__PWM_SIP6_WHEEL_RIGHT_A,
	.pwm_sip6_wheel_right_b = JS_HSIS_A2__PWM_SIP6_WHEEL_RIGHT_B,
};

static const struct_jumpingsumo_hsis_pwm_gen jumpingsumo_hsis_pwm_gen_classic_pcbA2 =
{
	.pwm_gen_jump_a = JS_HSIS_A2__PWM_GEN_JUMP_A,
	.pwm_gen_jump_b = JS_HSIS_A2__PWM_GEN_JUMP_B,
	.pwm_gen_headlight_left = -1,
	.pwm_gen_headlight_right = -1,
	.pwm_gen_green_led_left = -1,
	.pwm_gen_green_led_right = -1,
	.pwm_gen_outdrv = 0,
	.pwm_gen_headlight_complement = -1,
};

static const struct_jumpingsumo_hsis_gpio jumpingsumo_hsis_gpio_classic_pcbA2 =
{
	.gpio_INT_INERT = JS_HSIS_A2__GPIO_INT_INERT,
	.gpio_VBUS_DETECT = -1,
	.gpio_BUTTON = -1,
	.gpio_NRST_CAM = JS_HSIS_A2__GPIO_NRST_CAM,
	.gpio_RED_LED_LEFT = -1,
	.gpio_RED_LED_RIGHT = -1,
	.gpio_GREEN_LED_LEFT = -1,
	.gpio_GREEN_LED_RIGHT = -1,
	.gpio_JUMP_CTRL_1 = JS_HSIS_A2__GPIO_JUMP_CTRL_1,
	.gpio_JUMP_CTRL_2 = JS_HSIS_A2__GPIO_JUMP_CTRL_2,
	.gpio_PWMGEN_nOE = JS_HSIS_A2__GPIO_PWMGEN_nOE,
	.gpio_nAMP_PWDN = JS_HSIS_A2__GPIO_nAMP_PWDN,
	.gpio_MUX_VBAT_JUMP = -1,
	.gpio_MUX_WHEELS = -1,
	.gpio_nSLEEP_WHEEL = JS_HSIS_A2__GPIO_nSLEEP_WHEEL,
	.gpio_nFAULT_WHEEL = JS_HSIS_A2__GPIO_nFAULT_WHEEL,
	.gpio_POWER_ON_OFF = -1,
	.gpio_POWER_ON_OFF_on = -1,
	.gpio_USB_MUX_CMD = -1,
	.gpio_CHARGE_STATUS = JS_HSIS_A2__GPIO_CHARGE_STATUS,
	.gpio_LED_IR = JS_HSIS_A2__GPIO_LED_IR,
	.gpio_DEV_1 = JS_HSIS_A2__GPIO_DEV_GPIO_1,
	.gpio_DEV_2 = JS_HSIS_A2__GPIO_DEV_GPIO_2,
	.gpio_DEV_3 = JS_HSIS_A2__GPIO_DEV_GPIO_3,
	.gpio_DEV_4 = JS_HSIS_A2__GPIO_DEV_GPIO_4,
	.gpio_WLAN_nPWD = -1,				/* Only used on js classic proto A1 */
	.gpio_WLAN_WARM_RST = -1,			/* Only used on js classic proto A1 */
	.gpio_JMP_SWITCH = -1,
	.gpio_WiFi_RST = -1,
	.gpio_MCU_RST = -1,
	.gpio_MCU_IT = -1,
};

/* Classic PCB A3 and more */
static const struct_jumpingsumo_hsis_pwm_sip6 jumpingsumo_hsis_pwm_sip6_classic_pcbA3 =
{
	.pwm_sip6_wheel_left_a = JS_HSIS_A3__PWM_SIP6_WHEEL_LEFT_A,
	.pwm_sip6_wheel_left_b = JS_HSIS_A3__PWM_SIP6_WHEEL_LEFT_B,
	.pwm_sip6_wheel_right_a = JS_HSIS_A3__PWM_SIP6_WHEEL_RIGHT_A,
	.pwm_sip6_wheel_right_b = JS_HSIS_A3__PWM_SIP6_WHEEL_RIGHT_B,
};

static const struct_jumpingsumo_hsis_pwm_gen jumpingsumo_hsis_pwm_gen_classic_pcbA3 =
{
	.pwm_gen_jump_a = JS_HSIS_A3__PWM_GEN_JUMP_A,
	.pwm_gen_jump_b = JS_HSIS_A3__PWM_GEN_JUMP_B,
	.pwm_gen_headlight_left = -1,
	.pwm_gen_headlight_right = -1,
	.pwm_gen_green_led_left = JS_HSIS_A3__PWM_GEN_LEFT_EYE_LED_GREEN,
	.pwm_gen_green_led_right = JS_HSIS_A3__PWM_GEN_RIGHT_EYE_LED_GREEN,
	.pwm_gen_outdrv = 0,
	.pwm_gen_headlight_complement = -1,
};

static const struct_jumpingsumo_hsis_gpio jumpingsumo_hsis_gpio_classic_pcbA3 =
{
	.gpio_INT_INERT = JS_HSIS_A3__GPIO_INT_INERT,
	.gpio_VBUS_DETECT = -1,
	.gpio_BUTTON = -1,
	.gpio_NRST_CAM = JS_HSIS_A3__GPIO_NRST_CAM,
	.gpio_RED_LED_LEFT = JS_HSIS_A3__GPIO_LEFT_EYE_LED,
	.gpio_RED_LED_RIGHT = JS_HSIS_A3__GPIO_RIGHT_EYE_LED,
	.gpio_GREEN_LED_LEFT = -1,
	.gpio_GREEN_LED_RIGHT = -1,
	.gpio_JUMP_CTRL_1 = JS_HSIS_A3__GPIO_JUMP_CTRL_1,
	.gpio_JUMP_CTRL_2 = JS_HSIS_A3__GPIO_JUMP_CTRL_2,
	.gpio_PWMGEN_nOE = JS_HSIS_A3__GPIO_PWMGEN_nOE,
	.gpio_nAMP_PWDN = JS_HSIS_A3__GPIO_nAMP_PWDN,
	.gpio_MUX_VBAT_JUMP = JS_HSIS_A3__GPIO_MUX_GPIO_ADC_IN0,
	.gpio_MUX_WHEELS = JS_HSIS_A3__GPIO_MUX_GPIO_ADC_IN1_2,
	.gpio_nSLEEP_WHEEL = JS_HSIS_A3__GPIO_nSLEEP_WHEEL,
	.gpio_nFAULT_WHEEL = JS_HSIS_A3__GPIO_nFAULT_WHEEL,
	.gpio_POWER_ON_OFF = -1,
	.gpio_POWER_ON_OFF_on = -1,
	.gpio_USB_MUX_CMD = -1,
	.gpio_CHARGE_STATUS = JS_HSIS_A3__GPIO_CHARGE_STATUS,
	.gpio_LED_IR = JS_HSIS_A3__GPIO_LED_IR,
	.gpio_DEV_1 = JS_HSIS_A3__GPIO_DEV_GPIO_1,
	.gpio_DEV_2 = JS_HSIS_A3__GPIO_DEV_GPIO_2,
	.gpio_DEV_3 = JS_HSIS_A3__GPIO_DEV_GPIO_3,
	.gpio_DEV_4 = -1,
	.gpio_WLAN_nPWD = -1,				/* Only used on js classic proto A1 */
	.gpio_WLAN_WARM_RST = -1,			/* Only used on js classic proto A1 */
	.gpio_JMP_SWITCH = -1,
	.gpio_WiFi_RST = -1,
	.gpio_MCU_RST = -1,
	.gpio_MCU_IT = -1,
};

static const struct_jumpingsumo_hsis_pwm_sip6 jumpingsumo_hsis_pwm_sip6_evo_pcb00 =
{
	.pwm_sip6_wheel_left_a = JS_HSIS_EVO__PWM_SIP6_WHEEL_LEFT_A,
	.pwm_sip6_wheel_left_b = JS_HSIS_EVO__PWM_SIP6_WHEEL_LEFT_B,
	.pwm_sip6_wheel_right_a = JS_HSIS_EVO__PWM_SIP6_WHEEL_RIGHT_A,
	.pwm_sip6_wheel_right_b = JS_HSIS_EVO__PWM_SIP6_WHEEL_RIGHT_B,
};

static const struct_jumpingsumo_hsis_pwm_sip6 jumpingsumo_hsis_pwm_sip6_evo_pcb01 =
{
	.pwm_sip6_wheel_left_a = 0,	/* PWM_SIP6_0 */
	.pwm_sip6_wheel_left_b = 1,	/* PWM_SIP6_1 */
	.pwm_sip6_wheel_right_a = 2,	/* PWM_SIP6_2 */
	.pwm_sip6_wheel_right_b = 3,	/* PWM_SIP6_3 */
};

static const struct_jumpingsumo_hsis_pwm_sip6 jumpingsumo_hsis_pwm_sip6_evo_pcb02 =
{
	.pwm_sip6_wheel_left_a = 0,	/* PWM_SIP6_0 */
	.pwm_sip6_wheel_left_b = 1,	/* PWM_SIP6_1 */
	.pwm_sip6_wheel_right_a = 2,	/* PWM_SIP6_2 */
	.pwm_sip6_wheel_right_b = 3,	/* PWM_SIP6_3 */
};

static const struct_jumpingsumo_hsis_pwm_sip6 jumpingsumo_hsis_pwm_sip6_v3_pcb00 =
{
	.pwm_sip6_wheel_left_a = 0,	/* PWM_SIP6_0 */
	.pwm_sip6_wheel_left_b = 1,	/* PWM_SIP6_1 */
	.pwm_sip6_wheel_right_a = 2,	/* PWM_SIP6_2 */
	.pwm_sip6_wheel_right_b = 3,	/* PWM_SIP6_3 */
};

static const struct_jumpingsumo_hsis_pwm_gen jumpingsumo_hsis_pwm_gen_evo_pcb00 =
{
	.pwm_gen_jump_a = JS_HSIS_EVO__PWM_GEN_JUMP_A,
	.pwm_gen_jump_b = JS_HSIS_EVO__PWM_GEN_JUMP_B,
	.pwm_gen_headlight_left = JS_HSIS_EVO__PWM_GEN_HEADLIGHT_LEFT,
	.pwm_gen_headlight_right = JS_HSIS_EVO__PWM_GEN_HEADLIGHT_RIGHT,
	.pwm_gen_green_led_left = -1,
	.pwm_gen_green_led_right = -1,
	.pwm_gen_outdrv = 0,
	.pwm_gen_headlight_complement = 1,
};

static const struct_jumpingsumo_hsis_pwm_gen jumpingsumo_hsis_pwm_gen_evo_pcb01 =
{
	.pwm_gen_jump_a = 0,		/* PWM_GEN_J_A = PWM_GEN_0 */
	.pwm_gen_jump_b = 1,		/* PWM_GEN_J_B = PWM_GEN_1 */
	.pwm_gen_headlight_left = 2,	/* PWM_GEN_2 */
	.pwm_gen_headlight_right = 3,	/* PWM_GEN_3 */
	.pwm_gen_green_led_left = -1,
	.pwm_gen_green_led_right = -1,
	.pwm_gen_outdrv = 0,
	.pwm_gen_headlight_complement = 0,
};

static const struct_jumpingsumo_hsis_pwm_gen jumpingsumo_hsis_pwm_gen_evo_pcb02 =
{
	.pwm_gen_jump_a = 0,		/* PWM_GEN_J_A = PWM_GEN_0 */
	.pwm_gen_jump_b = 1,		/* PWM_GEN_J_B = PWM_GEN_1 */
	.pwm_gen_headlight_left = 2,	/* PWM_GEN_2 */
	.pwm_gen_headlight_right = 3,	/* PWM_GEN_3 */
	.pwm_gen_green_led_left = -1,
	.pwm_gen_green_led_right = -1,
	.pwm_gen_outdrv = 0,
	.pwm_gen_headlight_complement = 0,
};

static const struct_jumpingsumo_hsis_pwm_gen jumpingsumo_hsis_pwm_gen_v3_pcb00 =
{
	.pwm_gen_jump_a = 0,		/* PWM_GEN_J_A = PWM_GEN_0 */
	.pwm_gen_jump_b = 1,		/* PWM_GEN_J_B = PWM_GEN_1 */
	.pwm_gen_headlight_left = -1,	/* PWM_GEN_2 */
	.pwm_gen_headlight_right = -1,	/* PWM_GEN_3 */
	.pwm_gen_green_led_left = -1,
	.pwm_gen_green_led_right = -1,
	.pwm_gen_outdrv = 0,
	.pwm_gen_headlight_complement = 0,
};

static const struct_jumpingsumo_hsis_gpio jumpingsumo_hsis_gpio_evo_pcb00 =
{
	.gpio_INT_INERT = JS_HSIS_EVO__GPIO_INT_INERT,
	.gpio_VBUS_DETECT = JS_HSIS_EVO__GPIO_MUX_GPIO_VBUS_DETECT,
	.gpio_BUTTON = JS_HSIS_EVO__GPIO_BUTTON,
	.gpio_NRST_CAM = -1,
	.gpio_RED_LED_LEFT = JS_HSIS_EVO__GPIO_LEFT_EYE_LED_RED,
	.gpio_RED_LED_RIGHT = JS_HSIS_EVO__GPIO_RIGHT_EYE_LED_RED,
	.gpio_GREEN_LED_LEFT = JS_HSIS_EVO__GPIO_LEFT_EYE_LED_GREEN,
	.gpio_GREEN_LED_RIGHT = JS_HSIS_EVO__GPIO_RIGHT_EYE_LED_GREEN,
	.gpio_JUMP_CTRL_1 = JS_HSIS_EVO__GPIO_JUMP_CTRL_1,
	.gpio_JUMP_CTRL_2 = JS_HSIS_EVO__GPIO_JUMP_CTRL_2,
	.gpio_PWMGEN_nOE = JS_HSIS_EVO__GPIO_PWMGEN_nOE,
	.gpio_nAMP_PWDN = JS_HSIS_EVO__GPIO_nAMP_PWDN,
	.gpio_MUX_VBAT_JUMP = JS_HSIS_EVO__GPIO_MUX_GPIO_ADC_IN0,
	.gpio_MUX_WHEELS = JS_HSIS_EVO__GPIO_MUX_GPIO_ADC_IN1_2,
	.gpio_nSLEEP_WHEEL = JS_HSIS_EVO__GPIO_nSLEEP_WHEEL,
	.gpio_nFAULT_WHEEL = JS_HSIS_EVO__GPIO_nFAULT_WHEEL,
	.gpio_POWER_ON_OFF = JS_HSIS_EVO__GPIO_ON_OFF,
	.gpio_POWER_ON_OFF_on = 0,
	.gpio_USB_MUX_CMD = JS_HSIS_EVO__GPIO_MUX_GPIO_USB_MUX_CMD,
	.gpio_CHARGE_STATUS = -1,
	.gpio_LED_IR = -1,
	.gpio_DEV_1 = -1,
	.gpio_DEV_2 = -1,
	.gpio_DEV_3 = -1,
	.gpio_DEV_4 = -1,
	.gpio_WLAN_nPWD = -1,		/* Only used on js classic proto A1 */
	.gpio_WLAN_WARM_RST = -1,	/* Only used on js classic proto A1 */
	.gpio_JMP_SWITCH = -1,
	.gpio_WiFi_RST = -1,
	.gpio_MCU_RST = -1,
	.gpio_MCU_IT = -1,
};

static const struct_jumpingsumo_hsis_gpio jumpingsumo_hsis_gpio_evo_pcb01 = {
	.gpio_INT_INERT = 57,		/* INT_INERT */
	.gpio_VBUS_DETECT = 51,		/* VBUSD_MISO */
	.gpio_BUTTON = 50,		/* SWITCH_ON_MOSI */
	.gpio_NRST_CAM = 40,		/* WEBCAM_RST */
	.gpio_RED_LED_LEFT = 43,	/* LEFT_EYE_LED_RED */
	.gpio_RED_LED_RIGHT = 41,	/* RIGHT_EYE_LED_RED */
	.gpio_GREEN_LED_LEFT = 53,	/* LEFT_EYE_LED_GREEN */
	.gpio_GREEN_LED_RIGHT = 52,	/* RIGHT_EYE_LED_GREEN */
	.gpio_JUMP_CTRL_1 = 38,		/* JUMP_CTRL_1 */
	.gpio_JUMP_CTRL_2 = 39,		/* JUMP_CTRL_2 */
	.gpio_PWMGEN_nOE = 42,		/* PWM_GEN_nOE */
	.gpio_nAMP_PWDN = 45,		/* nAMP_PWDN */
	.gpio_MUX_VBAT_JUMP = 49,	/* ADC_IN0_GPIO */
	.gpio_MUX_WHEELS = 44,		/* ADC_IN1_2_GPIO */
	.gpio_nSLEEP_WHEEL = 55,	/* nSLEEP_WHEEL */
	.gpio_nFAULT_WHEEL = 54,	/* nFAULT_WHEEL */
	.gpio_POWER_ON_OFF = 48,	/* SYS_OFF_SCK */
	.gpio_POWER_ON_OFF_on = 0,
	.gpio_USB_MUX_CMD = 58,		/* MB_USB_SWC */
	.gpio_CHARGE_STATUS = -1,	/* Not Connected */
	.gpio_LED_IR = -1,
	.gpio_DEV_1 = -1,
	.gpio_DEV_2 = -1,
	.gpio_DEV_3 = -1,
	.gpio_DEV_4 = -1,
	.gpio_WLAN_nPWD = -1,		/* Only used on js classic proto A1 */
	.gpio_WLAN_WARM_RST = -1,	/* Only used on js classic proto A1 */
	.gpio_JMP_SWITCH = 12,
	.gpio_WiFi_RST = 11,
	.gpio_MCU_RST = 2,
	.gpio_MCU_IT = -1,
};

static const struct_jumpingsumo_hsis_gpio jumpingsumo_hsis_gpio_evo_pcb02 = {
	.gpio_INT_INERT = 57,		/* INT_INERT */
	.gpio_VBUS_DETECT = 51,		/* VBUSD_MISO */
	.gpio_BUTTON = 50,		/* SWITCH_ON_MOSI */
	.gpio_NRST_CAM = 40,		/* WEBCAM_RST */
	.gpio_RED_LED_LEFT = 43,	/* LEFT_EYE_LED_RED */
	.gpio_RED_LED_RIGHT = 41,	/* RIGHT_EYE_LED_RED */
	.gpio_GREEN_LED_LEFT = 53,	/* LEFT_EYE_LED_GREEN */
	.gpio_GREEN_LED_RIGHT = 52,	/* RIGHT_EYE_LED_GREEN */
	.gpio_JUMP_CTRL_1 = 38,		/* JUMP_CTRL_1 */
	.gpio_JUMP_CTRL_2 = 39,		/* JUMP_CTRL_2 */
	.gpio_PWMGEN_nOE = 42,		/* PWM_GEN_nOE */
	.gpio_nAMP_PWDN = 45,		/* nAMP_PWDN */
	.gpio_MUX_VBAT_JUMP = 49,	/* ADC_IN0_GPIO */
	.gpio_MUX_WHEELS = 44,		/* ADC_IN1_2_GPIO */
	.gpio_nSLEEP_WHEEL = 55,	/* nSLEEP_WHEEL */
	.gpio_nFAULT_WHEEL = 56,	/* nFAULT_WHEEL */
	.gpio_POWER_ON_OFF = 48,	/* SYS_OFF_SCK */
	.gpio_POWER_ON_OFF_on = 0,
	.gpio_USB_MUX_CMD = 58,		/* MB_USB_SWC */
	.gpio_CHARGE_STATUS = -1,	/* Not Connected */
	.gpio_LED_IR = -1,
	.gpio_DEV_1 = -1,
	.gpio_DEV_2 = -1,
	.gpio_DEV_3 = -1,
	.gpio_DEV_4 = -1,
	.gpio_WLAN_nPWD = -1,		/* Only used on js classic proto A1 */
	.gpio_WLAN_WARM_RST = -1,	/* Only used on js classic proto A1 */
	.gpio_JMP_SWITCH = 12,
	.gpio_WiFi_RST = 11,
	.gpio_MCU_RST = 2,
	.gpio_MCU_IT = -1,
};

static const struct_jumpingsumo_hsis_gpio jumpingsumo_hsis_gpio_v3_pcb00 = {
	.gpio_INT_INERT = 57,		/* INT_INERT */
	.gpio_VBUS_DETECT = -1,		/* VBUSD_MISO */
	.gpio_BUTTON = -1,		/* SWITCH_ON_MOSI */
	.gpio_NRST_CAM = 40,		/* WEBCAM_RST */
	.gpio_RED_LED_LEFT = 41,	/* LEFT_EYE_LED_RED */
	.gpio_RED_LED_RIGHT = 43,	/* RIGHT_EYE_LED_RED */
	.gpio_GREEN_LED_LEFT = 52,	/* LEFT_EYE_LED_GREEN */
	.gpio_GREEN_LED_RIGHT = 53,	/* RIGHT_EYE_LED_GREEN */
	.gpio_JUMP_CTRL_1 = 38,		/* JUMP_CTRL_1 */
	.gpio_JUMP_CTRL_2 = 39,		/* JUMP_CTRL_2 */
	.gpio_PWMGEN_nOE = 42,		/* PWM_GEN_nOE */
	.gpio_nAMP_PWDN = 45,		/* nAMP_PWDN */
	.gpio_MUX_VBAT_JUMP = -1,	/* ADC_IN0_GPIO */
	.gpio_MUX_WHEELS = -1,		/* ADC_IN1_2_GPIO */
	.gpio_nSLEEP_WHEEL = 55,	/* nSLEEP_WHEEL */
	.gpio_nFAULT_WHEEL = 56,	/* nFAULT_WHEEL */
	.gpio_POWER_ON_OFF = -1,	/* SYS_OFF_SCK */
	.gpio_POWER_ON_OFF_on = 0,
	.gpio_USB_MUX_CMD = 58,		/* SIP6_USB_SWC */
	.gpio_CHARGE_STATUS = -1,	/* Not Connected */
	.gpio_LED_IR = -1,
	.gpio_DEV_1 = -1,
	.gpio_DEV_2 = -1,
	.gpio_DEV_3 = -1,
	.gpio_DEV_4 = -1,
	.gpio_WLAN_nPWD = -1,		/* Only used on js classic proto A1 */
	.gpio_WLAN_WARM_RST = -1,	/* Only used on js classic proto A1 */
	.gpio_JMP_SWITCH = 12,		/* JUMP_SWITCH */
	.gpio_WiFi_RST = 11,		/* WiFi_RST */
	.gpio_MCU_RST = 2,		/* SIP6_MCU_RST */
	.gpio_MCU_IT = 9,
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

static void __init jpsumo_sys_set_bit(u32 reg, u32 bit, u32 val)
{
	u32 value = __raw_readl(PARROT6_VA_SYS + reg);
	value = (value & ~(1<<bit)) | ((val & 0x1) << bit);
	__raw_writel(value, PARROT6_VA_SYS + reg);
}

static void jpsumo_power_off(void)
{
	int gpio = jumpingsumo_hsis_gpio->gpio_POWER_ON_OFF;
	int val;
	val = gpio_get_value(gpio);
	gpio_set_value(gpio, val ? 0 : 1);
}

static void __init jpsumo_init(jpsumo_type_t jpsumo_type)
{
	int i;

	sip6_init(0);
	platform_device_register(&user_gpio);

	jumpingsumo_hsis = jumpingsumo_default_hsis;
	jumpingsumo_hsis.pcbrev = parrot_board_rev();
	switch (jpsumo_type) {
		case JPSUMO_CLASSIC:
			jumpingsumo_hsis.model = "Classic";
			break;
		case JPSUMO_EVO:
			jumpingsumo_hsis.model = "EVO";
			break;
		default:
		case JPSUMO_V3:
			jumpingsumo_hsis.model = "V3";
			break;
	}

#ifdef CONFIG_RAMOOPS
	jpsumo_ramoops_init();
#endif

	printk(KERN_INFO " ==== Jumping Sumo %s (PCB rev : %d) ====\n", jumpingsumo_hsis.model, jumpingsumo_hsis.pcbrev);

	switch ( jpsumo_type ) {
		default:
		case JPSUMO_V3:
			switch ( jumpingsumo_hsis.pcbrev )
			{
				default:
					printk(KERN_INFO "WARNING : unknown and/or not supported V3 board version\n");
					printk(KERN_INFO "value of pcb version : %x\n", jumpingsumo_hsis.pcbrev);
					printk(KERN_INFO "We're assuming that hardware version of this board is similar to the last hardware version managed by this BSP\n");
					/* There is no break, it's normal... */
				case JPSUMO_V3_PCB_00:
					jumpingsumo_hsis.hwrev = JPSUMO_V3_HW_00;
					printk(KERN_INFO "V3 hardware version 00 used (%d detected)\n", jumpingsumo_hsis.pcbrev);
					jumpingsumo_hsis_gpio = (struct_jumpingsumo_hsis_gpio *)&jumpingsumo_hsis_gpio_v3_pcb00;
					jumpingsumo_hsis_pwm_gen = (struct_jumpingsumo_hsis_pwm_gen *)&jumpingsumo_hsis_pwm_gen_v3_pcb00;
					jumpingsumo_hsis_pwm_sip6 = (struct_jumpingsumo_hsis_pwm_sip6 *)&jumpingsumo_hsis_pwm_sip6_v3_pcb00;
					parrot_init_pin(pins_init_p6i_jpsumo_v3_pcb00);

					// Pull Up on UART1 RX:
					p6i_set_pads_uart1_pullup();
					break;
			}
			break;
		case JPSUMO_EVO:
			switch ( jumpingsumo_hsis.pcbrev )
			{
				default:
					printk(KERN_INFO "WARNING : unknown and/or not supported EVO board version\n");
					printk(KERN_INFO "value of pcb version : %x\n", jumpingsumo_hsis.pcbrev);
					printk(KERN_INFO "We're assuming that hardware version of this board is similar to the last hardware version managed by this BSP\n");
					/* There is no break, it's normal... */
				case JPSUMO_EVO_PCB_02:
					jumpingsumo_hsis.hwrev = JPSUMO_EVO_HW_02;
					printk(KERN_INFO "EVO hardware version 02 used (%d detected)\n", jumpingsumo_hsis.pcbrev);
					jumpingsumo_hsis_gpio = (struct_jumpingsumo_hsis_gpio *)&jumpingsumo_hsis_gpio_evo_pcb02;
					jumpingsumo_hsis_pwm_gen = (struct_jumpingsumo_hsis_pwm_gen *)&jumpingsumo_hsis_pwm_gen_evo_pcb02;
					jumpingsumo_hsis_pwm_sip6 = (struct_jumpingsumo_hsis_pwm_sip6 *)&jumpingsumo_hsis_pwm_sip6_evo_pcb02;
					parrot_init_pin(pins_init_p6i_jpsumo_EVO_pcb02);
					platform_device_register(&p6_dynamic_pinmux_pic_prog);

					// Pull Up on UART1 RX:
					p6i_set_pads_uart1_pullup();
					break;
				case JPSUMO_EVO_PCB_00: // XXX : When HW00 do as if HW01 because of small hw issue.
				case JPSUMO_EVO_PCB_01:
					if (jumpingsumo_hsis.pcbrev == JPSUMO_EVO_PCB_00)
					{
						printk(KERN_INFO "**********************************************\n");
						printk(KERN_INFO "***   EVO hardware version 00 detected     ***\n");
						printk(KERN_INFO "***         But do as if HW01              ***\n");
						printk(KERN_INFO "***     because of small hw issue          ***\n");
						printk(KERN_INFO "**********************************************\n");
					}
					jumpingsumo_hsis.hwrev = JPSUMO_EVO_HW_01;
					printk(KERN_INFO "EVO hardware version 01 used (%d detected)\n", jumpingsumo_hsis.pcbrev);
					jumpingsumo_hsis_gpio = (struct_jumpingsumo_hsis_gpio *)&jumpingsumo_hsis_gpio_evo_pcb01;
					jumpingsumo_hsis_pwm_gen = (struct_jumpingsumo_hsis_pwm_gen *)&jumpingsumo_hsis_pwm_gen_evo_pcb01;
					jumpingsumo_hsis_pwm_sip6 = (struct_jumpingsumo_hsis_pwm_sip6 *)&jumpingsumo_hsis_pwm_sip6_evo_pcb01;

					parrot_init_pin(pins_init_p6i_jpsumo_EVO_pcb01);
					platform_device_register(&p6_dynamic_pinmux_pic_prog);

					// Pull Up on UART1 RX:
					p6i_set_pads_uart1_pullup();
					break;
			}
			break;

		case JPSUMO_CLASSIC:
			switch ( jumpingsumo_hsis.pcbrev ) {
				case JPSUMO_CLASSIC_PCB_A1:
					jumpingsumo_hsis.hwrev = JPSUMO_CLASSIC_HW_A1;
					printk(KERN_INFO "Classic hardware version A1 detected\n");
					jumpingsumo_hsis_gpio = (struct_jumpingsumo_hsis_gpio *)&jumpingsumo_hsis_gpio_classic_pcbA1;
					jumpingsumo_hsis_pwm_gen = (struct_jumpingsumo_hsis_pwm_gen *)&jumpingsumo_hsis_pwm_gen_classic_pcbA1;
					jumpingsumo_hsis_pwm_sip6 = (struct_jumpingsumo_hsis_pwm_sip6 *)&jumpingsumo_hsis_pwm_sip6_classic_pcbA1;
					parrot_init_pin(pins_init_p6i_jpsumo_A1);
					break;

				case JPSUMO_CLASSIC_PCB_A2:
					jumpingsumo_hsis.hwrev = JPSUMO_CLASSIC_HW_A2;
					printk(KERN_INFO "Classic hardware version A2 detected\n");
					jumpingsumo_hsis_gpio = (struct_jumpingsumo_hsis_gpio *)&jumpingsumo_hsis_gpio_classic_pcbA2;
					jumpingsumo_hsis_pwm_gen = (struct_jumpingsumo_hsis_pwm_gen *)&jumpingsumo_hsis_pwm_gen_classic_pcbA2;
					jumpingsumo_hsis_pwm_sip6 = (struct_jumpingsumo_hsis_pwm_sip6 *)&jumpingsumo_hsis_pwm_sip6_classic_pcbA2;
					parrot_init_pin(pins_init_p6i_jpsumo_A2);
					break;

				case JPSUMO_CLASSIC_PCB_A3:
				case JPSUMO_CLASSIC_PCB_B1:
				case JPSUMO_CLASSIC_PCB_B2:/* JPSUMO_CLASSIC_PCB_06 */
				case JPSUMO_CLASSIC_PCB_07:
				default:
					switch ( jumpingsumo_hsis.pcbrev )
					{
						case JPSUMO_CLASSIC_PCB_A3:
							jumpingsumo_hsis.hwrev = JPSUMO_CLASSIC_HW_A3;
							printk(KERN_INFO "Classic hardware version A3 detected\n");
							break;
						case JPSUMO_CLASSIC_PCB_B1:
							jumpingsumo_hsis.hwrev = JPSUMO_CLASSIC_HW_B1;
							printk(KERN_INFO "Classic hardware version B1 detected\n");
							break;
						case JPSUMO_CLASSIC_PCB_B2:/* JPSUMO_CLASSIC_PCB_06 */
							jumpingsumo_hsis.hwrev = JPSUMO_CLASSIC_HW_06;
							printk(KERN_INFO "Classic hardware version B2/06 detected\n");
							break;
						case JPSUMO_CLASSIC_PCB_07:
						default:
							jumpingsumo_hsis.hwrev = JPSUMO_CLASSIC_HW_07;
							printk(KERN_INFO "Classic hardware version 07 detected\n");
							break;
					}

					jumpingsumo_hsis_gpio = (struct_jumpingsumo_hsis_gpio *)&jumpingsumo_hsis_gpio_classic_pcbA3;
					jumpingsumo_hsis_pwm_gen = (struct_jumpingsumo_hsis_pwm_gen *)&jumpingsumo_hsis_pwm_gen_classic_pcbA3;
					jumpingsumo_hsis_pwm_sip6 = (struct_jumpingsumo_hsis_pwm_sip6 *)&jumpingsumo_hsis_pwm_sip6_classic_pcbA3;

					parrot_init_pin(pins_init_p6i_jpsumo_A3);

					// Pull Up on UART1 RX:
					p6i_set_pads_uart1_pullup();

					/* Internal pull-up on nFAULT_WHEEL (GPIO 54) */
					jpsumo_sys_set_bit(_P6I_SYS_P1_0, 3, 1);
					jpsumo_sys_set_bit(_P6I_SYS_P2_1, 14, 0);
					break;
			}
			break;
	}

	/* Enable power off if product has a GPIO for that. */
	if (jumpingsumo_hsis_gpio->gpio_POWER_ON_OFF >= 0)
		pm_power_off = jpsumo_power_off;

	/* ON/OFF P6i output GPIO */
	if (jumpingsumo_hsis_gpio->gpio_POWER_ON_OFF >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_POWER_ON_OFF,
				(jumpingsumo_hsis_gpio->gpio_POWER_ON_OFF_on==1)?GPIOF_OUT_INIT_HIGH:GPIOF_OUT_INIT_LOW,
				"POWER_ON_OFF", 0);

	// Turn Off Jump H-Bridge : keep gpios as input.
	if (jumpingsumo_hsis_gpio->gpio_JUMP_CTRL_1 >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_JUMP_CTRL_1, GPIOF_IN, "JUMP_CTRL_1", 1);
	if (jumpingsumo_hsis_gpio->gpio_JUMP_CTRL_2 >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_JUMP_CTRL_2, GPIOF_IN, "JUMP_CTRL_2", 1);

	// JS : Turn Off and export RED LEDs
	if (jumpingsumo_hsis_gpio->gpio_RED_LED_LEFT >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_RED_LED_LEFT, GPIOF_OUT_INIT_HIGH, "RED_LED_LEFT", 0);
	if (jumpingsumo_hsis_gpio->gpio_RED_LED_RIGHT >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_RED_LED_RIGHT, GPIOF_OUT_INIT_HIGH, "RED_LED_RIGHT", 0);

	// JS : Turn Off and export GREEN LEDs
	if (jumpingsumo_hsis_gpio->gpio_GREEN_LED_LEFT >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_GREEN_LED_LEFT, GPIOF_OUT_INIT_LOW, "GREEN_LED_LEFT", 0);
	if (jumpingsumo_hsis_gpio->gpio_GREEN_LED_RIGHT >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_GREEN_LED_RIGHT, GPIOF_OUT_INIT_LOW, "GREEN_LED_RIGHT", 0);

	// JS : Turn Off PWM Generator
	if (jumpingsumo_hsis_gpio->gpio_PWMGEN_nOE >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_PWMGEN_nOE, GPIOF_OUT_INIT_HIGH, "PWMGEN_nOE", 0);

	// Turn Off LED IR:
	if (jumpingsumo_hsis_gpio->gpio_LED_IR >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_LED_IR, GPIOF_OUT_INIT_LOW, "LED_IR", 0);

	if (jumpingsumo_hsis_gpio->gpio_CHARGE_STATUS >= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_CHARGE_STATUS, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_CHARGE_STATUS, GPIOF_IN, "CHARGE_STATUS", 0);
	}

	if (jumpingsumo_hsis_gpio->gpio_WLAN_nPWD >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_WLAN_nPWD, GPIOF_OUT_INIT_HIGH, "WLAN_nPWD", 0);

	if (jumpingsumo_hsis_gpio->gpio_WLAN_WARM_RST >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_WLAN_WARM_RST, GPIOF_OUT_INIT_LOW, "WLAN_WARM_RST", 0);

	if (jumpingsumo_hsis_gpio->gpio_nSLEEP_WHEEL >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_nSLEEP_WHEEL, GPIOF_OUT_INIT_LOW, "nSLEEP_WHEEL", 0);

	if (jumpingsumo_hsis_gpio->gpio_nFAULT_WHEEL >= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_nFAULT_WHEEL, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_nFAULT_WHEEL, GPIOF_IN, "nFAULT_WHEEL", 1);
	}

	if (jumpingsumo_hsis_gpio->gpio_BUTTON >= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_BUTTON, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_BUTTON, GPIOF_IN, "BUTTON", 0);
	}

	/* Interrupt pin for pal_gpio_irq */
	if (jumpingsumo_hsis_gpio->gpio_INT_INERT >= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_INT_INERT, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_INT_INERT, GPIOF_IN, "INT_INERT", 1);
	}

	/* Speaker amplifier */
	if (jumpingsumo_hsis_gpio->gpio_nAMP_PWDN >= 0) {
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_nAMP_PWDN, GPIOF_OUT_INIT_LOW, "nAMP_PWDN", 0);
	}

	/* Etron /reset */
	if (jumpingsumo_hsis_gpio->gpio_NRST_CAM >= 0)
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_NRST_CAM, GPIOF_OUT_INIT_LOW, "NRST_CAM", 0);

	/* INIT AAI */
	init_aai();

	/* Init ultra sound */
	if (jumpingsumo_hsis_gpio->gpio_MUX_VBAT_JUMP >= 0) {
		ultra_snd_platform_data.gpio_mux_vbat_jump = jumpingsumo_hsis_gpio->gpio_MUX_VBAT_JUMP;
		p6i_export_gpio(ultra_snd_platform_data.gpio_mux_vbat_jump, GPIOF_OUT_INIT_LOW, "MUX_VBAT_JUMP", 0);
	}

	if (jumpingsumo_hsis_gpio->gpio_MUX_WHEELS >= 0) {
		ultra_snd_platform_data.gpio_mux_wheels = jumpingsumo_hsis_gpio->gpio_MUX_WHEELS;
		p6i_export_gpio(ultra_snd_platform_data.gpio_mux_wheels,    GPIOF_OUT_INIT_LOW, "MUX_WHEELS",    0);
	}

	p6_us_device.dev.platform_data 			= &ultra_snd_platform_data;

	/* Detection of plug/unplug usb */
	if (jumpingsumo_hsis_gpio->gpio_VBUS_DETECT >= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_VBUS_DETECT, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_VBUS_DETECT, GPIOF_IN, "VBUS_DETECT", 0);
	}

	// Detection of jump switch
	if (jumpingsumo_hsis_gpio->gpio_JMP_SWITCH >= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_JMP_SWITCH, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_JMP_SWITCH, GPIOF_IN, "JMP_SWITCH", 0);
	}

	// Wifi Reset
	if (jumpingsumo_hsis_gpio->gpio_WiFi_RST >= 0) {
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_WiFi_RST, GPIOF_OUT_INIT_LOW, "WIFI_RST", 0);
	}

	// MCU Reset
	if (jumpingsumo_hsis_gpio->gpio_MCU_RST >= 0) {
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_MCU_RST,
				GPIOF_OUT_INIT_LOW,
				"MCU_RST", 0);
		if (jumpingsumo_hsis_gpio->gpio_MCU_IT >= 0) {
			p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_MCU_IT,
					GPIOF_IN, "MCU_IT", 0);
			printk(KERN_INFO "Now registering spi_mcu_minidrones3_board_info \
					irq gpio %d, nb spi = %d\n",
					jumpingsumo_hsis_gpio->gpio_MCU_IT, ARRAY_SIZE(
						spi_mcu_minidrones3_board_info));
			mcu_minidrones3_data.gpio_rst =
				jumpingsumo_hsis_gpio->gpio_MCU_RST;
			spi_mcu_minidrones3_board_info->irq =
				gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_MCU_IT,
						GPIO_IRQ_POS, GPIO_DEBOUNCE_NONE);
			spi_register_board_info(spi_mcu_minidrones3_board_info,
					ARRAY_SIZE(spi_mcu_minidrones3_board_info));
		} else {
			printk("Now registering p6i_spi_board_info, nb spi = %d\n", ARRAY_SIZE(p6i_spi_board_info));
			spi_register_board_info(p6i_spi_board_info, ARRAY_SIZE(p6i_spi_board_info));
		}
	}

	// JS : Init USB switch (0: USB Connector, 1: USB hub)
	if (jumpingsumo_hsis_gpio->gpio_USB_MUX_CMD >= 0) {
		if (parrot_force_usb_device)
			p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_USB_MUX_CMD, GPIOF_OUT_INIT_LOW, "USB_MUX_CMD", 0);
		else
			p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_USB_MUX_CMD, GPIOF_OUT_INIT_HIGH, "USB_MUX_CMD", 0);
	}

	if (jumpingsumo_hsis_gpio->gpio_DEV_1 >= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_DEV_1, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_DEV_1, GPIOF_IN, "DEV_1", 1);
	}
	if (jumpingsumo_hsis_gpio->gpio_DEV_2 >= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_DEV_2, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_DEV_2, GPIOF_IN, "DEV_2", 1);
	}
	if (jumpingsumo_hsis_gpio->gpio_DEV_3 >= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_DEV_3, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_DEV_3, GPIOF_IN, "DEV_3", 1);
	}
	if (jumpingsumo_hsis_gpio->gpio_DEV_4>= 0) {
		gpio_interrupt_register(jumpingsumo_hsis_gpio->gpio_DEV_4, GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		p6i_export_gpio(jumpingsumo_hsis_gpio->gpio_DEV_4, GPIOF_IN, "DEV_4", 1);
	}

	// I2C :
	p6_i2cm0_device.dev.platform_data = &i2cm_platform_js_i2c0;
	p6i_set_pads_i2c(0);
	p6i_set_i2c_drive_strength(0, 0x0);

	// JS : eMMC - SDIO
	p6i_set_pads_sdcard(50000000);	// Set SDIO frequency at 50MHz & do set drive strength

	printk(KERN_INFO "Initializing Ethernet JTAG ...\n");
	//p6i_eth_on_jtag_init();

	switch ( jumpingsumo_hsis.hwrev )
	{
		case JPSUMO_CLASSIC_HW_A1:
			p6_sdhci0_device.dev.platform_data = &jpsumo_mmc_wifi_platform_data;

			platform_add_devices( p6i_jpsumo_A1_devices, ARRAY_SIZE(p6i_jpsumo_A1_devices) );
			break;
		case JPSUMO_CLASSIC_HW_A2:
			platform_add_devices( p6i_jpsumo_A2plus_devices, ARRAY_SIZE(p6i_jpsumo_A2plus_devices));
			break;
		case JPSUMO_CLASSIC_HW_A3:
		case JPSUMO_CLASSIC_HW_B1:
		case JPSUMO_CLASSIC_HW_B2:
		case JPSUMO_CLASSIC_HW_06:
		case JPSUMO_CLASSIC_HW_07:
			platform_add_devices(p6i_jpsumo_A2plus_devices, ARRAY_SIZE(p6i_jpsumo_A2plus_devices));
			break;

		case JPSUMO_EVO_HW_00:
		case JPSUMO_EVO_HW_01:
		case JPSUMO_EVO_HW_02:

			/* set drive strengh for usb */
			p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_PREEMDEPTH,0x01);
			p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_ENPRE,0x01);
			p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVAMPLITUDE,0x03);
			p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVSLOPE,0x0f);

			p6_sdhci0_device.dev.platform_data = &jpsumo_mmc_platform_data;

			platform_add_devices( p6i_jpsumo_EVO_devices, ARRAY_SIZE(p6i_jpsumo_EVO_devices) );
			break;
		case JPSUMO_V3_HW_00:
		default:
			/* set drive strengh for usb */
			p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_PREEMDEPTH,0x01);
			p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_ENPRE,0x01);
			p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVAMPLITUDE,0x03);
			p6i_set_usb_drive_strength(P6I_SYS_USB_CTL0_PHY_HSDRVSLOPE,0x0f);

			p6_sdhci0_device.dev.platform_data = &jpsumo_mmc_platform_data;

			platform_add_devices( p6i_jpsumo_v3_devices, ARRAY_SIZE(p6i_jpsumo_v3_devices) );
			break;
	}

	p6i_export_uart_hw_infos(1, 0, "");

	for (i = 0; i < ARRAY_SIZE(jpsumo_i2c_devices); i++)
	{
		struct my_i2c_board_info *ptr=&jpsumo_i2c_devices[i];
		if (ptr->hwrev_mask & (1<<jumpingsumo_hsis.hwrev)) {
			p6i_export_i2c_hw_infos(ptr->busnum,
					ptr->info.addr,
					ptr->version,
					ptr->info.type);
			i2c_register_board_info(0, &(ptr->info),
					1);
		}
	}

	sysfs_jpsumo_init();
}

static void __init jpsumo_classic_init(void)
{
	jpsumo_init(JPSUMO_CLASSIC);
}

static void __init jpsumo_evo_init(void)
{
	jpsumo_init(JPSUMO_EVO);
}

static void __init jpsumo_v3_init(void)
{
	jpsumo_init(JPSUMO_V3);
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

MAP_BSP_VALUE_STRING(model, jumpingsumo_hsis);
MAP_BSP_VALUE_INT_NP(hwrev, jumpingsumo_hsis);
MAP_BSP_VALUE_INT_NP(pcbrev, jumpingsumo_hsis);

MAP_BSP_VALUE_INT(pwm_sip6_wheel_left_a, jumpingsumo_hsis_pwm_sip6);
MAP_BSP_VALUE_INT(pwm_sip6_wheel_left_b, jumpingsumo_hsis_pwm_sip6);
MAP_BSP_VALUE_INT(pwm_sip6_wheel_right_a, jumpingsumo_hsis_pwm_sip6);
MAP_BSP_VALUE_INT(pwm_sip6_wheel_right_b, jumpingsumo_hsis_pwm_sip6);

MAP_BSP_VALUE_INT(pwm_gen_green_led_left, jumpingsumo_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_green_led_right, jumpingsumo_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_headlight_left, jumpingsumo_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_headlight_right, jumpingsumo_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_jump_a, jumpingsumo_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_jump_b, jumpingsumo_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_outdrv, jumpingsumo_hsis_pwm_gen);
MAP_BSP_VALUE_INT(pwm_gen_headlight_complement, jumpingsumo_hsis_pwm_gen);

MAP_BSP_VALUE_INT(gpio_INT_INERT, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_VBUS_DETECT, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_BUTTON, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_NRST_CAM, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_RED_LED_LEFT, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_RED_LED_RIGHT, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_GREEN_LED_LEFT, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_GREEN_LED_RIGHT, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_JUMP_CTRL_1, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_JUMP_CTRL_2, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_PWMGEN_nOE, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_nAMP_PWDN, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_MUX_VBAT_JUMP, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_MUX_WHEELS, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_nSLEEP_WHEEL, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_nFAULT_WHEEL, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_POWER_ON_OFF, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_POWER_ON_OFF_on, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_USB_MUX_CMD, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_WLAN_nPWD, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_WLAN_WARM_RST, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_JMP_SWITCH, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_WiFi_RST, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_MCU_RST, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_MCU_IT, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_DEV_1, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_DEV_2, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_DEV_3, jumpingsumo_hsis_gpio);
MAP_BSP_VALUE_INT(gpio_DEV_4, jumpingsumo_hsis_gpio);

static const struct attribute *jpsumo_bsp_attrs[] = {
	BSP_ATTR(model),
	BSP_ATTR(hwrev),
	BSP_ATTR(pcbrev),
	BSP_ATTR(pwm_sip6_wheel_left_a),
	BSP_ATTR(pwm_sip6_wheel_left_b),
	BSP_ATTR(pwm_sip6_wheel_right_a),
	BSP_ATTR(pwm_sip6_wheel_right_b),
	BSP_ATTR(pwm_gen_green_led_left),
	BSP_ATTR(pwm_gen_green_led_right),
	BSP_ATTR(pwm_gen_headlight_left),
	BSP_ATTR(pwm_gen_headlight_right),
	BSP_ATTR(pwm_gen_jump_a),
	BSP_ATTR(pwm_gen_jump_b),
	BSP_ATTR(pwm_gen_outdrv),
	BSP_ATTR(pwm_gen_headlight_complement),
	BSP_ATTR(gpio_INT_INERT),
	BSP_ATTR(gpio_VBUS_DETECT),
	BSP_ATTR(gpio_BUTTON),
	BSP_ATTR(gpio_NRST_CAM),
	BSP_ATTR(gpio_RED_LED_LEFT),
	BSP_ATTR(gpio_RED_LED_RIGHT),
	BSP_ATTR(gpio_GREEN_LED_LEFT),
	BSP_ATTR(gpio_GREEN_LED_RIGHT),
	BSP_ATTR(gpio_JUMP_CTRL_1),
	BSP_ATTR(gpio_JUMP_CTRL_2),
	BSP_ATTR(gpio_PWMGEN_nOE),
	BSP_ATTR(gpio_nAMP_PWDN),
	BSP_ATTR(gpio_MUX_VBAT_JUMP),
	BSP_ATTR(gpio_MUX_WHEELS),
	BSP_ATTR(gpio_nSLEEP_WHEEL),
	BSP_ATTR(gpio_nFAULT_WHEEL),
	BSP_ATTR(gpio_POWER_ON_OFF),
	BSP_ATTR(gpio_POWER_ON_OFF_on),
	BSP_ATTR(gpio_USB_MUX_CMD),
	BSP_ATTR(gpio_WLAN_nPWD),
	BSP_ATTR(gpio_WLAN_WARM_RST),
	BSP_ATTR(gpio_JMP_SWITCH),
	BSP_ATTR(gpio_WiFi_RST),
	BSP_ATTR(gpio_MCU_RST),
	BSP_ATTR(gpio_MCU_IT),
	BSP_ATTR(gpio_DEV_1),
	BSP_ATTR(gpio_DEV_2),
	BSP_ATTR(gpio_DEV_3),
	BSP_ATTR(gpio_DEV_4),
	NULL
};

static const struct attribute_group jpsumo_bsp_attr_group = {
	.attrs = (struct attribute **)jpsumo_bsp_attrs,
};

static __init int sysfs_jpsumo_init(void)
{
	struct kobject *js_kobject;
	int ret;

	printk(KERN_INFO "JumpingSumo %s: exporting HSIS to userspace in /sys/kernel/hsis",
			jumpingsumo_hsis.model);

	js_kobject = kobject_create_and_add("hsis", kernel_kobj);
	if (!js_kobject)
		return -ENOMEM;

	ret = sysfs_create_group(js_kobject, &jpsumo_bsp_attr_group);
	if (ret) {
		kobject_put(js_kobject);
		return ret;
	}

	return 0;
}

MACHINE_START(PARROT_JPSUMO_EVO, "Jumping Sumo EVO sip6 board")
	/* Maintainer: Parrot S.A. */
#ifdef CONFIG_RAMOOPS
	.reserve    = jpsumo_reserve_mem,
#endif
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io	 = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer	  = &p6_timer,
	.init_machine   = jpsumo_evo_init,
MACHINE_END

MACHINE_START(PARROT_JPSUMO, "Jumping Sumo Classic sip6 board")
	/* Maintainer: Parrot S.A. */
#ifdef CONFIG_RAMOOPS
	.reserve    = jpsumo_reserve_mem,
#endif
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io	 = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer	  = &p6_timer,
	.init_machine   = jpsumo_classic_init,
MACHINE_END

MACHINE_START(PARROT_JPSUMO3, "Jumpingsumo3 board")
	/* Maintainer: Parrot S.A. */
#ifdef CONFIG_RAMOOPS
	.reserve    = jpsumo_reserve_mem,
#endif
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io	 = p6i_map_io,
	.init_irq   = p6_init_irq,
	.timer	  = &p6_timer,
	.init_machine   = jpsumo_v3_init,
MACHINE_END


