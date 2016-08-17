/*
* drivers/misc/lps22hb.c
*
* STMicroelectronics LPS22HB Pressure / Temperature Sensor module driver
*
* Copyright (C) 2015 STMicroelectronics - HESA BU - Application Team
* Authors: Adalberto Muhuho (adalberto.muhuho@st.com)
* The structure of this driver is based on reference code previously
* delivered by Lorenzo Sarchi
*
* Version: 0.0.2
* Date: 2016/Apr/19
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
*
** Output data from the device are available from the assigned
* /dev/input/eventX device;
*
* LPS22HB can be controlled by sysfs interface looking inside:
* /sys/bus/i2c/devices/<busnum>-<devaddr>/
*
* LPS22HB make available two i2C addresses selectable from platform_data.
*
*
* Read pressures and temperatures output can be converted in units of
* measurement by dividing them respectively for SENSITIVITY_P and SENSITIVITY_T.
* Temperature values must then be added by the constant float TEMPERATURE_OFFSET
* expressed as Celsius degrees.
*
* Obtained values are then expessed as
* mbar (=0.1 kPa) and Celsius degrees.
*
*
*/
/******************************************************************************
 Revision history:

 Revision 0.0.1 2015/Dec/11: 1st beta version

 Revision 0.0.2 2016/Apr/19:
	Revision 0.0.2 downgrades previous License to GPLv2
******************************************************************************/

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
//#include <linux/export.h>
#include <linux/module.h>
#include "lps22hb.h"

//#define	DEBUG 		1

/* Driver interface version. Increment on interface change. */
#define	DRIVER_VERSION	1

#define	PR_ABS_MAX	8388607		/* 24 bit 2'compl */
#define	PR_ABS_MIN	-8388608
#ifdef SHRT_MAX
#define	TEMP_MAX	SHRT_MAX
#define TEMP_MIN	SHRT_MIN
#else
#define	TEMP_MAX	SHORT_MAX
#define TEMP_MIN	SHORT_MIN
#endif

/* Device ID */
#define	WHOAMI_LPS22_PRS	0xB1

/*	REGISTERS */
#define	INT_CFG_REG			0x0B		/*	interrupt config reg	*/
#define	THS_P_L				0x0C		/*	pressure threshold	*/
#define	THS_P_H				0x0D		/*	pressure threshold	*/
#define	WHO_AM_I			0x0F		/*	Device ID register		*/
#define	CTRL_REG1			0x10		/*	Control register 1	*/
#define	CTRL_REG2			0x11		/*	Control register 2	*/
#define	CTRL_REG3			0x12		/*	Control register 3	*/
#define	FIFO_CTRL			0x14		/*  Fifo control register */
#define	REF_P_XL			0x15		/*	pressure reference	*/
#define	REF_P_L				0x16		/*	pressure reference	*/
#define	REF_P_H				0x17		/*	pressure reference	*/
#define	RPDS_TRM_L			0x18		/*	NEW	*/
#define	RPDS_TRM_H			0x19		/*	NEW	*/
#define	RESOL_CONF			0x1A		/*	Resolution configuration */
#define	CTRL_REG4			0x23		/*	Control register 4	*/
#define	INT_SRC_REG			0x25		/*	interrupt source reg	*/
#define	FIFO_STATUS			0x26        /*  Fifo Status reg */
#define	STATUS_REG			0X27		/*	Status reg		*/
#define	PRESS_OUT_XL		0x28		/*	press output (3 regs)	*/
#define	TEMP_OUT_L			0x2B		/*	temper output (2 regs)	*/

/*	REGISTERS ALIASES	*/
#define	P_REF_INDATA_REG	REF_P_XL
#define	P_THS_INDATA_REG	THS_P_L
#define	P_OUTDATA_REG		PRESS_OUT_XL
#define	T_OUTDATA_REG		TEMP_OUT_L
#define	OUTDATA_REG		    PRESS_OUT_XL

/* Bitmasks */
#define	ODR_MASK			 0x70
#define	DIFF_MASK			 0x08
#define	BDU_MASK			 0x02
#define	RESET_AZ_MASK		 0x10
#define	LC_EN_MASK   		 0x01
#define EN_LPFP_MASK    	 0x08
#define LPF_CFG_MASK    	 0x04
#define RESET_ARP_MASK  	 0x40
#define DIFF_EN_MASK    	 0x08
#define PLE_MASK			 0x02
#define PHE_MASK			 0x01
#define FIFO_EN_MASK    	 0x40
#define FIFO_MODE_MASK		 0xE0
#define FIFO_SAMPLE_MASK	 0x1F
#define	AUTOZ_MASK			 0x20
#define	AUTOZ_OFF			 0x00
#define	AUTORIFP_MASK		 0x80
#define STOP_ON_FTH_MASK     0x20
#define DRDY_MASK            0x04
/* Barometer and Termometer output data rate ODR */
#define	ODR_ONESH	0x00	/* one shot both		*/
#define	ODR_1_1		0x10	/*  1  Hz baro,  1  Hz term ODR	*/
#define	ODR_10_10	0x20	/* 10  Hz baro, 10  Hz term ODR	*/
#define	ODR_25_25	0x30	/* 25  Hz baro, 25  Hz term ODR	*/
#define	ODR_50_50	0x40	/* 50  Hz baro, 50  Hz term ODR	*/
#define	ODR_75_75	0x50	/* 75  Hz baro, 75  Hz term ODR	*/

/* Additional operating modes defines */
#define	I2C_AUTO_INCREMENT	0         //LPS25H legacy (0x80), not needed on LPS22HB (therefore 0)
#define	AUTOZ_ENABLE		1
#define	AUTOZ_DISABLE		0
#define RES_MAX 			0
#define	FUZZ				0
#define	FLAT				0
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES			5

/* RESUME STATE INDICES */
#define	RES_REF_P_XL		0
#define	RES_REF_P_L			1
#define	RES_REF_P_H			2
#define	RES_REFT_L			3
#define	RES_REFT_H			4
#define	RES_RESOL_CONF		5
#define	RES_CTRL_REG1		6
#define	RES_CTRL_REG2		7
#define	RES_CTRL_REG3		8
#define	RES_CTRL_REG4		9
#define	RES_INT_CFG_REG		10
#define	RES_FIFO_CTRL		11
#define	RES_THS_P_L			12
#define	RES_THS_P_H			13
#define	RES_RPSD_TRIM_L		14
#define	RES_RPSD_TRIM_H		15
#define	RESUME_ENTRIES		16
/* end RESUME STATE INDICES */

u8 decimator_count = 0;
u8 logout_decimation = 1;
u8 hex_measr_logging = 0;

#ifdef CONFIG_PARROT_ST22HB_USE_THREADED_INTERRUPTS
inline int use_threaded_interrupts(void) {
	return 1;
}
#else
inline int use_threaded_interrupts(void) {
	return 0;
}
#endif

static const struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lps22_prs_odr_table[] = {
	{13,	ODR_75_75 },
	{20,	ODR_50_50 },
	{40,	ODR_25_25 },
	{100,	ODR_10_10 },
	{1000,	ODR_1_1 },
};

struct lps22_prs_data {
	struct i2c_client *client;
	struct lps22_prs_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev_pres;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;


	atomic_t enabled;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];

#ifdef DEBUG
	u8 reg_addr;
#endif

	int use_smbus;
};

struct outputdata {
	s32 press;
	s16 temperature;
};

static const struct lps22_prs_platform_data default_lps22_pdata = {
	.poll_interval = 1000,
	.min_interval = LPS22_PRS_MIN_POLL_PERIOD_MS,
};

u8 snsdata[3];

static int lps22_prs_i2c_read(struct lps22_prs_data *prs, u8 *buf, int len)
{
	int ret;
	int tries = 0;
	u8 reg = buf[0];
	u8 cmd = reg;
#ifdef DEBUG
	unsigned int ii;
#endif
	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (prs->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(prs->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&prs->client->dev,
					"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
					"command=0x%02x, buf[0]=0x%02x\n",
					ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(prs->client,
					cmd, len, buf);
#ifdef DEBUG
			dev_warn(&prs->client->dev,
					"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
					"command=0x%02x, ",
					ret, len, cmd);
			for (ii = 0; ii < len; ii++)
				printk(KERN_DEBUG "buf[%d]=0x%02x,",
						ii, buf[ii]);

			printk("\n");
#endif
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&prs->client->dev,
					"read transfer error: len:%d, command=0x%02x\n",
					len, cmd);
			return 0;
		}
		return len;
	}

	do {
		ret = i2c_master_send(prs->client, &cmd, sizeof(cmd));
		if (ret != sizeof(cmd))
			msleep_interruptible (I2C_RETRY_DELAY);
	}	while ( (ret != sizeof(cmd)) && (++tries < I2C_RETRIES));
	if (ret != sizeof(cmd))
		return ret;
	return i2c_master_recv(prs->client, (char *)buf, len);
}

static int lps22_prs_i2c_write(struct lps22_prs_data *prs,
		u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = prs->client->addr,
			.flags = prs->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(prs->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible (I2C_RETRY_DELAY);
	}	while ( (err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&prs->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}
	return 0;
}

static int lps22_prs_hw_init(struct lps22_prs_data *prs)
{
	int err;
	u8 buf[6];

	pr_info("%s: hw init start\n", LPS22_PRS_DEV_NAME);
	dev_dbg(&prs->client->dev,"%s: hw init start\n", LPS22_PRS_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = lps22_prs_i2c_read(prs, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		prs->hw_working = 1;
	if (buf[0] != WHOAMI_LPS22_PRS) {
		err = -1; /* TODO:choose the right coded error */
		goto error_unknown_device;
	}


	buf[0] = (I2C_AUTO_INCREMENT | P_REF_INDATA_REG);
	buf[1] = prs->resume_state[RES_REF_P_XL];
	buf[2] = prs->resume_state[RES_REF_P_L];
	buf[3] = prs->resume_state[RES_REF_P_H];

	err = lps22_prs_i2c_write(prs, buf, 3);
	if (err < 0)
		goto err_resume_state;
	printk("hw_init: REF_P pass \r\n");

	buf[0] = RESOL_CONF;
	buf[1] = prs->resume_state[RES_RESOL_CONF];
	err = lps22_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto err_resume_state;
	printk("hw_init: RESOL_CONF pass \r\n");

	buf[0] = (I2C_AUTO_INCREMENT | P_THS_INDATA_REG);
	buf[1] = prs->resume_state[RES_THS_P_L];
	buf[2] = prs->resume_state[RES_THS_P_H];
	err = lps22_prs_i2c_write(prs, buf, 2);
	if (err < 0)
		goto err_resume_state;
#ifdef DEBUG
	printk("hw_init: P_THS_INDATA_REG pass \r\n");
#endif

	buf[0] = CTRL_REG2;
	buf[1] = (prs->resume_state[RES_CTRL_REG2]) | 0x10;
	buf[2] = prs->resume_state[RES_CTRL_REG3];

	err = lps22_prs_i2c_write(prs, buf, 2);
	if (err < 0)
		goto err_resume_state;
#ifdef DEBUG
	printk("hw_init: P_CTRL_REGS23 pass \r\n");
#endif

	buf[0] = INT_CFG_REG;
	buf[1] = prs->resume_state[RES_INT_CFG_REG];
	err = lps22_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto err_resume_state;
#ifdef DEBUG
	printk("hw_init: INT_CFG_REG pass \r\n");
#endif

	buf[0] = CTRL_REG1;
	buf[1] = prs->resume_state[RES_CTRL_REG1];
	err = lps22_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto err_resume_state;
#ifdef DEBUG
	printk("hw_init: CTRL_REG1 pass \r\n");
#endif

	if (use_threaded_interrupts()) {
		buf[0] = CTRL_REG3;
		buf[1] = DRDY_MASK;
		err = lps22_prs_i2c_write(prs, buf, 1);
		if (err < 0)
			goto err_resume_state;
	}

	prs->hw_initialized = 1;

	pr_info("%s: hw init done\n", LPS22_PRS_DEV_NAME);
	dev_dbg(&prs->client->dev, "%s: hw init done\n", LPS22_PRS_DEV_NAME);
	return 0;

error_firstread:
	prs->hw_working = 0;
	dev_warn(&prs->client->dev, "Error reading WHO_AM_I: is device "
			"available/working?\n");
	goto err_resume_state;
error_unknown_device:
	dev_err(&prs->client->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n", WHOAMI_LPS22_PRS, buf[0]);
err_resume_state:
	prs->hw_initialized = 0;
	dev_err(&prs->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lps22_prs_device_power_off(struct lps22_prs_data *prs)
{
	int err;

	u8 buf[2] = { CTRL_REG1, 0x00 }; //STdbg: was 0x7F

	err = lps22_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		dev_err(&prs->client->dev, "soft power off failed: %d\n", err);

	if (prs->pdata->power_off)
		prs->pdata->power_off();
	prs->hw_initialized = 0;
}

int lps22_prs_update_odr(struct lps22_prs_data *prs, int poll_period_ms)
{
	int err = -1;
	int i;

	u8 buf[2];
	u8 init_val, updated_val;
	u8 curr_val, new_val;
	u8 mask = ODR_MASK;


	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (longest period) backward (shortest
	 * period), to support the poll_interval requested by the system.
	 * It must be the longest period shorter then the set poll period.*/
	for (i = ARRAY_SIZE(lps22_prs_odr_table) - 1; i >= 0; i--) {
#ifdef DEBUG
		printk("poll period tab index %d \r\n",i);
		printk("poll period tab cutoff %d \r\n",lps22_prs_odr_table[i].cutoff_ms);
		printk("poll period tab mask %02x \r\n",lps22_prs_odr_table[i].mask);
#endif
		if ((lps22_prs_odr_table[i].cutoff_ms <= poll_period_ms)
				|| (i == 0))
			break;
	}

#ifdef DEBUG
	printk("\r\n");
	printk("new poll period setting: %d \r\n",poll_period_ms);
#endif

	new_val = lps22_prs_odr_table[i].mask;

#ifdef DEBUG
	printk("new ODR bits: %02x \r\n",new_val);
#endif

	/* before to change the ODR it is mandatory to power down
	   the device */

	buf[0] = CTRL_REG1;
	err = lps22_prs_i2c_read(prs, buf, 1);
	if (err < 0)
		goto error;
	/* work on all but ENABLE bits */
	/* power down */
	init_val = buf[0];
	prs->resume_state[RES_CTRL_REG1] = init_val ;

	curr_val = init_val & 0x0F;

	buf[0] = CTRL_REG1;
	buf[1] = curr_val;
	err = lps22_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto error;

	/* set new ODR*/
	buf[0] = CTRL_REG1;
	updated_val = ((mask & new_val) | ((~mask) & curr_val) | BDU_MASK);

	buf[0] = CTRL_REG1;
	buf[1] = updated_val;
	err = lps22_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		goto error;

	prs->resume_state[RES_CTRL_REG1] = updated_val;

	return err;

error:
	dev_err(&prs->client->dev, "update odr failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int lps22_prs_device_power_on(struct lps22_prs_data *prs)
{
	int err = -1;

	if (prs->pdata->power_on) {
		err = prs->pdata->power_on();
		if (err < 0) {
			dev_err(&prs->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
	}

	if (!prs->hw_initialized) {
		err = lps22_prs_hw_init(prs);
		lps22_prs_update_odr(prs, prs->pdata->poll_interval);
		if (prs->hw_working == 1 && err < 0) {
			lps22_prs_device_power_off(prs);
			return err;
		}
	}

	return 0;
}

static int lps22_prs_set_press_reference(struct lps22_prs_data *prs,
		s32 new_reference)
{
	int err;

	u8 bit_valuesXL,bit_valuesL, bit_valuesH;
	u8 buf[4];

	bit_valuesXL = (u8) (new_reference & 0x0000FF);
	bit_valuesL = (u8)((new_reference & 0x00FF00) >> 8);
	bit_valuesH = (u8)((new_reference & 0xFF0000) >> 16);


	buf[0] = (I2C_AUTO_INCREMENT | P_REF_INDATA_REG);
	buf[1] = bit_valuesXL;
	buf[2] = bit_valuesL;
	buf[3] = bit_valuesH;

	err = lps22_prs_i2c_write(prs,buf,3);
	if (err < 0)
		return err;

	prs->resume_state[RES_REF_P_XL] = bit_valuesXL;
	prs->resume_state[RES_REF_P_L] = bit_valuesL;
	prs->resume_state[RES_REF_P_H] = bit_valuesH;

#ifdef DEBUG
	printk("LPS22HB new reference pressure setting : %d \r\n",
			(((u32)bit_valuesH)<<16)+(((u32)bit_valuesL)<<8)+((u32)(bit_valuesXL)));
#endif

	return err;
}

static int lps22_prs_get_press_reference(struct lps22_prs_data *prs,
		s32 *buf32)
{
	int err;

	u8 bit_valuesXL, bit_valuesL, bit_valuesH;
	u8 buf[3];
	u16 temp = 0;

	buf[0] =  (I2C_AUTO_INCREMENT | P_REF_INDATA_REG);
	err = lps22_prs_i2c_read(prs, buf, 3);
	if (err < 0)
		return err;
	bit_valuesXL = buf[0];
	bit_valuesL = buf[1];
	bit_valuesH = buf[2];


	temp = (( bit_valuesH ) << 8 ) | ( bit_valuesL ) ;
	*buf32 = (s32)((((s16) temp) << 8) | ( bit_valuesXL ));
#ifdef DEBUG
	dev_dbg(&prs->client->dev,"%s val: %+d", LPS22_PRS_DEV_NAME, *buf32 );
#endif
	return err;
}

static int lps22_prs_get_presstemp_data(struct lps22_prs_data *prs,
		struct outputdata *out)
{
	int err;
	/* Data bytes from hardware	PRESS_OUT_XL,PRESS_OUT_L,PRESS_OUT_H, */
	/*				TEMP_OUT_L, TEMP_OUT_H */

	u8 prs_data[5];

	s32 pressure;
	s16 temperature;

	int regToRead = 5;

	prs_data[0] = (I2C_AUTO_INCREMENT | OUTDATA_REG);
	err = lps22_prs_i2c_read(prs, prs_data, regToRead);
	if (err < 0)
		return err;

	//#ifdef DEBUG
#if 0
	if (hex_measr_logging)
		printk("temp out tH = 0x%02x, tL = 0x%02x,"
				"press_out: pH = 0x%02x, pL = 0x%02x, pXL= 0x%02x\n",
				prs_data[4],
				prs_data[3],
				prs_data[2],
				prs_data[1],
				prs_data[0]);
#endif

	pressure = (s32)((((s8) prs_data[2]) << 16) |
			(prs_data[1] <<  8) |
			( prs_data[0]));
	temperature = (s16) ((((s8) prs_data[4]) << 8) | (prs_data[3]));

	//#ifdef DEBUG
#if 0
	if ((decimator_count%logout_decimation)==0)
		printk("%d \n", (int32_t)pressure);
	decimator_count++;
#endif

	out->press = pressure;

	out->temperature = temperature;

	return err;
}



static void lps22_prs_report_values(struct lps22_prs_data *prs,
		struct outputdata *out)
{

	input_report_abs(prs->input_dev_pres, ABS_PR, out->press);

	input_report_abs(prs->input_dev_pres, ABS_TEMP, out->temperature);
	input_sync(prs->input_dev_pres);
}

static void lps22_prs_input_notify(struct lps22_prs_data *prs)
{
	struct outputdata output;
	int err;

	mutex_lock(&prs->lock);
	err = lps22_prs_get_presstemp_data(prs, &output);
	if (err < 0)
		dev_err(&prs->client->dev, "get_pressure_data failed\n");
	else
		lps22_prs_report_values(prs, &output);

	mutex_unlock(&prs->lock);
}

static irqreturn_t lps22_interrupt_thread(int irq, void *dev_id)
{
	struct lps22_prs_data *prs = dev_id;

	lps22_prs_input_notify(prs);

	return IRQ_HANDLED;
}

static int lps22_prs_enable(struct lps22_prs_data *prs)
{
	int err;
	unsigned long jiffies;

	if (!atomic_cmpxchg(&prs->enabled, 0, 1)) {
		err = lps22_prs_device_power_on(prs);
		if (err < 0) {
			atomic_set(&prs->enabled, 0);
			return err;
		}

		if (use_threaded_interrupts()) {
			err = request_threaded_irq(prs->client->irq,
						   NULL,
						   lps22_interrupt_thread,
						   IRQF_ONESHOT |
						   IRQF_TRIGGER_HIGH,
						   LPS22_PRS_DEV_NAME,
						   prs);

			if (err) {
				pr_err("failed to register interrupt %s",
				       LPS22_PRS_DEV_NAME);
				return err;
			}
		} else {
			jiffies = msecs_to_jiffies(prs->pdata->poll_interval);
			schedule_delayed_work(&prs->input_work, jiffies);
		}
	}

	return 0;
}

static int lps22_prs_disable(struct lps22_prs_data *prs)
{
	if (atomic_cmpxchg(&prs->enabled, 1, 0)) {
		if (use_threaded_interrupts())
			free_irq(prs->client->irq, prs);
		else
			cancel_delayed_work_sync(&prs->input_work);

		lps22_prs_device_power_off(prs);
	}

	return 0;
}


static ssize_t attr_get_polling_rate(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int val;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	mutex_lock(&prs->lock);
	val = prs->pdata->poll_interval;
	mutex_unlock(&prs->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max((unsigned int)interval_ms,prs->pdata->min_interval);

	mutex_lock(&prs->lock);
	prs->pdata->poll_interval = interval_ms;
	lps22_prs_update_odr(prs, interval_ms);
	mutex_unlock(&prs->lock);
	return size;
}


static ssize_t attr_get_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	int val = atomic_read(&prs->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

#ifdef DEBUG
	pr_info("\n%s Value= \"%s\" \n", LPS22_PRS_DEV_NAME, buf);
#endif

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS22_PRS_DEV_NAME, val);
#endif

	if (val)
		lps22_prs_enable(prs);
	else
		lps22_prs_disable(prs);

	return size;
}

static ssize_t attr_get_press_ref(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	s32 val = 0;

	mutex_lock(&prs->lock);
	err = lps22_prs_get_press_reference(prs, &val);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_press_ref(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err = -1;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	long val = 0;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	if (val < PR_ABS_MIN || val > PR_ABS_MAX)
		return -EINVAL;


	mutex_lock(&prs->lock);
	err = lps22_prs_set_press_reference(prs, val);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	return size;
}

static ssize_t attr_set_autozero(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 const mask = AUTOZ_MASK;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);
	snsdata[0] = INT_CFG_REG;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[INT_CFG_REG] = init_val;

	updated_val = ( (mask & (((u8)val)<<5)) | ((~mask) & init_val));
	snsdata[0] = INT_CFG_REG;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

exit :
	return size;
}

static ssize_t attr_reset_autozero(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 const mask = RESET_AZ_MASK;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);
	snsdata[0] = INT_CFG_REG;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[INT_CFG_REG] = init_val;

	updated_val = ( (mask & (((u8)val)<<4)) | ((~mask) & init_val));
	snsdata[0] = INT_CFG_REG;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

exit:
	return size;
}

static ssize_t attr_set_autorifp(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 const mask = AUTORIFP_MASK;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);
	snsdata[0] = INT_CFG_REG;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[INT_CFG_REG] = init_val;

	updated_val = ((mask & (((u8)val)<<7)) | ((~mask) & init_val));
	snsdata[0] = INT_CFG_REG;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

exit :
	return size;
}

static ssize_t attr_reset_autorifp(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;

	u8 const mask = RESET_ARP_MASK;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);

	snsdata[0] = INT_CFG_REG;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[INT_CFG_REG] = init_val;

	updated_val = ( (mask & (((u8)val)<<6)) | ((~mask) & init_val));
	snsdata[0] = INT_CFG_REG;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

exit:
	return size;
}

static ssize_t attr_set_pthreshold(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&prs->lock);

	snsdata[0] = THS_P_L;
	snsdata[1] = ((u16)val) & 0xFF;
	snsdata[2] = (((u16)val) >> 8) & 0xFF;
	err = lps22_prs_i2c_write(prs, snsdata, 2);


	if (err >= 0) {
		prs->resume_state[RES_THS_P_L] = snsdata[1];
		prs->resume_state[RES_THS_P_H] = snsdata[2];
	}

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

#ifdef DEBUG
	printk("LPS22HB new pressure threshold setting : %d \r\n",
			((((u16)snsdata[2])<<8)+((u16)(snsdata[1]))));
#endif

	return size;
}

static ssize_t attr_get_pthreshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u16 val = 0;

	mutex_lock(&prs->lock);
	snsdata[0] = THS_P_L;
	err = lps22_prs_i2c_read(prs, snsdata, 2);
	val = ((u16)snsdata[1] << 8) + snsdata[0];
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_pthreshold_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;

	u8 mask = ((u8)(DIFF_EN_MASK | PLE_MASK | PHE_MASK));

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);
	snsdata[0] = INT_CFG_REG;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[INT_CFG_REG] = init_val;

	updated_val = ((~mask) & init_val);
	if (val == 1)
		updated_val |= mask;

	snsdata[0] = INT_CFG_REG;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);

	if (err >= 0)
		prs->resume_state[RES_INT_CFG_REG] = updated_val;


	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
exit:
	return size;
}

static ssize_t attr_get_pthreshold_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;
	u8 mask = ((u8)(DIFF_EN_MASK | PLE_MASK | PHE_MASK));

	mutex_lock(&prs->lock);
	snsdata[0] = INT_CFG_REG;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	val = (snsdata[0] & mask);
	mutex_unlock(&prs->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_watermark_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 const mask = STOP_ON_FTH_MASK;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);

	snsdata[0] = CTRL_REG2;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_CTRL_REG2] = init_val;

	updated_val = ((mask & ((u8)val)<<5) | ((~mask) & init_val));

	snsdata[0] = CTRL_REG2;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	else
		prs->resume_state[RES_CTRL_REG2] = updated_val;
exit:
	return size;
}

static ssize_t attr_get_watermark_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	snsdata[0] = CTRL_REG2;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	val = ((snsdata[0] & 0x20)>>5);
	mutex_unlock(&prs->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_lc_mode_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 const mask = LC_EN_MASK;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);

	snsdata[0] = RESOL_CONF;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_RESOL_CONF] = init_val;

	updated_val = ((mask & ((u8)val)) | ((~mask) & init_val));

	/* power down the device before going ahead with LC enable update */

	snsdata[0] = CTRL_REG1;
	snsdata[1] = (prs->resume_state[RES_CTRL_REG1] & 0x0F);
	err = lps22_prs_i2c_write(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}
	/* power down transaction end */

	snsdata[0] = RESOL_CONF;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	} else {
		prs->resume_state[RES_RESOL_CONF] = updated_val;
	}

	/* power up the device or at least get CTRL_REG1 back to its previous state */

	snsdata[0] = CTRL_REG1;
	snsdata[1] = (prs->resume_state[RES_CTRL_REG1] & 0x7F);
	err = lps22_prs_i2c_write(prs, snsdata, 1);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	/* power up transaction end */
exit:
	return size;
}

static ssize_t attr_get_lc_mode_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	snsdata[0] = RESOL_CONF;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	val = snsdata[1] & 0x1;
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_lpf_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 const mask = EN_LPFP_MASK;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) & (val != 1))
		goto exit;

	mutex_lock(&prs->lock);

	snsdata[0] = CTRL_REG1;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_CTRL_REG1] = init_val;

	updated_val = ((mask & ((u8)val)<<3) | ((~mask) & init_val));

	snsdata[0] = CTRL_REG1;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	else
		prs->resume_state[RES_CTRL_REG1] = updated_val;
exit:
	return size;
}

static ssize_t attr_get_lpf_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	snsdata[0] = CTRL_REG1;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	val = ((snsdata[1] & 0x08) >> 3);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_set_lpf_cutoff_freq(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 const mask = LPF_CFG_MASK;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	if ((val!=0) && (val!=1))
		goto exit;

	mutex_lock(&prs->lock);

	snsdata[0] = CTRL_REG1;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_CTRL_REG1] = init_val;

	updated_val = (((mask) & (((u8)val)<<2)) | ((~mask) & init_val));

	snsdata[0] = CTRL_REG1;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	else
		prs->resume_state[RES_CTRL_REG1] = updated_val;

exit:
	return size;
}

static ssize_t attr_get_lpf_cutoff_freq(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	snsdata[0] = CTRL_REG1;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	val = ((snsdata[0] & LPF_CFG_MASK)>>2) & 0x1;
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_get_fifo_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	snsdata[0] = FIFO_STATUS;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	val = snsdata[0];
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_get_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	snsdata[0] = STATUS_REG;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	val = snsdata[0];
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_get_interrupt_source(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;


	mutex_lock(&prs->lock);
	snsdata[0] = INT_SRC_REG;//CTRL_REG2;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	val = snsdata[0];
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_set_fifo(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err= -1;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	static u8 mask = FIFO_EN_MASK;

#ifdef DEBUG
	pr_info("\n%s Value= \"%s\" \n", LPS22_PRS_DEV_NAME, buf);
#endif
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if ((val != 0) && (val != 1))
		goto exit;

#ifdef DEBUG
	pr_info("\n%s Valid val to put in reg2: %lu \r\n", LPS22_PRS_DEV_NAME, val);
#endif

	mutex_lock(&prs->lock);
	snsdata[0] = CTRL_REG2;
	err = lps22_prs_i2c_read(prs, snsdata, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_CTRL_REG2] = init_val;
	updated_val = (((mask) & (((u8)val)<<6)) | ((~mask) & (init_val)));

	snsdata[0] = CTRL_REG2;
	snsdata[1] = updated_val;
	err = lps22_prs_i2c_write(prs, snsdata, 1);

	mutex_unlock(&prs->lock);

	if (err < 0)
		return err;
	else
		prs->resume_state[RES_CTRL_REG2] = updated_val;

exit:
	return size;

}

static ssize_t attr_fifo_mode(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err= -1;
	u8 new_val;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS22_PRS_DEV_NAME, val);
#endif

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS22_PRS_DEV_NAME, val);
#endif

	mutex_lock(&prs->lock);
	x[0] = FIFO_CTRL;
	err = lps22_prs_i2c_read(prs, x, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	new_val = ( ((u8)val << 5) | (x[0] & ~FIFO_MODE_MASK) );

	x[0] = FIFO_CTRL;
	x[1] = new_val;
	err = lps22_prs_i2c_write(prs, x, 1);

	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}
	else
		prs->resume_state[RES_FIFO_CTRL] = new_val;

	mutex_unlock(&prs->lock);

	return size;
}

static ssize_t attr_set_samples_fifo(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int err= -1;
	u8 new_val;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS22_PRS_DEV_NAME, val);
#endif

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

#ifdef DEBUG
	pr_info("\n%s Valid val: %lu ", LPS22_PRS_DEV_NAME, val);
#endif

	mutex_lock(&prs->lock);
	x[0] = FIFO_CTRL;
	err = lps22_prs_i2c_read(prs, x, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}

	new_val = ( ( ((u8)val) - 1) | (x[0] & FIFO_MODE_MASK ) );

	x[0] = FIFO_CTRL;
	x[1] = new_val;
	err = lps22_prs_i2c_write(prs, x, 1);
	if (err < 0)
	{
		mutex_unlock(&prs->lock);
		return err;
	}
	else
		prs->resume_state[RES_FIFO_CTRL] = new_val;


	mutex_unlock(&prs->lock);

	return size;
}

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int rc;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&prs->lock);
	x[0] = prs->reg_addr;
	mutex_unlock(&prs->lock);
	x[1] = val;
	rc = lps22_prs_i2c_write(prs, x, 1);
	if (rc < 0)
		return rc;

	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t ret;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&prs->lock);
	data = prs->reg_addr;
	mutex_unlock(&prs->lock);
	rc = lps22_prs_i2c_read(prs, &data, 1);
	if (rc < 0)
		return rc;

	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&prs->lock);
	prs->reg_addr = val;
	mutex_unlock(&prs->lock);
	return size;
}

static ssize_t attr_reg_dump(struct device *dev, struct device_attribute *attr,char *buf)
{
	ssize_t ret;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	int err;
	u8 data = 0;
	u8 addr;

	printk("\r\n");
	mutex_lock(&prs->lock);
	for(addr=0x0B;addr<=0x2C;addr++){
		snsdata[0] = addr;
		err = lps22_prs_i2c_read(prs, snsdata, 1);
		if (err < 0) {
			printk("Error reading from register %02x \r\n",addr);
		} else {
			printk("register addr: %02x value: %02x \r\n",addr,snsdata[0]);
			if (addr == 0x0F)
				data = snsdata[0];
		}
	}
	mutex_unlock(&prs->lock);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_set_logout_decimation(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&prs->lock);
	logout_decimation = val;
	mutex_unlock(&prs->lock);
	return size;
}

static ssize_t attr_set_hex_measr_log(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if ((val != 0) && (val!=1))
		goto exit;

	mutex_lock(&prs->lock);
	hex_measr_logging = val;
	mutex_unlock(&prs->lock);

exit:
	return size;
}
#endif


static struct device_attribute attributes[] = {
	__ATTR(poll_period_ms, 0664, attr_get_polling_rate,attr_set_polling_rate),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(pressure_reference_level, 0664, attr_get_press_ref,attr_set_press_ref),
	__ATTR(pressure_threshold, 0664, attr_get_pthreshold, attr_set_pthreshold),
	__ATTR(enable_pthreshold_detection, 0664, attr_get_pthreshold_enable, attr_set_pthreshold_enable),
	__ATTR(enable_lc_mode, 0664, attr_get_lc_mode_enable, attr_set_lc_mode_enable),
	__ATTR(enable_lpf, 0664, attr_get_lpf_enable, attr_set_lpf_enable),
	__ATTR(lpf_cutoff_freq, 0664, attr_get_lpf_cutoff_freq, attr_set_lpf_cutoff_freq),
	__ATTR(enable_watermark, 0664, attr_get_watermark_enable, attr_set_watermark_enable),
	__ATTR(enable_autozero, 0222, NULL, attr_set_autozero),
	__ATTR(reset_autozero, 0222, NULL, attr_reset_autozero),
	__ATTR(enable_autorifp, 0222, NULL, attr_set_autorifp),
	__ATTR(reset_autorifp, 0222, NULL, attr_reset_autorifp),
	__ATTR(fifo_status, 0664, attr_get_fifo_status, NULL),
	__ATTR(status, 0664, attr_get_status, NULL),
	__ATTR(int_source, 0664, attr_get_interrupt_source, NULL),
	__ATTR(enable_fifo, 0222, NULL, attr_set_fifo),
	__ATTR(num_samples_fifo, 0222, NULL, attr_set_samples_fifo),
	__ATTR(fifo_mode, 0664, NULL, attr_fifo_mode),
#ifdef DEBUG
	__ATTR(reg_value, 0664, attr_reg_get, attr_reg_set),//DONE
	__ATTR(reg_addr, 0222, NULL, attr_addr_set),//DONE
	__ATTR(reg_dump, 0664, attr_reg_dump, NULL),
	__ATTR(dmesg_decimation, 0222,NULL,attr_set_logout_decimation),
	__ATTR(hex_measr_log, 0222,NULL,attr_set_hex_measr_log),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i, ret;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		ret = device_create_file(dev, attributes + i);
	if (ret < 0)
		goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return ret;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static void lps22_prs_input_work_func(struct work_struct *work)
{
	struct lps22_prs_data *prs = container_of(
			(struct delayed_work *)work,
			struct lps22_prs_data,
			input_work);

	static struct outputdata output;
	int err;

	mutex_lock(&prs->lock);
	err = lps22_prs_get_presstemp_data(prs, &output);
	if (err < 0)
		dev_err(&prs->client->dev, "get_pressure_data failed\n");
	else
		lps22_prs_report_values(prs, &output);

	schedule_delayed_work(&prs->input_work,
			msecs_to_jiffies(prs->pdata->poll_interval));

	mutex_unlock(&prs->lock);
}

int lps22_prs_input_open(struct input_dev *input)
{
	struct lps22_prs_data *prs = input_get_drvdata(input);

	return lps22_prs_enable(prs);
}

void lps22_prs_input_close(struct input_dev *dev)
{
	lps22_prs_disable(input_get_drvdata(dev));
}


static int lps22_prs_validate_pdata(struct lps22_prs_data *prs)
{
	/* checks for correctness of minimal polling period */
	prs->pdata->min_interval = (unsigned int)LPS22_PRS_MIN_POLL_PERIOD_MS;
	/* max((unsigned int)LPS22_PRS_MIN_POLL_PERIOD_MS,
	   prs->pdata->min_interval); */

	prs->pdata->poll_interval = max(prs->pdata->poll_interval,
			prs->pdata->min_interval);

	/* Checks polling interval relative to minimum polling interval */
	if (prs->pdata->poll_interval < prs->pdata->min_interval) {
		dev_err(&prs->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lps22_prs_input_init(struct lps22_prs_data *prs)
{
	int err;

	if (!use_threaded_interrupts())
		INIT_DELAYED_WORK(&prs->input_work, lps22_prs_input_work_func);

	prs->input_dev_pres = input_allocate_device();
	if (!prs->input_dev_pres) {
		err = -ENOMEM;
		dev_err(&prs->client->dev, "input device allocate failed\n");
		goto err0;
	}

	prs->input_dev_pres->open = lps22_prs_input_open;
	prs->input_dev_pres->close = lps22_prs_input_close;
	prs->input_dev_pres->name = LPS22_PRS_DEV_NAME;
	prs->input_dev_pres->id.bustype = BUS_I2C;
	prs->input_dev_pres->id.vendor = 0x0483; /* STMicro USB VID */
	prs->input_dev_pres->id.product = WHOAMI_LPS22_PRS;
	prs->input_dev_pres->id.version = DRIVER_VERSION;
	prs->input_dev_pres->dev.parent = &prs->client->dev;


	input_set_drvdata(prs->input_dev_pres, prs);

	set_bit(EV_ABS, prs->input_dev_pres->evbit);

	input_set_abs_params(prs->input_dev_pres, ABS_PR,
			PR_ABS_MIN, PR_ABS_MAX, FUZZ, FLAT);
	input_set_abs_params(prs->input_dev_pres, ABS_TEMP,
			TEMP_MIN, TEMP_MAX, FUZZ, FLAT);

	prs->input_dev_pres->name = "LPS22 barometer";

	err = input_register_device(prs->input_dev_pres);
	if (err) {
		dev_err(&prs->client->dev,
				"unable to register input polled device %s\n",
				prs->input_dev_pres->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(prs->input_dev_pres);
err0:
	return err;
}

static void lps22_prs_input_cleanup(struct lps22_prs_data *prs)
{
	input_unregister_device(prs->input_dev_pres);
	/* input_free_device(prs->input_dev);*/
}

static int lps22_prs_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct lps22_prs_data *prs;
	int err = -1;
	u8 buf[5];
	u32 smbus_func;


	pr_info("%s: probe start.\n", LPS22_PRS_DEV_NAME);


	smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;

	prs = kzalloc(sizeof(struct lps22_prs_data), GFP_KERNEL);
	if (prs == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
				"%d\n", err);
		goto err_exit_alloc_data_failed;
	}

	prs->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)){
			prs->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto err_exit_check_functionality_failed;
		}
	}

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODATA;
		goto err_exit_check_functionality_failed;
	}

	msleep(100);

	mutex_init(&prs->lock);
	mutex_lock(&prs->lock);

	prs->client = client;
	i2c_set_clientdata(client, prs);

	prs->pdata = kmalloc(sizeof(*prs->pdata), GFP_KERNEL);
	if(prs->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlockfreedata;
	}

	if (client->dev.platform_data == NULL) {
		memcpy(prs->pdata, &default_lps22_pdata,
				sizeof(*prs->pdata));
		dev_info(&client->dev, "using default plaform_data for "
				"lps22\n");
	} else {
		memcpy(prs->pdata, client->dev.platform_data,
				sizeof(*prs->pdata));
		dev_info(&client->dev, "using user plaform_data for "
				"lps22\n");
	}

	if (prs->pdata->init) {
		err = prs->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "lps pdata init failed: "
					"%d\n", err);
			goto err_pdata_init;
		}
	}


	/* read chip id */
	buf[0] = 0x0f;

	err = lps22_prs_i2c_read(prs, buf, 1);
	msleep(60);

	if (err < 0) {
		dev_warn(&prs->client->dev, "Error reading WHO_AM_I: is device"
				" available/working?\n");
		goto err_mutexunlockfreedata;
	} else
		prs->hw_working = 1;

	if (buf[0] != 0xB1) {
		dev_err(&prs->client->dev,
				"device unknown. Expected: 0x%02x,"
				" Replies: 0x%02x\n", 0xBD, buf[0]);
		err = -1;
		goto err_mutexunlockfreedata;
	}

	pr_info("%s ID Chip OK \n", LPS22_PRS_DEV_NAME);
	msleep(60);

	err = lps22_prs_validate_pdata(prs);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err_exit_kfree_pdata;
	}


	if (prs->pdata->init) {
		err = prs->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_exit_pointer;
		}
	}

	memset(prs->resume_state, 0, ARRAY_SIZE(prs->resume_state));

	/* init registers which need values different from zero */
	prs->resume_state[RES_CTRL_REG1] = (ODR_MASK & ODR_1_1) | (BDU_MASK);

	err = lps22_prs_device_power_on(prs);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_exit_pointer;
	}

	atomic_set(&prs->enabled, 1);

	err = lps22_prs_update_odr(prs, prs->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err_power_off;
	}


	err = lps22_prs_input_init(prs);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
				"device LPS22_PRS_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lps22_prs_device_power_off(prs);

	/* As default, do not report information */
	atomic_set(&prs->enabled, 0);

	mutex_unlock(&prs->lock);

	dev_info(&client->dev, "%s: probed\n", LPS22_PRS_DEV_NAME);

	buf[0] = CTRL_REG2;
	buf[1] = 0x10;
	err = lps22_prs_i2c_write(prs, buf, 1);
	if (err < 0)
		return err;

	return 0;

/*
err_remove_sysfs_int:
lps22_prs_remove_sysfs_interfaces(&client->dev);
*/
err_input_cleanup:
	lps22_prs_input_cleanup(prs);
err_power_off:
	lps22_prs_device_power_off(prs);
err_exit_pointer:
	if (prs->pdata->exit)
		prs->pdata->exit();
err_pdata_init:
	if (prs->pdata->exit)
		prs->pdata->exit();
err_exit_kfree_pdata:
	kfree(prs->pdata);

err_mutexunlockfreedata:
	mutex_unlock(&prs->lock);
	kfree(prs);
err_exit_alloc_data_failed:
err_exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LPS22_PRS_DEV_NAME);
	return err;
}

static int __devexit lps22_prs_remove(struct i2c_client *client)
{
	struct lps22_prs_data *prs = i2c_get_clientdata(client);

	lps22_prs_input_cleanup(prs);
	lps22_prs_device_power_off(prs);
	remove_sysfs_interfaces(&client->dev);

	if (prs->pdata->exit)
		prs->pdata->exit();
	kfree(prs->pdata);
	kfree(prs);

	return 0;
}

#ifdef CONFIG_PM

static int lps22_prs_resume(struct i2c_client *client)
{
	struct lps22_prs_data *prs = i2c_get_clientdata(client);

	if (prs->on_before_suspend)
		return lps22_prs_enable(prs);
	return 0;
}

static int lps22_prs_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lps22_prs_data *prs = i2c_get_clientdata(client);

	prs->on_before_suspend = atomic_read(&prs->enabled);
	return lps22_prs_disable(prs);
}

#else

#define lps001wp_prs_resume	    NULL
#define	lps001wp_prs_suspend	NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id lps22_prs_id[]
= { { LPS22_PRS_DEV_NAME, 0}, { },};

MODULE_DEVICE_TABLE(i2c, lps22_prs_id);

static struct i2c_driver lps22_prs_driver = {
	.driver = {
		.name = LPS22_PRS_DEV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = lps22_prs_probe,
	.remove = __devexit_p(lps22_prs_remove),
	.id_table = lps22_prs_id,
#ifdef CONFIG_PM
	.resume = lps22_prs_resume,
	.suspend = lps22_prs_suspend,
#endif
};

static int __init lps22_prs_init(void)
{
#ifdef DEBUG
	pr_debug("%s barometer driver: init\n", LPS22_PRS_DEV_NAME);
#endif
	return i2c_add_driver(&lps22_prs_driver);
}

static void __exit lps22_prs_exit(void)
{
#ifdef DEBUG
	pr_debug("%s barometer driver exit\n", LPS22_PRS_DEV_NAME);
#endif
	i2c_del_driver(&lps22_prs_driver);
	return;
}

module_init(lps22_prs_init);
module_exit(lps22_prs_exit);

MODULE_DESCRIPTION("STMicrolelectronics lps22 pressure sensor sysfs driver");
MODULE_AUTHOR("HESA BU, STMicroelectronics");
MODULE_LICENSE("GPL");
