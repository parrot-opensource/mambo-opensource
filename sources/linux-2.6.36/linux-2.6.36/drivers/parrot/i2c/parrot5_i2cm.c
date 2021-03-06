/**
 * @file parrot5_i2cm.c
 *
 * @brief Parrot I2C Master driver
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     david.guilloteau@parrot.com
 * @author     matthieu.castet@parrot.com
 * @date       2008-08-28
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

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>

#include <asm/io.h>
#include <mach/parrot.h>
#include <mach/regs-i2c-p5.h>
#include <mach/i2c.h>

#define dprintk(...)


#define PARROT5_I2CM_NAME "parrot5-i2cm"

#define PARROT5_I2CM_TIMEOUT 10000 // in ms

struct parrot5_i2cm_data {

	spinlock_t		        lock;
	wait_queue_head_t	        wait;
	int wait_up;
	int stop_sent;

	struct i2c_msg* msgs;
	int msgs_num;
	int msgs_idx;
	int data_pos;
	int state;
	unsigned int                    status;

	struct i2c_adapter	        adapter;
	struct parrot5_i2cm_platform	*pdata;

	struct clk		        *clk;

	struct resource		        *ioarea;
	void __iomem		        *base;
};

enum {
	RESTART,
	STOP,
	DATA_W,
	DATA_R,
};

/**
 * Interrupt functions
 */
static void parrot5_i2cm_enable_irq(struct parrot5_i2cm_data *drv_data)
{
	writel(I2CM_ITEN_ITEN, drv_data->base + I2CM_ITEN);
}

static void parrot5_i2cm_disable_irq(struct parrot5_i2cm_data *drv_data)
{
	writel(I2CM_ITEN_ITDIS, drv_data->base + I2CM_ITEN);
}

static void parrot5_i2cm_acknowledge_irq(struct parrot5_i2cm_data *drv_data)
{
	writel(I2CM_COMMAND_ITACK, drv_data->base + I2CM_COMMAND);
}

static void parrot5_i2cm_write_byte(struct parrot5_i2cm_data *drv_data,
				   char byte, int stop)
{
	int ctrl;

	dprintk("write byte %x (stop %d)\n", byte, stop);
	ctrl = I2CM_COMMAND_WR;

	if (stop) {
		ctrl |= I2CM_COMMAND_STO;
		drv_data->stop_sent = 1;
	}

	writel(byte, drv_data->base + I2CM_TRANSMIT);
	writel(ctrl, drv_data->base + I2CM_COMMAND);
}

static void parrot5_i2cm_read_byte(struct parrot5_i2cm_data *drv_data,
					struct i2c_msg* msg,
				  int noack, int stop)
{
	int ctrl;
	dprintk("read byte (noack %d, stop %d)\n", noack, stop);

	ctrl = I2CM_COMMAND_RD;

	if (stop) {
		ctrl |= I2CM_COMMAND_STO;
		drv_data->stop_sent = 1;
	}

	if (noack) {
		ctrl |= I2CM_COMMAND_NACK;
	}
	else if (!(msg->flags & I2C_M_NO_RD_ACK)) {
		ctrl |= I2CM_COMMAND_ACK;
	}
	writel(ctrl, drv_data->base + I2CM_COMMAND);
}

static int parrot5_i2cm_get_addr(struct i2c_msg *msg)
{
	int addr = (msg->addr & 0x7f) << 1;

	if (msg->flags & I2C_M_RD) {
		addr |= 1;
	}

	return addr;
}

static void parrot5_i2cm_start_message(struct parrot5_i2cm_data *drv_data,
				      struct i2c_msg* msg, int stop)
{
	int addr, ctrl;

	ctrl = (I2CM_COMMAND_STA | I2CM_COMMAND_WR);
	if (stop) {
		ctrl |= I2CM_COMMAND_STO;
		drv_data->stop_sent = 1;
	}

	addr = parrot5_i2cm_get_addr(msg);

	dprintk("start addr %x\n", addr);
	writel(addr, drv_data->base + I2CM_TRANSMIT);
	writel(ctrl, drv_data->base + I2CM_COMMAND);
}

static void parrot5_i2cm_send_stop(struct parrot5_i2cm_data *drv_data)
{
	dprintk("stop\n");
	drv_data->stop_sent = 1;
	writel(I2CM_COMMAND_STO, drv_data->base + I2CM_COMMAND);
}

static int parrot5_i2cm_wait_transfert(struct parrot5_i2cm_data *drv_data, u32 flags)
{
	u32 status;
	int ret = -1;
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);

	/* wait the end of current transfert */
	do {
		status = readl(drv_data->base + I2CM_STATUS);
		if ((status & flags) == 0) {
			ret = 0;
			break;
		}
		msleep(1); //XXX
	} while (time_is_after_jiffies(timeout));
	return ret;
}

static int parrot5_i2cm_need_reset(u32 status)
{
	/* reset if status is not clean */
	return (status & ~(I2CM_STATUS_RXACK|I2CM_STATUS_AL|I2CM_STATUS_BUSY));
}

/* reset controller state :
 wait end of TIP, [end of busy].
 clean irq [and busy]

 busy is touched only is clean_busy is true (we should own the bus)
 */
static int parrot5_i2cm_reset(struct parrot5_i2cm_data *drv_data, int clean_busy)
{
	u32 status;
	int ret;
	/* disable irq */
	parrot5_i2cm_disable_irq(drv_data);

	/* wait a stable state */
	ret = parrot5_i2cm_wait_transfert(drv_data, I2CM_STATUS_TIP);
	if (ret) {
		printk("timeout while waiting the end of current transfert\n");
		goto exit;
	}
	status = readl(drv_data->base + I2CM_STATUS);
	if (clean_busy && drv_data->stop_sent && (status & I2CM_STATUS_BUSY)) {
		ret = parrot5_i2cm_wait_transfert(drv_data, I2CM_STATUS_BUSY);
		if (ret) {
			printk("timeout while waiting busy\n");
			goto exit;
		}
	}

	/* remove pending irq */
	if (status & I2CM_STATUS_IF)
		parrot5_i2cm_acknowledge_irq(drv_data);

	status = readl(drv_data->base + I2CM_STATUS);

	/* send stop if needed */
	if (clean_busy && (status & I2CM_STATUS_BUSY)) {
		parrot5_i2cm_send_stop(drv_data);
		ret = parrot5_i2cm_wait_transfert(drv_data, I2CM_STATUS_BUSY);
		if (ret) {
			printk("timeout while sending stop bit\n");
			goto exit;
		}
	}

exit:
	parrot5_i2cm_enable_irq(drv_data);
	return 0;
}

static irqreturn_t parrot5_i2cm_irq(int irq, void *dev_id)
{
	struct parrot5_i2cm_data *drv_data = dev_id;
	unsigned int status = readl(drv_data->base + I2CM_STATUS);
	struct i2c_msg* msg = &drv_data->msgs[drv_data->msgs_idx];
	int noack;
	int stop;

	if ((status & I2CM_STATUS_IF) == 0)
		return IRQ_NONE;

	/* TODO. There is a potential optimisation here : we could acknowledged
	  irq in the next cmd */
	/* acknowledge interrupt */
	parrot5_i2cm_acknowledge_irq(drv_data);

	dprintk("irq status %x state %d\n", status, drv_data->state);
	if (status & I2CM_STATUS_AL) {
		/* abitration lost */
		drv_data->status = I2CM_STATUS_AL;
		/* abort transfert without generating stop */
		drv_data->wait_up = 1;
	}
	else if (drv_data->state == DATA_R) {
		WARN_ON(!(msg->flags & I2C_M_RD));
		/* read received data */
		msg->buf[drv_data->data_pos] = readl(drv_data->base + I2CM_TRANSMIT);
	}
	else if ((status & I2CM_STATUS_RXACK) && !(msg->flags & I2C_M_IGNORE_NAK)) {
		/* nak */
		drv_data->status = I2CM_STATUS_RXACK;
	}

	/* if it was last cmd, return */
	if (drv_data->stop_sent || drv_data->wait_up) {
		drv_data->wait_up = 1;
		wake_up(&drv_data->wait);
		goto exit;
	}


	BUG_ON(drv_data->state == STOP);

	/* after here we will sumbit a new command */

	if (drv_data->status) {
		/* nak, abort transfert : we didn't submit stop if we are here */
		drv_data->state = STOP;
	}
	else if (drv_data->state == RESTART && msg->len) {
		if (msg->flags & I2C_M_RD)
			drv_data->state = DATA_R;
		else
			drv_data->state = DATA_W;
	}
	else {
		if (drv_data->data_pos + 1 >= msg->len) {
			/* we are at the end of the data buffer,
			   check next operation */
			if (drv_data->msgs_idx + 1 >= drv_data->msgs_num) {
				/* no more operation */
				drv_data->state = STOP;
				WARN_ON(1);
			}
			else {
				drv_data->data_pos = 0;
				drv_data->msgs_idx++;
				msg = &drv_data->msgs[drv_data->msgs_idx];
#if 0
				/* XXX we could avoid restart between some transaction
				   I2C_M_NOSTART but we don't support it ATM
				 */
				if (!(msg->flags & I2C_M_NOSTART &&
						((msg->flags & I2C_M_RD) && drv_data->state == DATA_R) ||
						(!(msg->flags & I2C_M_RD) && drv_data->state == DATA_W)
				   ))

#endif
				/* restart */
				drv_data->state = RESTART;
			}
		}
		else {
			drv_data->data_pos++;
		}
	}
	dprintk("data pos %d, len %d\n", drv_data->data_pos, msg->len);
	dprintk("msg idx %d, num %d\n", drv_data->msgs_idx, drv_data->msgs_num);

	/* for read nack the last read */
	noack = (drv_data->data_pos + 1 == msg->len);
	/* send stop when last data (noack) and last cmd */
	stop = (noack && drv_data->msgs_idx + 1 == drv_data->msgs_num);

	switch (drv_data->state) {
		case DATA_W:
			parrot5_i2cm_write_byte(drv_data,
				msg->buf[drv_data->data_pos], stop);
		break;
		case DATA_R:
			parrot5_i2cm_read_byte(drv_data, msg, noack, stop);
		break;
		case RESTART:
			/* XXX what happen if addr change */
			parrot5_i2cm_start_message(drv_data, msg, stop && !msg->len);
		break;
		case STOP:
			parrot5_i2cm_send_stop(drv_data);
		break;
	}

exit:
	return IRQ_HANDLED;
}

/**
 * Hardware init
 */
static int parrot5_i2cm_hw_init(struct parrot5_i2cm_data *drv_data, int id)
{
	int prescale = 0;
	int ret = 0;
	const char *const clk_name = id ? "i2cm1" :"i2cm";

	/* enable I2C master clock */
	drv_data->clk = clk_get(NULL, clk_name);
	if (IS_ERR(drv_data->clk)) {
		ret = -EINVAL;
		goto exit;
	}
	clk_enable(drv_data->clk);

	/* reset register */
	/* XXX does it do something ??? */
	writel(0, drv_data->base + I2CM_COMMAND);
	parrot5_i2cm_acknowledge_irq(drv_data);

	/* program prescale register */
	prescale = clk_get_rate(drv_data->clk) / 5 / drv_data->pdata->bus_freq - 1;
	if (prescale < 1)
		prescale = 1;
	writel(prescale, drv_data->base + I2CM_PRESCALE);
	if (readl(drv_data->base + I2CM_PRESCALE) != prescale) {
		writel(0xffffffff, drv_data->base + I2CM_PRESCALE);

		WARN(1, KERN_ERR"i2cm prescaler is too big %d (max %d)\n",
				prescale, readl(drv_data->base + I2CM_PRESCALE));
	}

	/* interrupt mode  */
	parrot5_i2cm_enable_irq(drv_data);

exit:
	return ret;
}

#ifdef CONFIG_HBRIDGE_SAFETY_HACK
#include <linux/gpio.h>
enum {
	PWM_UNKNOWN = -1,
	PWM_OFF,
	PWM_ON,
	PWM_PWM,
	PWM_PWMG,
};

static int pwm0_state_bkp = PWM_UNKNOWN;
static int pwm1_state_bkp = PWM_UNKNOWN;
static int pwm0_ratio_bkp = -1;
static int pwm1_ratio_bkp = -1;
int pwm0_output_bkp = -1;
int pwm1_output_bkp = -1;
EXPORT_SYMBOL(pwm0_output_bkp);
EXPORT_SYMBOL(pwm1_output_bkp);

static int check_pca9633_state(struct i2c_msg *msgs, int num, int save)
{
	int i;
	int pca_reg = -1;

	int pwm0_state = pwm0_state_bkp;
	int pwm1_state = pwm1_state_bkp;
	int pwm0_ratio = pwm0_ratio_bkp;
	int pwm1_ratio = pwm1_ratio_bkp;
	int pwm0_output = pwm0_output_bkp;
	int pwm1_output = pwm1_output_bkp;

	for (i = 0; i < num; i++) {
		int idx = 0;
		if ((msgs[i].addr & 0x7f) == 0x70) {
			printk("pca9633 all call addr\n");
			return -EREMOTEIO;
		}
		if ((msgs[i].addr & 0x7f) == 0x3) {
			printk("pca9633 sw reset\n");
			if (msgs[i].len != 2) {
				printk("pca9633 wrong size %d\n", msgs[i].len);
				return -EREMOTEIO;
			}
			if (msgs[i].buf[0] != 0xa5 && msgs[i].buf[1] != 0x5a) {
				printk("pca9633 wrong args\n");
				return -EREMOTEIO;
			}
			pwm0_state = PWM_OFF;
			pwm1_state = PWM_OFF;
			pwm0_ratio = 0;
			pwm1_ratio = 0;
			pwm0_output = 1;
			pwm1_output = 1;
		}
		if ((msgs[i].addr & 0x7f) == 0x62 || (msgs[i].addr & 0x7f) == 0x15) {
			/* write transaction, we will reset the register addr */
			if (!(msgs[i].flags & I2C_M_RD))
				pca_reg = -1;
			if (msgs[i].len == 0) {
				printk("pca9633 null size\n");
				return -EREMOTEIO;
			}
			if (pca_reg == -1) {
				/* want a write transaction for register setting */
				if (msgs[i].flags & I2C_M_RD) {
					printk("pca9633 not setting regiter\n");
					return -EREMOTEIO;
				}
				pca_reg = msgs[i].buf[idx++];
				if (!((pca_reg & 0xe0) == 0x80 || (pca_reg & 0xe0) == 0x00)) {
					printk("pca9633 increment mode not supported\n");
					return -EREMOTEIO;
				}
			}
			/* We are strict here for simplicity. But instead of expecting a reset,
			   we could check that all important regs are init
			 */
			if (pwm0_state == PWM_UNKNOWN || pwm1_state == PWM_UNKNOWN) {
				printk("pca9633 access reg without reset %d\n", msgs[i].buf[0]);
				return -EREMOTEIO;
			}
			for (; idx < msgs[i].len; idx++) {
				/* parse data */
				if (!(msgs[i].flags & I2C_M_RD)) {
					char val = msgs[i].buf[idx];
					switch (pca_reg & 0xf) {
						case 0:
							if (val != 0 && val != 1) {
								printk("trying to set mode1 to %d\n", val);
								return -EREMOTEIO;
							}
							break;
						case 0x1:
							if (val != 1) {
								printk("trying to set mode2 to %d\n", val);
								return -EREMOTEIO;
							}
							break;
						case 0x8:
							pwm0_state = val & 3;
							pwm1_state = (val>>2) & 3;
							if (pwm0_state == PWM_ON)
								pwm0_output = 0;
							else if (pwm0_state == PWM_OFF)
								pwm0_output = 1;
							else {
								if (pwm0_ratio)
									pwm0_output = 0;
								else
									pwm0_output = 1;
							}
							if (pwm1_state == PWM_ON)
								pwm1_output = 0;
							else if (pwm1_state == PWM_OFF)
								pwm1_output = 1;
							else {
								if (pwm1_ratio)
									pwm1_output = 0;
								else
									pwm1_output = 1;
							}
							break;
						case 0x2:
							/* pwm0 duty */
							pwm0_ratio = val;
							if (pwm0_state == PWM_PWM || pwm0_state == PWM_PWMG) {
								if (pwm0_ratio)
									pwm0_output = 0;
								else
									pwm0_output = 1;
							}
							break;
						case 0x3:
							/* pwm0 duty */
							pwm1_ratio = val;
							if (pwm1_state == PWM_PWM || pwm1_state == PWM_PWMG) {
								if (pwm1_ratio)
									pwm1_output = 0;
								else
									pwm1_output = 1;
							}
							break;
					}

					if (pwm0_output == 0 && gpio_get_value(38)) {
						printk("pca9633 pwm0 will go 0 while gpio (OEn%d)\n", gpio_get_value(42));
						printk("pca9633 reg[0x%x]=0x%x\n", pca_reg & 0xf, val);
						return -EREMOTEIO;
					}
					if (pwm1_output == 0 && gpio_get_value(39)) {
						printk("pca9633 pwm1 will go 0 while gpio (OEn%d)\n", gpio_get_value(42));
						printk("pca9633 reg[0x%x]=0x%x\n", pca_reg & 0xf, val);
						return -EREMOTEIO;
					}

				}

				/* auto increment */
				if (pca_reg & 0x80) {
					pca_reg++;
					if (pca_reg == 0x8d)
						pca_reg = 0x80;
				}
			}
		}
	}
	if (save) {
		pwm0_state_bkp = pwm0_state;
		pwm1_state_bkp = pwm1_state;
		pwm0_ratio_bkp = pwm0_ratio;
		pwm1_ratio_bkp = pwm1_ratio;
		pwm0_output_bkp = pwm0_output;
		pwm1_output_bkp = pwm1_output;
	}
	return 0;
}

static void set_pca9633_error(struct i2c_msg *msgs, int num)
{
	/* if there is an error on the transaction
	   set unknow state.
	   With current code, this will abort all new transaction
	   if no reset is done.
	   Because output are undef, setting gpio to 1 will be filtered.
	 */
	int i;

	int pwm0_state = pwm0_state_bkp;
	int pwm1_state = pwm1_state_bkp;
	int pwm0_ratio = pwm0_ratio_bkp;
	int pwm1_ratio = pwm1_ratio_bkp;
	int pwm0_output = pwm0_output_bkp;
	int pwm1_output = pwm1_output_bkp;

	for (i = 0; i < num; i++) {
		if ((msgs[i].addr & 0x7f) == 0x3) {
			pwm0_state = PWM_UNKNOWN;
			pwm1_state = PWM_UNKNOWN;
			pwm0_ratio = -1;
			pwm1_ratio = -1;
			pwm0_output = -1;
			pwm1_output = -1;
			printk("pca9633 reset transaction error\n");
		}
		if ((msgs[i].addr & 0x7f) == 0x62 || (msgs[i].addr & 0x7f) == 0x15) {
			pwm0_state = PWM_UNKNOWN;
			pwm1_state = PWM_UNKNOWN;
			pwm0_ratio = -1;
			pwm1_ratio = -1;
			pwm0_output = -1;
			pwm1_output = -1;
			printk("pca9633 transaction error\n");
		}
	}
	pwm0_state_bkp = pwm0_state;
	pwm1_state_bkp = pwm1_state;
	pwm0_ratio_bkp = pwm0_ratio;
	pwm1_ratio_bkp = pwm1_ratio;
	pwm0_output_bkp = pwm0_output;
	pwm1_output_bkp = pwm1_output;
}

#endif
/**
 *  I2C transfer main function
 */
static int parrot5_i2cm_master_xfer(struct i2c_adapter *i2c_adap,
				    struct i2c_msg *msgs,
				    int num)
{
	struct parrot5_i2cm_data *drv_data = i2c_adap->algo_data;
	int ret = num;
	int timeout;
	int time_left;
	u32 status;
	int retry = 0;
	unsigned long flags;

#ifdef CONFIG_HBRIDGE_SAFETY_HACK
	if (check_pca9633_state(msgs, num, 1))
		return -EREMOTEIO;
#endif

retry_transfert:
	retry++;
	if (retry > 10) {
		printk("i2c too much retry. Aborting.\n");
		goto xfer_error;
	}

	status = readl(drv_data->base + I2CM_STATUS);

	if (parrot5_i2cm_need_reset(status)) {
		printk("bad status before transfert %x, resetting\n", status);
		parrot5_i2cm_reset(drv_data, 0);
		status = readl(drv_data->base + I2CM_STATUS);
		printk("status after reset %x\n", status);
	}

	if (parrot5_i2cm_need_reset(status)) {
		printk("can't reset i2c : aborting\n");
		return -EREMOTEIO;
	}

	drv_data->msgs = msgs;
	drv_data->msgs_num = num;
	drv_data->msgs_idx = 0;
	drv_data->data_pos = 0;
	drv_data->state = RESTART;
	drv_data->status = 0;
	drv_data->stop_sent = 0;
	drv_data->wait_up = 0;


	/* if another master take the bus before we issue the start condition we
	   have a race. We should make the busy check + sending start in less
	   than 4 �s @ 100Khz. That with we disable irq.

	   an disassembling show the lock is taken for 30 instructions.
	 */
	local_irq_save(flags);
	status = readl(drv_data->base + I2CM_STATUS);
	if (status & I2CM_STATUS_BUSY) {
		local_irq_restore(flags);
		printk("i2c bus is busy. Waiting...\n");
		goto arbitration_error;
	}

	parrot5_i2cm_start_message(drv_data, msgs, num == 1 && msgs[0].len == 0);
	local_irq_restore(flags);

	timeout = msecs_to_jiffies(drv_data->adapter.timeout);
	time_left = wait_event_timeout(drv_data->wait,
							     drv_data->wait_up,
							     timeout);

	if (drv_data->status == I2CM_STATUS_AL) {
		printk("i2c arbitration lost. Waiting...\n");
		goto arbitration_error;
	}

	status = readl(drv_data->base + I2CM_STATUS);

	if (time_left <= 0 || parrot5_i2cm_need_reset(status) ||
			(status & I2CM_STATUS_BUSY)) {
		printk("xfer timeout %d, wake %d,%d\n", time_left,
				drv_data->stop_sent, drv_data->wait_up);
		printk("xfer end  hw status %x, status %d\n",
			status,
			drv_data->status);
	}

	if ((status & I2CM_STATUS_BUSY) && drv_data->stop_sent == 0) {
		/* we own the bus and need to release it */
		parrot5_i2cm_reset(drv_data, 1);
	}

	if (parrot5_i2cm_need_reset(status)) {
		printk("bad status after transfert %x, resetting\n", status);
		parrot5_i2cm_reset(drv_data, 0);
		status = readl(drv_data->base + I2CM_STATUS);
		printk("status after reset %x\n", status);
	}

	if (time_left == 0) {
		printk("i2c timeout\n");
		ret = -EREMOTEIO;
	}
	else if (drv_data->status)
		ret = -EREMOTEIO;

#ifdef CONFIG_HBRIDGE_SAFETY_HACK
	if (ret != num) {
		set_pca9633_error(msgs, num);
	}
#endif
	return ret;

arbitration_error:
	/* an other master is driving the bus, wait for the stop condition */
	if (parrot5_i2cm_wait_transfert(drv_data, I2CM_STATUS_BUSY)) {
		printk("i2c : ab timeout while waiting busy\n");
		goto xfer_error;
	}
	goto retry_transfert;

xfer_error:
#ifdef CONFIG_HBRIDGE_SAFETY_HACK
	set_pca9633_error(msgs, num);
#endif
	return -EREMOTEIO;
}

static u32 parrot5_i2cm_funct(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm parrot5_i2cm_algorithm = {
	.master_xfer	= parrot5_i2cm_master_xfer,
	.functionality	= parrot5_i2cm_funct,
};

/**
 * probe function
 */
static int parrot5_i2cm_probe(struct platform_device *pdev)
{
	struct parrot5_i2cm_data *drv_data;
	struct resource *res;
	int ret;

	drv_data = kzalloc(sizeof(struct parrot5_i2cm_data), GFP_KERNEL);
	if (!drv_data) {
		ret = -ENOMEM;
		goto no_mem;
	}

	drv_data->pdata = pdev->dev.platform_data;
	if (!drv_data->pdata) {
		dev_err(&pdev->dev, "no driver data\n");
		ret = -ENODEV;
		goto no_pdata;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "get memory region resource failed\n");
		ret = -ENOENT;
		goto no_res;
	}

	drv_data->ioarea = request_mem_region(res->start,
					      res->end - res->start + 1,
					      pdev->name);
	if (!drv_data->ioarea) {
		dev_err(&pdev->dev, "request memory region failed\n");
		ret = -ENOENT;
		goto no_res;
	}

	drv_data->base = ioremap(res->start, res->end - res->start + 1);
	if (!drv_data->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto no_map;
	}

	ret = parrot5_i2cm_hw_init(drv_data, pdev->id);
	if (ret) {
		dev_err(&pdev->dev, "Hardware init failed (%d)\n", ret);
		goto err_hw;
	}

	init_waitqueue_head(&drv_data->wait);

	platform_set_drvdata(pdev, drv_data);

	ret = request_irq(platform_get_irq(pdev, 0),
			  parrot5_i2cm_irq, IRQF_SHARED,
			  pdev->name, drv_data);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed (%d)\n", ret);
		goto no_irq;
	}


	/* setup info for the i2c core */
	memcpy(drv_data->adapter.name, pdev->name, strlen(pdev->name));
	drv_data->adapter.algo = &parrot5_i2cm_algorithm;
	drv_data->adapter.algo_data = drv_data;
	drv_data->adapter.dev.parent = &pdev->dev;
	drv_data->adapter.owner = THIS_MODULE;
	drv_data->adapter.timeout = PARROT5_I2CM_TIMEOUT;

	/* we don't use i2c_add_adapter to allow
	   board config to use i2c_register_board_info
	 */
	drv_data->adapter.nr = pdev->id;
	i2c_add_numbered_adapter(&drv_data->adapter);

	dev_info(&pdev->dev, "controller probe successfully\n");

	return 0;

no_irq:
err_hw:
no_map:
	iounmap(drv_data->base);
	release_resource(drv_data->ioarea);
no_res:
no_pdata:
	kfree(drv_data);

no_mem:
	return ret;

}

/**
 * remove function
 */
static int parrot5_i2cm_remove(struct platform_device *pdev)
{
	struct parrot5_i2cm_data *drv_data = platform_get_drvdata(pdev);

	parrot5_i2cm_disable_irq(drv_data);
	clk_disable(drv_data->clk);
	clk_put(drv_data->clk);
	free_irq(platform_get_irq(pdev, 0), drv_data);

	iounmap(drv_data->base);

	release_resource(drv_data->ioarea);

	kfree(drv_data);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/**
 * platform driver structure
 */
static struct platform_driver parrot5_i2cm_driver = {
	.probe		= parrot5_i2cm_probe,
	.remove		= parrot5_i2cm_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PARROT5_I2CM_NAME,
	},
};

/**
 * init function
 */
static int __init parrot5_i2cm_init(void)
{
	return platform_driver_register(&parrot5_i2cm_driver);
}
module_init(parrot5_i2cm_init);

/**
 * exit function
 */
static void __exit parrot5_i2cm_exit(void)
{
	platform_driver_unregister(&parrot5_i2cm_driver);
}
module_exit(parrot5_i2cm_exit);

MODULE_AUTHOR("Parrot SA by David Guilloteau, Matthieu CASTET");
MODULE_DESCRIPTION("Parrot I2C Master driver");
MODULE_LICENSE("GPL");
