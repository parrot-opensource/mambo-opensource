/**
 ********************************************************************************
 * @file p6_pwm.c
 * @brief kernel driver for p6 pwm
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     François Guillemé <francois.guilleme@parrot.com>
 * @date       1-Oct-2008
 ********************************************************************************
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <linux/err.h>
#include <linux/clk.h>
#include <asm/uaccess.h>

#include <mach/hardware.h>
#include <mach/parrot.h>
#include <mach/map-p6.h>
#include <mach/regs-pwm-p6.h>

#include "pwm_ops.h"
#include "pwm_ioctl.h"

#include <mach/delos.h>

#define PWM_NB_TIMER 16

#define BFMASK(_start, _width) (((1 << (_width))-1) << (_start))

#define BFSET(_v, _start, _width, _val) \
	do { \
		_v &= ~(BFMASK(_start, _width)); \
		_v |= ((_val & ((1 << (_width))-1)) << _start); \
	} while (0);

#define BFGET(_v,_start,_width) \
	(( _v  & BFMASK(_start, _width)) >> _start)

static const int delos_verbose = 0;
static struct clk *pwm_clk;
static char __iomem *pwm_regbase;
static uint32_t pwm_active;		// we can handle up to 32 timers

static uint32_t delos_ratio;
static uint32_t delos_speed;

static uint8_t delos_pwms_enabled = 0;


static bool px_pwm_is_active(uint32_t n)
{
	return n < PWM_NB_TIMER ? BFGET(pwm_active, n, 1) != 0 : false;
}

static inline void px_pwm_set_active(uint32_t n, bool active)
{
	// activate the clock if we start the first timer
	if (active && pwm_active == 0) {
		printk(KERN_DEBUG "start pwm clock\n");
		clk_enable(pwm_clk);
	}

	BFSET(pwm_active, n, 1, active);

#ifdef STOP_ON_EXIT
	// stop the clock if we stop the last timer
	if (!active && pwm_active == 0) {
		printk(KERN_DEBUG "stop pwm clock\n");
		clk_disable(pwm_clk);
	}
#endif
}

/**
 * @brief set the clock mode of a channel
 *
 * @param ntimer
 * @param clock mode
 */
static void px_pwm_set_mode(int ntimer, bool mode)
{
	int offset = 4 + (ntimer & 0x03) + (ntimer >> 2) * 8;
	uint32_t val;

	val = __raw_readl(pwm_regbase + P6_PWM_CTL);

	BFSET(val, offset, 1, mode);

	__raw_writel(val, pwm_regbase + P6_PWM_CTL);
}

/**
 * @brief enable or disable a pwm channel
 *
 * @param ntimer
 * @param start
 */
static void px_pwm_set_start(int ntimer, bool start)
{
	uint32_t val;
	int offset = (ntimer & 0x03) + (ntimer >> 2) * 8;

	val = __raw_readl(pwm_regbase + P6_PWM_CTL);

	BFSET(val, offset, 1, start);

	__raw_writel(val, pwm_regbase + P6_PWM_CTL);
}

/* Note to get a good ratio precision, frequency should be choosen so that
   pclk/(2*freq) got lot's of 0 at the end.
   Ie freq =pclk/(2*val), where the number of leading 0 are the ratio precision.
   But there is an overflow for 156Mhz starting for freq = pclk/(2*0x3000)
   ie freq < 9521Hz.
   */
static unsigned int compute_speed(unsigned int speed, unsigned int *length)
{
	unsigned int pclk = clk_get_rate(pwm_clk);
	unsigned int val = (pclk + speed) / ( speed * 2);
	unsigned int tmp;

	if (val <= 1)
		val = 1;

	/* *length >= 0 */
	*length = ffs(val) - 1;

	/* fallback to p5+ config */
	if (*length < 8 && val >= 0xf00)
		*length = 8;

	tmp = fls(val >> *length);
	if (tmp > 16) {
		*length += tmp - 16;
	}

	if (*length > 0) {
		/* round it */
		val += 1 << (*length - 1);
		val >>= *length;
	}

	val -= 1;
	/* should never happen because val is a 32 bits int */
	WARN_ON(*length & ~(0x1f));
	WARN_ON(val > 0xffff);
	return val;
}

static unsigned int compute_ratio(unsigned int ratio, unsigned int length)
{
	unsigned int val;
	if (ratio > PWM_WIDTH_MAX)
		return -EINVAL;
	/* XXX overflow happen if length > 16 */

	val = ((ratio << length) + PWM_WIDTH_MAX/2) / PWM_WIDTH_MAX;
	WARN_ON(val > 0xffff);
	return val;
}

/**
 * @brief set the speed of a channel
 *
 * @param ntimer
 * @param speed
 */
static void px_pwm_set_speed(int ntimer, unsigned int speed)
{
	unsigned int length, freq;
	uint32_t ratio;
	freq = compute_speed(speed, &length);

	__raw_writel(freq, pwm_regbase + P6_PWM_SPEED00 + ntimer*4);
	ratio = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	BFSET(ratio, 16, 5, length);
	__raw_writel(ratio, pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
}

/*
 * @brief set the speed of a channel keeping 8bits precision on WIDTH
 *
 * @param ntimer
 * @param speed
 */
static void px_pwm_set_speed_8b_ratio_mode(int ntimer, unsigned int speed)
{
	unsigned int pclk = clk_get_rate(pwm_clk);
	unsigned int length, freq;
	uint32_t ratio;
	/* this makes us have 8 bits precision on WIDTH*/
	length = 8;

	if ( !speed )
		speed = 1;
	/* 512 is 2^(8+1) ; if we want to keep the 8 bits precision on WIDTH,
	 *  we cannot have a frequency to high, the max will be 304687Hz on P6
	 *  and 270833Hz on P6i
	 */
	if ( (pclk / speed) < 512 )
		freq = 0;
	else
		freq = ( (pclk / speed) >> (length + 1) ) - 1;
	__raw_writel(freq, pwm_regbase + P6_PWM_SPEED00 + ntimer*4);

	/* Get current value of RATIO register */
	ratio = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4);

	/* write length in bit 16 to 20 of RATIO register, keep old value for
	 * the other bits.*/
	BFSET(ratio, 16, 5, length);
	__raw_writel(ratio, pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
}


/**
 * @brief get the speed of a channel
 *
 * @param ntimer
 *
 * @return
 */
static unsigned int px_pwm_get_speed(int ntimer)
{
	unsigned int freq;
	uint32_t pclk = clk_get_rate(pwm_clk);
	uint32_t val = __raw_readl(pwm_regbase + P6_PWM_SPEED00 + ntimer*4);
	uint32_t length = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4) >> 16;
	freq = pclk /((val + 1) * 2);
	freq >>= length;
	if (freq == 0)
		freq = 1;
	return freq;
}

/**
 * @brief set the ratio of a channel
 *
 * @param ntimer
 * @param ratio
 */
static void px_pwm_set_ratio(int ntimer, unsigned int ratio)
{
	uint32_t ratio_val = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	uint32_t length = ratio_val >> 16;
	unsigned int ra;
	if (length == 0) {
		/* TODO may be add 0/1 mode */
		/* clock mode */
		px_pwm_set_mode(ntimer, true);
		return;
	}
	px_pwm_set_mode(ntimer, false);

	ra = compute_ratio(ratio, length);

	BFSET(ratio_val, 0, 16, ra);

	__raw_writel(ratio_val, pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
}

/**
 * @brief set the ratio of a channel
 * This function should be used only if PWM freq was set with PWM_SET_FREQ_8BITS_RATIO_MODE
 * (px_pwm_set_ratio can still be used even if PWM freq was set with PWM_SET_FREQ_8BITS_RATIO_MODE)
 * @param ntimer
 * @param ratio
 */
static int px_pwm_set_ratio_8b_ratio_mode(int ntimer, unsigned int ratio)
{
	uint32_t ratio_val = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	uint32_t length = ratio_val >> 16;
	unsigned int ra;

	if (length != 8) {
		/* If we get here, it is not used correctly */
		printk(KERN_DEBUG "px_pwm_set_ratio_8b_ratio_mode :"
				"(length != 8) => ERROR!!!\n");
		return -EINVAL;
	}

	px_pwm_set_mode(ntimer, false);
	ra = ratio;
	BFSET(ratio_val, 0, 16, ra);
	__raw_writel(ratio_val, pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	return 0;
}

/**
 * @brief get the ratio of a channel
 *
 * @param ntimer
 *
 * @return
 */
static unsigned int px_pwm_get_ratio(int ntimer)
{
	uint32_t ratio_val = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	uint32_t length = ratio_val >> 16;
	uint32_t ratio = ratio_val & 0xffff;

	if (length == 0)
		return PWM_WIDTH_MAX/2; /* clock mode */
	else
		return (ratio * PWM_WIDTH_MAX) >> length;
}

/**
 * @brief get the ratio of a channel
 * This function should be used only if PWM freq was set with PWM_SET_FREQ_8BITS_RATIO_MODE
 * (px_pwm_get_ratio can still be used even if PWM freq was set with PWM_SET_FREQ_8BITS_RATIO_MODE)
 * Here, the ratio returned depend on the "length", if in 8BITS mode, lenght is 8 and ratio goes up to 256 (100%)
 *
 * @param ntimer
 *
 * @return
 */
static unsigned int px_pwm_get_ratio_8b_ratio_mode(int ntimer)
{
	uint32_t ratio_val = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	uint32_t length = ratio_val >> 16;
	uint32_t ratio = ratio_val & 0xffff;

	if (length == 0)
		return -1; /* clock mode */
	else
		return ratio;
}

/**
 * reserve a pwm. Note that it is to the caller to make sure, there is no
 * concurent access
 *
 * @param timer : timer number
 * @return 0 or an error code
 */
static int px_pwm_request(unsigned int timer)
{
	if (timer >= PWM_NB_TIMER)
		return -EINVAL;

	if (px_pwm_is_active(timer))
		return -EBUSY;

	px_pwm_set_active(timer, true);

	return 0;
}

/**
 * release a pwm. Note that it is to the caller to make sure, there is no
 * concurent access
 *
 * @param timer : timer number
 * @return 0 or an error code
 */
static int px_pwm_release(unsigned int timer)
{
	if (timer >= PWM_NB_TIMER)
		return -EINVAL;

	if(delos_pwms_enabled == 0)
	{
		if (!px_pwm_is_active(timer))
			return -EBUSY;
		px_pwm_set_active(timer, false);
	}
	else
	{
		if ( !px_pwm_is_active(0) )
		{
			return -EBUSY;
		}
		if ( !px_pwm_is_active(1) )
		{
			return -EBUSY;
		}
		if ( !px_pwm_is_active(2) )
		{
			return -EBUSY;
		}
		if ( !px_pwm_is_active(3) )
		{
			return -EBUSY;
		}

		px_pwm_set_active(0, false);
		px_pwm_set_active(1, false);
		px_pwm_set_active(2, false);
		px_pwm_set_active(3, false);
		delos_pwms_enabled = 0;
	}

	return 0;
}

/**
 * start a timer
 *
 * @param timer the timer to use
 *
 * @return error code
 */
static int px_pwm_start(unsigned int timer)
{
	unsigned long flags;

	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	local_irq_save(flags);

	px_pwm_set_start(timer, 1);

	local_irq_restore(flags);

	return 0;
}

/** stop a timer
 *
 * @param timer
 * @return error code
 */
static int px_pwm_stop(unsigned int timer)
{
	unsigned long flags;

	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	local_irq_save(flags);

	px_pwm_set_start(timer, 0);

	local_irq_restore(flags);

	return 0;
}

/** configure a timer freq
 *
 * if the timer if not stopped, the value will taken in account a the next
 * reload (if autoreload is on)
 *
 * This function will keep the highest possible precision on FREQUENCY, even if it means that RATIO precision will be decreased.
 *
 * @param timer
 * @param freq in HZ
 *
 * @return error code
 */
static int px_pwm_set_freq(unsigned int timer, unsigned int freq)
{

	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	px_pwm_set_speed(timer, freq);

	return 0;
}

/** configure a timer freq in a way we still have precision on WIDTH
 *
 * if the timer if not stopped, the value will taken in account a the next
 * reload (if autoreload is on)
 *
 * This function will keep 8 bits precision on ratio, even if it means that FREQUENCY precision might be decreased (for high frequency, precision is really bad in this mode).
 *
 * @param timer
 * @param freq in HZ
 *
 * @return error code
 */
static int px_pwm_set_freq_8b_ratio_mode(unsigned int timer, unsigned int freq)
{

	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	px_pwm_set_speed_8b_ratio_mode(timer, freq);

	return 0;
}

static int px_pwm_get_freq(unsigned int timer, unsigned int *freq)
{
	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	*freq = px_pwm_get_speed(timer);
	return 0;
}

/* ratio = 0 ... 100 00 */
/**
 * configure the duty cycle
 *
 * @param timer
 * @param ratio (percentage * 100 of the dc)
 *
 * @return error code
 */
static int px_pwm_set_width(unsigned int timer, unsigned int ratio)
{
	if (ratio > PWM_WIDTH_MAX)
		return -EINVAL;

	px_pwm_set_ratio(timer, ratio);

	return 0;
}

/* ratio = 0 ... 256 */
/**
 * configure the duty cycle
 *
 * This function should be used only if PWM freq was set with PWM_SET_FREQ_8BITS_RATIO_MODE
 * (px_pwm_set_width can still be used even if PWM freq was set with PWM_SET_FREQ_8BITS_RATIO_MODE)
 *
 * @param timer
 * @param ratio (0 -> 256 => 256 is 100%)
 *
 * @return error code
 */
static int px_pwm_set_width_8b_ratio_mode(unsigned int timer, unsigned int ratio)
{
	if (ratio > PWM_WIDTH_8BITS_RATIO_MODE_MAX)
	{
		printk(KERN_DEBUG "px_pwm_set_width_8b_ratio_mode : (ratio > PWM_WIDTH_8BITS_RATIO_MODE_MAX) => ERROR!!!\n");
		return -EINVAL;
	}

	return px_pwm_set_ratio_8b_ratio_mode(timer, ratio);

}

static int px_pwm_get_width(unsigned int timer, unsigned int *ratio)
{
	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	*ratio = px_pwm_get_ratio(timer);

	return 0;
}


static int px_pwm_get_width_8b_ratio_mode(unsigned int timer, unsigned int *ratio)
{
	if (timer > PWM_NB_TIMER)
		return -EINVAL;

	*ratio = px_pwm_get_ratio_8b_ratio_mode(timer);

	return 0;
}

/**
 * @brief Reserve pwms and set pwm registers in order to make sure that the four pwms are synchronized
 * @return 0 or an error code
 */
static int delos_pwm_request(unsigned int timer)
{
	unsigned int gpio_status;

	if (timer >= PWM_NB_TIMER)
		return -EINVAL;

	if ( px_pwm_is_active(0) )
	{
		return -EBUSY;
	}
	if ( px_pwm_is_active(1) )
	{
		return -EBUSY;
	}
	if ( px_pwm_is_active(2) )
	{
		return -EBUSY;
	}
	if ( px_pwm_is_active(3) )
	{
		return -EBUSY;
	}

	px_pwm_set_active(0, true);
	px_pwm_set_active(1, true);
	px_pwm_set_active(2, true);
	px_pwm_set_active(3, true);


	delos_pwms_enabled = 1;

	delos_motor_disable(&gpio_status);

	__raw_writel(0, pwm_regbase + P6_PWM_RATIO00);
	__raw_writel(0, pwm_regbase + P6_PWM_RATIO01);
	__raw_writel(0, pwm_regbase + P6_PWM_RATIO02);
	__raw_writel(0, pwm_regbase + P6_PWM_RATIO03);

	__raw_writel(0, pwm_regbase + P6_PWM_SPEED00);
	__raw_writel(0, pwm_regbase + P6_PWM_SPEED01);
	__raw_writel(0, pwm_regbase + P6_PWM_SPEED02);
	__raw_writel(0, pwm_regbase + P6_PWM_SPEED03);

	__raw_writel(0xF, pwm_regbase + P6_PWM_CTL);

	msleep(1000);

	__raw_writel(0x0, pwm_regbase + P6_PWM_CTL);

	delos_motor_enable(gpio_status);

	return 0;
}


/**
 * @brief set the ratio of all channels on Delos
 */
static int delos_set_raw_ratios(pwm_delos_quadruplet* ratios)
{
	unsigned int ntimer;

	/*
	 * We assume the last bits of register ratio are the same for the four pwms
	 */
	delos_ratio = (ratios->val[0]>>16)&0xf;


	for (ntimer=0;ntimer<4;ntimer++){
		if(delos_verbose)printk("Delos PWM ratio %d = 0x%x\n",ntimer,ratios->val[ntimer]);
		__raw_writel(ratios->val[ntimer], pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	}
	return 0;
}

/**
 * @brief set the frequencies of all channels on Delos
 */
static int delos_set_raw_speeds(pwm_delos_quadruplet* speeds)
{
	unsigned int ntimer;

	/*
	 * We assume values into PWM_SPEEDxx are the same for the four pwms
	 */
	delos_speed = speeds->val[0] & 0x0000FFFF;

	for (ntimer=0;ntimer<4;ntimer++){
		if(delos_verbose)printk("Delos PWM speed %d = 0x%x\n",ntimer,speeds->val[ntimer]);
		__raw_writel(speeds->val[ntimer], pwm_regbase + P6_PWM_SPEED00 + ntimer*4);
	}
	return 0;
}


/*
 * In fact, PCLK is not always set to 156MHz, it may be set to 130Mhz or 138Mhz.
 * For the intended use (calculation of an approximated delay beetween two pwm launching, it doesn't matter.
 */
#define PCLK_MHZ   156

/**
 * @brief Activate PWMs on Delos
 */
static int delos_set_raw_ctrl_reg(unsigned int reg)
{
	unsigned long flags;
	unsigned int ctrl_reg;
	unsigned int gpio_status;

	/*
	 * Computes delay for desynchronize the four pwms
	 * Delay between each pwm launching is T/4 with T = (PWM_speed + 1)*2^(delos_ratio[20:16] + 1)/fpclk
	 * stride unit is ns
	 */

	unsigned int stride = ((delos_speed + 1)*(1<<(delos_ratio + 1))/PCLK_MHZ * 1000)>>2;

	if(delos_verbose)printk("Delos PWM control -> 0x%x\n",reg);

	delos_motor_disable(&gpio_status);

	local_irq_save(flags);

	/* If one of the PWM is requested to be active, we activate the global PWM clock too */
	if (reg&0xf) { clk_enable(pwm_clk); } else { clk_disable(pwm_clk); }

	ctrl_reg = __raw_readl( pwm_regbase + P6_PWM_CTL);
	ctrl_reg |= reg & 0x1;
	__raw_writel(ctrl_reg, pwm_regbase + P6_PWM_CTL);
	wmb();
	ndelay(stride);
	ctrl_reg |= reg & 0x2;
	__raw_writel(ctrl_reg, pwm_regbase + P6_PWM_CTL);
	wmb();
	ndelay(stride);
	ctrl_reg |= reg & 0x4;
	__raw_writel(ctrl_reg, pwm_regbase + P6_PWM_CTL);
	wmb();
	ndelay(stride);
	ctrl_reg |= reg & 0x8;
	__raw_writel(ctrl_reg, pwm_regbase + P6_PWM_CTL);
	wmb();

	delos_motor_enable(gpio_status);

	local_irq_restore(flags);
	return 0;
}

static int pw_px_pwm_ioctl(unsigned int pwm, unsigned int cmd, unsigned long arg)
{
	int ret;
	switch (cmd) {
		case PWM_DELOS_REQUEST:
			ret = delos_pwm_request(0xF);
			break;
		case PWM_DELOS_SET_RATIOS:
			{
				pwm_delos_quadruplet pwm_ratios;
				if (copy_from_user(&pwm_ratios, (int __user *) arg
							,sizeof(pwm_ratios))) {
					ret = -EFAULT;
					break;
				}
				ret = delos_set_raw_ratios(&pwm_ratios);
				break;
			}
		case PWM_DELOS_SET_SPEEDS:
			{
				pwm_delos_quadruplet pwm_speeds;
				if (copy_from_user(&pwm_speeds, (int __user *) arg,sizeof(pwm_speeds))) {
					ret = -EFAULT;
					break;
				}
				ret = delos_set_raw_speeds(&pwm_speeds);
				break;
			}

		case PWM_DELOS_SET_CTRL:
			{
				unsigned int ctrl;
				if (get_user(ctrl, (int __user *) arg))
				{
					ret = -EFAULT;
					break;
				}
				ret = delos_set_raw_ctrl_reg(ctrl);
				break;
			}

			// 8BITS_RATIO_MODE :
		case PWM_SET_FREQ_8BITS_RATIO_MODE:
			{	// Set freq so that we still have 8 bits precision on WIDTH ;
				// WIDTH can be set with PWM_SET_WIDTH (0->10000) or PWM_SET_WIDTH_8BITS_RATIO_MODE (0->256)
				unsigned int pwm_freq;
				if (get_user(pwm_freq, (int __user *) arg)) {
					ret = -EFAULT;
					break;
				}
				ret = px_pwm_set_freq_8b_ratio_mode(pwm, pwm_freq);
				break;
			}
		case PWM_SET_WIDTH_8BITS_RATIO_MODE:
			{	// Set width between 0 and 256. XXX works only if freq is set with PWM_SET_FREQ_8BITS_RATIO_MODE
				unsigned int pwm_width;
				if (get_user(pwm_width, (int __user *) arg)) {
					ret = -EFAULT;
					break;
				}
				ret = px_pwm_set_width_8b_ratio_mode(pwm, pwm_width);
				break;
			}
		case PWM_GET_WIDTH_8BITS_RATIO_MODE:
			{	// Directly returns the content of the RATIO register (bit 0 to 15)
				unsigned int pwm_width;
				ret = px_pwm_get_width_8b_ratio_mode(pwm, &pwm_width);
				if (put_user(pwm_width, (int __user *) arg)) {
					ret = -EFAULT;
				}
				break;
			}

		default:
			ret = -ENOTTY;
	}
	return ret;
}

struct pwm_ops px_pwm_ops = {
	.pwm_max = PWM_NB_TIMER-1,
	.pwm_start = px_pwm_start,
	.pwm_stop = px_pwm_stop,
	.pwm_request = px_pwm_request,
	.pwm_release = px_pwm_release,
	.pwm_set_width = px_pwm_set_width,
	.pwm_set_freq = px_pwm_set_freq,
	.pwm_get_width = px_pwm_get_width,
	.pwm_get_freq = px_pwm_get_freq,
	.pwm_ioctl = pw_px_pwm_ioctl,
	.owner = THIS_MODULE,
};

static int __devinit px_pwm_init(void)
{
	/* XXX this should use platform stuff... */
	pwm_regbase = ioremap(PARROT6_PWM, 256);
	if (pwm_regbase == NULL) {
		printk( KERN_ERR "ioremap failed\n");
		return -ENOMEM;
	}
	pwm_clk = clk_get(NULL, "pwm");

	if (IS_ERR(pwm_clk)) {
		printk(KERN_ERR "PWM clock not found");
		return -EBUSY;
	}

	pwm_active = 0;

	return register_pwm(&px_pwm_ops);
}

static void __exit px_pwm_exit(void)
{
	clk_disable(pwm_clk);
	clk_put(pwm_clk);

	unregister_pwm(&px_pwm_ops);
}

module_init(px_pwm_init);
module_exit(px_pwm_exit);

MODULE_AUTHOR("PARROT SA");
MODULE_DESCRIPTION("p6 pwm driver");
MODULE_LICENSE("GPL");



