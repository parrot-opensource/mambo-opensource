/**
********************************************************************************
* @file p5p_pwm.c 
* @brief kernel driver for p5p pwm
*
* Copyright (C) 2008 Parrot S.A.
*
* @author     François Guillemé <francois.guilleme@parrot.com>
* @date       1-Oct-2008
********************************************************************************
*/
#error this is buggy

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <linux/err.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/arch/parrot.h>
#include <asm/arch/map-p5.h>
#include <asm/arch/sysc.h>

#include "pwm_ops.h"
#include "pwm_ioctl.h"

#define PWM_NB_TIMER 8

#define NOREG ~0

#define BFMASK(_start, _width) (((1 << (_width))-1) << (_start))

#define BFSET(_v, _start, _width, _val) \
	do { \
		_v &= ~(BFMASK(_start, _width)); \
		_v |= ((_val & ((1 << (_width))-1)) << _start); \
	} while (0); 

#define BFGET(_v,_start,_width) \
	(( _v  & BFMASK(_start, _width)) >> _start)

struct register_layout {
	uint32_t ctrl;
	uint32_t speed03[4];
	uint32_t ratio03;
	uint32_t speed47[4];
	uint32_t ratio47;
};

static struct register_layout __iomem *pwm_regbase;
static uint32_t pwm_active;		// we can handle up to 32 timers

static bool px_pwm_is_active(uint32_t n) { 
	return n<PWM_NB_TIMER ? BFGET(pwm_active, n, 1) != 0 : false; 
}

static inline void px_pwm_set_active(uint32_t n, bool active) { 
	static char *scode[]={ "PWM0", "PWM1", "PWM2", "PWM3", "PWM4a", "PWM5a", "PWM6a", "PWM7a" };
	static uint16_t code[]={ PWM0, PWM1, PWM2, PWM3, PWM4a, PWM5a, PWM6a, PWM7a };

	if (n < PWM_NB_TIMER) {
		struct clk *clk = clk_get(NULL, "pwm");
	
		if (unlikely(clk == NULL)) {
			printk(KERN_ERR "PWM clock not found");
			return;
		}

		// activate the clock if we start the first timer
		if (active && pwm_active == 0) {
			printk(KERN_DEBUG "start pwm clock\n");
			clk_enable(clk);
		}

		// set/clear the appropriate pin
		if (active) {
			printk(KERN_DEBUG "select PIN %s\n", scode[n]);
			parrot_select_pin(code[n]);
		}

		BFSET(pwm_active, n, 1, active);

		// stop the clock if we stop the last timer
		if (!active && pwm_active == 0) {
			printk(KERN_DEBUG "stop pwm clock\n");
			clk_disable(clk);
		}
	}
}

/** 
 * @brief get the register address corresponding to a pwm channel
 * 
 * @param ntimer number of the timer
 * 
 * @return address of timer or NOREG if index is invalid
 */
static inline uint32_t px_pwm_speed_reg(int ntimer) {
	struct register_layout *regs = pwm_regbase;

	if (ntimer >= 0 && ntimer < 4)
		return (uint32_t)(&regs->speed03[ntimer]);
	else if (ntimer >= 4 && ntimer < 8)
		return (uint32_t)(&regs->speed47[ntimer]);
	else
		return NOREG;
}

static inline uint32_t px_pwm_ratio_reg(int ntimer) {
	struct register_layout *regs = pwm_regbase;

	if (ntimer >= 0 && ntimer < 4)
		return (uint32_t)(&regs->ratio03);
	else if (ntimer >= 4 && ntimer < 8)
		return (uint32_t)(&regs->ratio47);
	else
		return NOREG;
}

static inline uint32_t px_pwm_ctrl_reg(void) {
	struct register_layout *regs = pwm_regbase;

	return (uint32_t)(&regs->ctrl);
}

/** 
 * @brief enable or disable a pwm channel
 * 
 * @param ntimer 
 * @param start 
 */
static void px_pwm_set_start(int ntimer, bool start) {
	uint32_t reg = px_pwm_ctrl_reg();
	
	if (reg != NOREG) {
		uint32_t val;
		uint8_t offset = (ntimer & 0x03) + (ntimer >> 2) * 8;

		val = __raw_readl(reg);

		BFSET(val, offset, 1, start);

		printk(KERN_DEBUG "(pwm_set_start) ctrl (0x%08X) = %08X\n", reg, val);
		
		__raw_writel(val, reg);
	}
}

/** 
 * @brief set the clock mode of a channel
 * 
 * @param ntimer 
 * @param clock mode 
 */
static void px_pwm_set_mode(int ntimer, bool mode) {
	uint32_t reg = px_pwm_ctrl_reg();

	if (reg != NOREG) {
		uint8_t offset = 4 + (ntimer & 0x03) + (ntimer >> 2) * 8;
		uint32_t val;

		printk(KERN_DEBUG "pwm set_mode %d %d\n", ntimer, mode);
		val = __raw_readl(reg);

		BFSET(val, offset, 1, (mode?1:0));

		printk(KERN_DEBUG "(pwm_set_mode) ctrl (0x%08X) = %08X\n", reg, val);

		__raw_writel(val, reg);
	}
}

/** 
 * @brief get the mode of a channel
 * 
 * @param ntimer 
 * 
 * @return 
 */
static bool px_pwm_get_mode(int ntimer) {
	uint32_t reg = px_pwm_ctrl_reg();
	bool mode = false;

	if (reg != NOREG) {
		uint8_t offset = 4 + (ntimer & 0x03) + (ntimer >> 2) * 8;
		uint32_t val;
		
		val = __raw_readl(reg);

		printk(KERN_DEBUG "(pwm_get_mode) ctrl (0x%08X) = %08X\n", reg, val);

		mode = BFGET(val, offset, 1) != 0;
	}
	return mode;
}

/** 
 * @brief set the speed of a channel
 * 
 * @param ntimer 
 * @param speed 
 */
static void px_pwm_set_speed(int ntimer, unsigned int speed) {
	uint32_t reg = px_pwm_speed_reg(ntimer);
	struct clk *clk = clk_get(NULL, "pwm");

	if (unlikely(clk == NULL)) {
		printk(KERN_ERR "PWM clock not found\n");
		return;
	}

	if (reg != NOREG) {
		bool mode = px_pwm_get_mode(ntimer);
		uint32_t pclk = clk_get_rate(clk);
		uint32_t val;
		
		if (!mode) 
			val = pclk / (4*256*speed) - 1;
		else 
			val = pclk / ( 4*speed ) - 1;

		printk(KERN_DEBUG "pwm set_speed %d %u => %d\n", ntimer, speed, val);

		printk(KERN_DEBUG "(pwm_set_speed) speed reg (0x%08X) = %08X\n", reg, val);

		if (speed > 0xffff) speed = 0xffff;

		__raw_writel(val, reg);
	}
}

/** 
 * @brief get the speed of a channel
 * 
 * @param ntimer 
 * 
 * @return 
 */
static uint32_t px_pwm_get_speed(int ntimer) {
	uint32_t reg = px_pwm_speed_reg(ntimer);
	uint32_t freq = ~0;
	struct clk *clk = clk_get(NULL, "pwm");

	if (unlikely(clk == NULL)) {
		printk(KERN_ERR "PWM clock not found\n");
		return freq;
	}
	if (reg != NOREG) {
		bool mode = px_pwm_get_mode(ntimer);
		uint32_t pclk = clk_get_rate(clk);
		uint32_t val = __raw_readl(reg);

		
		printk(KERN_DEBUG "(pwm_get_speed) speed reg (0x%08X) = %08X\n", reg, val);

		if (mode)
			freq = pclk /((val + 1) * 4);
		else
			freq = pclk /((val + 1) * 4 * 256);

	}
	return freq;
}

/** 
 * @brief set the ratio of a channel
 * 
 * @param ntimer 
 * @param ratio 
 */
static void px_pwm_set_ratio(int ntimer, uint32_t ratio) {
	uint32_t reg = px_pwm_ratio_reg(ntimer);
	uint8_t offset = (ntimer & 0x03 ) * 8;

	if (reg != NOREG) {
		uint32_t val = __raw_readl(reg);

		printk(KERN_DEBUG "pwm set_ratio %d %d/255\n", ntimer, ratio);

		BFSET(val, offset, 8, ratio);

		printk(KERN_DEBUG "(pwm_set_ratio) ratio reg (0x%08X) = %08X\n", reg, val);

		if (ratio) {
			px_pwm_set_mode(ntimer, false);
			__raw_writel(val, reg);
		}
		else
			px_pwm_set_mode(ntimer, true);

	}
}

/** 
 * @brief get the ratio of a channel
 * 
 * @param ntimer 
 * 
 * @return 
 */
static uint8_t px_pwm_get_ratio(int ntimer) {
	uint8_t val = ~0;
	uint32_t reg = px_pwm_ratio_reg(ntimer);
	uint8_t offset = (ntimer & 0x03 ) * 8;

	if (reg != NOREG)
		val = BFGET(__raw_readl(reg), offset, 8);

	printk(KERN_DEBUG "(pwm_get_ratio) ratio reg (0x%08X) = %08X\n", reg, val);

	return val;
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
    
	printk(KERN_DEBUG "pwm request %d\n", timer);

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

    if (!px_pwm_is_active(timer))
        return -EBUSY;
    
	printk(KERN_DEBUG "pwm release %d\n", timer);

	px_pwm_set_active(timer, false);
    
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

	printk(KERN_DEBUG "pwm start %d\n", timer);

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

	printk(KERN_DEBUG "pwm stop %d\n", timer);

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

	printk(KERN_DEBUG "pwm set_ratio %d %d.%02d %\n", timer, ratio/100, ratio%100);

    ratio = (ratio*256)/(100*100);
	
	if (ratio == 256) ratio = 255;
	if (ratio > 255) ratio = 0;

	px_pwm_set_ratio(timer, ratio);

    return 0;
}

static int px_pwm_get_width(unsigned int timer, unsigned int *ratio)
{
    if (timer > PWM_NB_TIMER)
        return -EINVAL;

    *ratio = (100*100*(px_pwm_get_ratio(timer)))/256;

    return 0;
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
    .owner = THIS_MODULE,
};

static int __devinit px_pwm_init(void)
{
	pwm_regbase = ioremap(PARROT5_PWM, SZ_4K);
	if (pwm_regbase == NULL) {
		printk( KERN_ERR "ioremap failed\n");
		return -ENOMEM;
	}

	pwm_active = 0;
    return register_pwm(&px_pwm_ops);
}

static void __exit px_pwm_exit(void)
{
	struct clk *clk = clk_get(NULL, "pwm");

	if (unlikely(clk == NULL)) {
		printk(KERN_ERR "PWM clock not found\n");
	}
	else
		clk_disable(clk);

    unregister_pwm(&px_pwm_ops);
}

module_init(px_pwm_init);
module_exit(px_pwm_exit);

MODULE_AUTHOR("PARROT SA");
MODULE_DESCRIPTION("p5p pwm driver");
MODULE_LICENSE("GPL");


