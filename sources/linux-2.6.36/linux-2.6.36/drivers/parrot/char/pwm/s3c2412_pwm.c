/**
********************************************************************************
* @file s3c2412_pwm.c
* @brief kernel driver for s3c2412 pwm
*
* Copyright (C) 2007 Parrot S.A.
*
* @author     Matthieu CASTET <matthieu.castet@parrot.com>
* @date       2007-06-15
********************************************************************************
*/


#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <linux/err.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/arch/regs-timer.h>
#include <asm/arch/regs-gpio.h>

#include "pwm_ops.h"
#include "pwm_ioctl.h"



/* timer 4 is for kernel */
#define S3C2410_TCON_OFFSET(timer) ((timer==0)?0:(4+timer*4))

#define dprintk(...)

/* public header */
enum {
    PWM_INVERT_OFF,
    PWM_INVERT_ON
};

enum {
    PWM_AUTO_RELOAD_OFF,
    PWM_AUTO_RELOAD_ON
};

enum {
    PWM_TOUT_OFF,
    PWM_TOUT_ON
};

enum {
    PWM_TIMER0,
    PWM_TIMER1,
    PWM_TIMER2,
    PWM_TIMER3,
    PWM_TIMER_MAX = PWM_TIMER3,
};


struct pwm {
    int used;
    int freq;
    int width;
};

static struct pwm pwm[PWM_TIMER_MAX];

/**
 * reserve a pwm. Note that it is to the caller to make sure, there is no
 * concurent access
 *
 * @param timer : timer number
 * @return 0 or an error code
 */
static int s3c2412pwm_request(unsigned int timer)
{
    if (timer > PWM_TIMER_MAX)
        return -EINVAL;

    if (pwm[timer].used)
        return -EBUSY;
    pwm[timer].used = 1;
    return 0;
}

/**
 * release a pwm. Note that it is to the caller to make sure, there is no
 * concurent access
 *
 * @param timer : timer number
 * @return 0 or an error code
 */
static int s3c2412pwm_release(unsigned int timer)
{
    if (timer > PWM_TIMER_MAX)
        return -EINVAL;

    if (!pwm[timer].used)
        return -EINVAL;
    pwm[timer].used = 0;
    return 0;
}

/**
 * get the global prescaler factor for a timer
 * @parm timer
 *
 * @return factor
 */
static int get_prescaler(unsigned int timer)
{
    unsigned long tcfg0;

    tcfg0 = __raw_readl(S3C2410_TCFG0);

    /* read the prescaler */
    if (timer < 2) {
        tcfg0 &= S3C2410_TCFG_PRESCALER0_MASK;
        return tcfg0;
    }
    else {
        tcfg0 &= S3C2410_TCFG_PRESCALER1_MASK;
        return tcfg0 >> S3C2410_TCFG_PRESCALER1_SHIFT;
    }
}

/**
 * start a timer
 *
 * @param timer the timer to use
 *
 * @return error code
 */
static int s3c2412pwm_start(unsigned int timer)
{
    unsigned long tcon;
    unsigned long flags;
    /* invert do we want to invert TOUT output
     * auto_reload
     * tout do we want to activate TOUT gpio
     */
    const int invert = 0, auto_reload = 1, tout = 1;

    if (timer > PWM_TIMER_MAX)
        return -EINVAL;

    /* if the TOUT is already active there will be some discontinuity :
     * the manual update will reset the engine, and make
     * TOUT = !invert
     *
     * We can do nothing and assume we don't care of the signal before the
     * start.
     */

    local_irq_save(flags);

    tcon = __raw_readl(S3C2410_TCON);

    /* 2 configure timer */
    tcon &= ~((S3C2410_TCON_T0START|S3C2410_TCON_T0INVERT|S3C2410_TCON_T0RELOAD)<<S3C2410_TCON_OFFSET(timer));

    if (invert)
        tcon |= S3C2410_TCON_T0INVERT<<S3C2410_TCON_OFFSET(timer);
    if (auto_reload)
        tcon |= S3C2410_TCON_T0RELOAD<<S3C2410_TCON_OFFSET(timer);

    tcon |= S3C2410_TCON_T0MANUALUPD<<S3C2410_TCON_OFFSET(timer);
    dprintk("tcon init : %08lx\n", tcon);
    __raw_writel(tcon, S3C2410_TCON);

    /* 3 start the timer */
    tcon &= ~(S3C2410_TCON_T0MANUALUPD<<S3C2410_TCON_OFFSET(timer));
    tcon |= S3C2410_TCON_T0START<<S3C2410_TCON_OFFSET(timer);
    dprintk("tcon start : %08lx\n", tcon);
    __raw_writel(tcon, S3C2410_TCON);

    local_irq_restore(flags);

    if (tout)
        s3c2410_gpio_cfgpin(S3C2410_GPB0+timer, S3C2410_GPIO_SFN2);

    return 0;
}

/** stop a timer
 *
 * @param timer
 * @return error code
 */
static int s3c2412pwm_stop(unsigned int timer)
{
    unsigned long tcon;
    unsigned long flags;

    if (timer > PWM_TIMER_MAX)
        return -EINVAL;
    local_irq_save(flags);
    tcon = __raw_readl(S3C2410_TCON);
    tcon &= ~(S3C2410_TCON_T0START<<S3C2410_TCON_OFFSET(timer));
    __raw_writel(tcon, S3C2410_TCON);
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
static int s3c2412pwm_set_freq(unsigned int timer, unsigned int freq)
{
    unsigned long tcfg1;
    unsigned long pclk, timer_base;
    unsigned long flags;
    struct clk *clk;
    int div, cnt;

    if (timer > PWM_TIMER_MAX)
        return -EINVAL;

    clk = clk_get(NULL, "timers");
    if (IS_ERR(clk))
        panic("failed to get clock for system timer");

    /* it is enabled by int time.c */
    pclk = clk_get_rate(clk);
    timer_base = pclk / (get_prescaler(timer) + 1);
    dprintk("pclk %ld, timer_base %ld\n", pclk, timer_base);

    /* first diviser is 1 */
    timer_base>>=1;
    if (freq > timer_base) {
        printk("freq is too high\n");
        return -EINVAL;
    }

    /* find the best diviser */
    for (div = 0; div < 4; div++) {
        dprintk("min : %ld\n", timer_base/0xffff);
        /* this need to be > and not >= because of our rounding */
        if (freq > timer_base/0xffff)
            break;
        timer_base>>=1;
    }

    if (div == 4) {
        printk("freq is too low\n");
        return -EINVAL;
    }

    /* configure diviser */
    local_irq_save(flags);
    tcfg1 = __raw_readl(S3C2410_TCFG1);
    tcfg1 &= ~(S3C2410_TCFG1_MUX0_MASK<<(timer*4));
    tcfg1 |= div << (timer*4);
    __raw_writel(tcfg1, S3C2410_TCFG1);
    local_irq_restore(flags);

    cnt = (timer_base + freq/2 ) / freq;
    /* timers reload after counting zero, so reduce the count by 1 */
    cnt--;
    dprintk("set freq : %d tcfg1 : %ld, cnt : %d\n", freq, tcfg1, cnt);

    __raw_writel(cnt, S3C2410_TCNTB(timer));
    /* 100 % dc */
    __raw_writel(cnt-1, S3C2410_TCMPB(timer));
    pwm[timer].freq = timer_base / (cnt + 1);

    return 0;
}

static int s3c2412pwm_get_freq(unsigned int timer, unsigned int *freq)
{
    if (timer > PWM_TIMER_MAX)
        return -EINVAL;

    *freq = pwm[timer].freq;
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
static int s3c2412pwm_set_width(unsigned int timer, unsigned int ratio)
{
    unsigned long cnt, cmp;
    if (ratio > PWM_WIDTH_MAX)
        return -EINVAL;
    
    cnt = __raw_readl(S3C2410_TCNTB(timer));

    /* the TCMPB is not check at the start of the timer,
     * but only after the first tick. So we should remove a
     * tick if is the same than the counter
     */
    cmp = (cnt * ratio + PWM_WIDTH_MAX/2) / PWM_WIDTH_MAX;
    if (cnt <= cmp)
        cmp = cnt - 1;
    dprintk("set ratio : %d, cnt : %ld\n", ratio, cmp);
    __raw_writel(cmp, S3C2410_TCMPB(timer));
    pwm[timer].width = ratio;
    return 0;
}

static int s3c2412pwm_get_width(unsigned int timer, unsigned int *ratio)
{
    if (timer > PWM_TIMER_MAX)
        return -EINVAL;

    *ratio = pwm[timer].width;
    return 0;
}

struct pwm_ops s3c2412_pwm_ops = {
    .pwm_max = PWM_TIMER_MAX,
    .pwm_start = s3c2412pwm_start,
    .pwm_stop = s3c2412pwm_stop,
    .pwm_request = s3c2412pwm_request,
    .pwm_release = s3c2412pwm_release,
    .pwm_set_width = s3c2412pwm_set_width,
    .pwm_set_freq = s3c2412pwm_set_freq,
    .pwm_get_width = s3c2412pwm_get_width,
    .pwm_get_freq = s3c2412pwm_get_freq,
    .owner = THIS_MODULE,
};


static int __devinit s3c2412_pwm_init(void)
{
    unsigned long tcon, tcfg0, tcfg1;
    /* reset timer0-3 */
    tcon = __raw_readl(S3C2410_TCON);
    tcon &= (S3C2410_TCON_T4START|S3C2410_TCON_T4MANUALUPD|S3C2410_TCON_T4RELOAD);
    __raw_writel(tcon, S3C2410_TCON);

    tcfg1 = __raw_readl(S3C2410_TCFG1);
    tcfg1 &= S3C2410_TCFG1_MUX4_MASK;
    __raw_writel(tcfg1, S3C2410_TCFG1);

    /* set prescaler 0 to 0 */
    tcfg0 = __raw_readl(S3C2410_TCFG0);
    tcfg0 &= S3C2410_TCFG_PRESCALER1_MASK;
    __raw_writel(tcfg0, S3C2410_TCFG0);

    return register_pwm(&s3c2412_pwm_ops);
}

static void __exit s3c2412_pwm_exit(void)
{
    unsigned long tcon;
    /* stop timer0-3 */
    tcon = __raw_readl(S3C2410_TCON);
    tcon &= (S3C2410_TCON_T4START|S3C2410_TCON_T4MANUALUPD|S3C2410_TCON_T4RELOAD);
    __raw_writel(tcon, S3C2410_TCON);

    unregister_pwm(&s3c2412_pwm_ops);
}


module_init(s3c2412_pwm_init);
module_exit(s3c2412_pwm_exit);


MODULE_AUTHOR("PARROT SA by Matthieu CASTET <matthieu.castet@parrot.com>");
MODULE_DESCRIPTION("S3c2412 pwm driver");
MODULE_LICENSE("GPL");
