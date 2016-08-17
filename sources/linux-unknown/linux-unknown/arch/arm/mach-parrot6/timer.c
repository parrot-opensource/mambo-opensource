/*
 *  linux/arch/arm/mach-parrot6/timer.c
 *
 *  Copyright (C) 2008 Parrot SA
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

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/system.h>
#include <asm/leds.h>
#include <asm/mach-types.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/platform.h>
#include <mach/map.h>

#define TLOAD_MAX      (PARROT6_PCLK_HS/16)

/* for clocksource and clock event, tune scale and shift */
/* only for p5+ */
#if defined(CONFIG_GENERIC_CLOCKEVENTS)
static unsigned long timer_reload;
static unsigned long timer_scale;

#include <linux/clockchips.h>
#include <linux/cnt32_to_63.h>

/* the counter will wrap every year */
#define TCR2NS_SCALE_FACTOR 8

static unsigned long tcr2ns_scale;

/*static */cycle_t parrot6_get_cycles(void)
{
	return ~readl(PARROT6_VA_SYS+_P6_SYS_TIM2CNT);
}
EXPORT_SYMBOL(parrot6_get_cycles);

static struct clocksource clocksource_parrot6 = {
	.name 		= "timer2",
 	.rating		= 200,
 	.read		= parrot6_get_cycles,
	.mask		= CLOCKSOURCE_MASK(32),
 	.shift 		= 26, /* diff is  +54 ns per 30s */
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init parrot6_clocksource_init(void)
{
/* SCALE 0 mean 130 Mhz and 20 s wrap */
#define SCALE 0
	unsigned long ahb_clk;

	/* setup timer2 as free-running clocksource */
	writel(0, PARROT6_VA_SYS+_P6_SYS_TIM2CTL);
	writel(0xffffffff, PARROT6_VA_SYS+_P6_SYS_TIM2LD);
	/* no interrupt free running */
	writel(SCALE | P6_SYS_TIMXCTL_ENABLE,
	       PARROT6_VA_SYS+_P6_SYS_TIM2CTL);

	ahb_clk = clk_get_rate(NULL);

 	clocksource_parrot6.mult =
 		clocksource_hz2mult(ahb_clk/(1 << (4*SCALE)), clocksource_parrot6.shift);
 	clocksource_register(&clocksource_parrot6);

	tcr2ns_scale = clocksource_hz2mult(ahb_clk/(1 << (4*SCALE)), TCR2NS_SCALE_FACTOR);
	/*
	 * We want an even value to automatically clear the top bit
	 * returned by cnt32_to_63() without an additional run time
	 * instruction. So if the LSB is 1 then round it up.
	 */
	if (tcr2ns_scale&1)
		tcr2ns_scale++;

 	return 0;
}

/*
 * Return ns . Use clock source
 *
 */
unsigned long long sched_clock(void)
{
	unsigned long long v = cnt32_to_63(parrot6_get_cycles());
	return (v * tcr2ns_scale) >> TCR2NS_SCALE_FACTOR;
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *clk)
{
	unsigned long ctrl;

	switch(mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel(timer_reload, PARROT6_VA_SYS+_P6_SYS_TIM3LD);

		ctrl = timer_scale | P6_SYS_TIMXCTL_PERIODIC | P6_SYS_TIMXCTL_ENABLE;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		ctrl = timer_scale;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		ctrl = 0;
	}

	writel(ctrl, PARROT6_VA_SYS+_P6_SYS_TIM3CTL);
}

static int timer_set_next_event(unsigned long evt,
				struct clock_event_device *unused)
{
	unsigned long ctrl = readl(PARROT6_VA_SYS+_P6_SYS_TIM3CTL);

	writel(evt, PARROT6_VA_SYS+_P6_SYS_TIM3LD);
	writel(ctrl | P6_SYS_TIMXCTL_ENABLE, PARROT6_VA_SYS+_P6_SYS_TIM3CTL);

	return 0;
}

static struct clock_event_device timer3_clockevent =	 {
	.name		= "timer3",
	.shift		= 32,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= timer_set_mode,
	.set_next_event	= timer_set_next_event,
};

/*
 * IRQ handler for the timer
 */
static irqreturn_t parrot6_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &timer3_clockevent;

	/* emulate oneshot mode, we should keep configuration as next event is
	 * used without timer reprogramming.
	 * We are in free running mode, so disabling the clock should be useless,
	 * if we assume the clock reprogramming is fast enough (less than
	 * 27s).
	 */
#if 0
	if (evt->mode == CLOCK_EVT_MODE_ONESHOT) {
		unsigned long ctrl = readl(PARROT6_VA_SYS+_P6_SYS_TIM3CTL);
		writel(ctrl & ~(P6_SYS_TIMXCTL_ENABLE),
				PARROT6_VA_SYS+_P6_SYS_TIM3CTL);
	}
#endif
	/* clear timer flag */
	writel(P6_SYS_ITACK_TIM3, PARROT6_VA_SYS+_P6_SYS_ITACK);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction parrot6_timer_irq = {
	.name		= "Parrot6 Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= parrot6_timer_interrupt,
};


static void __init parrot6_clockevent_init(void)
{
	u32 iten;
	int i, scale;
	unsigned long tload, ahb_clk, divider, tdivider;

	/* setup timer 3 */
	writel(0, PARROT6_VA_SYS+_P6_SYS_TIM3CTL);

	/* AHB bus clock */
	clk_enable(NULL);
	ahb_clk = clk_get_rate(NULL);

	/* choose proper prescaling (1, 16 or 256), starting from highest
	   resolution */
	tdivider = 0;
	scale = -1;

	for (i = 0; i < 4; i++) {

		/* divider is 2^(4*scale) */
		divider = 1 << (4*i);
		tload = ahb_clk/divider/HZ - 1;

		if (tload <= 0xffffffff) {
			tdivider = divider;
			scale = i;
			break;
		}
	}

	printk("timer load=0x%04lx, AHB bus %lu MHz, prescaling=%lu\n",
	       tload, ahb_clk/1000000, tdivider);

	/* check if we have a valid configuration */
	if (tdivider == 0) {
		panic("timer_init: bad HZ, cannot configure timer !");
		return;
	}
	/* ahb = 156Mhz
	 * scale = 0 [6ns, 27s]
	 * scale = 1 [96ns, 7.2Min]
	 * ahb = 130Mhz
	 * scale = 0 [7ns, 33s]
	 */
	timer_reload = tload;
	timer_scale = scale;

	/* enable timer interrupt */
	iten = __raw_readl(PARROT6_VA_SYS+_P6_SYS_ITEN)|P6_SYS_ITEN_TIM3;
	__raw_writel(iten, PARROT6_VA_SYS+_P6_SYS_ITEN);
	setup_irq(IRQ_P6_TIMER3, &parrot6_timer_irq);

	timer3_clockevent.mult =
		div_sc(ahb_clk/tdivider, NSEC_PER_SEC, timer3_clockevent.shift);
	timer3_clockevent.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &timer3_clockevent);
	timer3_clockevent.min_delta_ns =
		clockevent_delta2ns(0xf, &timer3_clockevent);

	timer3_clockevent.cpumask = cpumask_of(0);
	clockevents_register_device(&timer3_clockevent);
}

/**
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init parrot6_timer_init(void)
{
	parrot6_clockevent_init();
	parrot6_clocksource_init();
}

struct sys_timer p6_timer = {
	.init		= parrot6_timer_init,
};

#else /* old time code */

static unsigned long timer_16usec_ticks;

/*
 * Returns number of us since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
static unsigned long p6_gettimeoffset(void)
{
	u32 val, load, status, fix = 0;

	val = __raw_readl(PARROT6_VA_SYS+_P6_SYS_TIM3CNT);
	status = __raw_readl(PARROT6_VA_SYS+_P6_SYS_STATUS);
	load = __raw_readl(PARROT6_VA_SYS+_P6_SYS_TIM3LD);

	/* fix value when an interrupt is pending */
	if (status & P6_SYS_STATUS_IT_TIM3) {
		/* re-read timer value to resolve wrapping issues */
		val = __raw_readl(PARROT6_VA_SYS+_P6_SYS_TIM3CNT);
		if (val) {
			fix = load+1;
		}
	}
	/* note: 'val' and 'tload' values are less than TLOAD_MAX */
	val = fix+load-val;

	/* convert ticks to micro-seconds */
	return (16*val)/timer_16usec_ticks;
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t p6_timer_interrupt(int irq, void *dev_id)
{
	/* acknowledge interrupt */
	__raw_writel(P6_SYS_ITACK_TIM3, PARROT6_VA_SYS+_P6_SYS_ITACK);

	timer_tick();

	return IRQ_HANDLED;
}

static struct irqaction p6_timer_irq = {
	.name		= "Parrot6 Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= p6_timer_interrupt,
};

/**
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init p6_timer_init(void)
{
	u32 iten;
	unsigned long tload, tclk;

	/* stop timer 3 */
	__raw_writel(0, PARROT6_VA_SYS+_P6_SYS_TIM3CTL);

	/* timer clock is AHB bus clock with prescaling=16 */
	clk_enable(NULL);
	tclk = clk_get_rate(NULL)/16;
	tload = tclk/HZ - 1;

	printk("timer load=%lu, clock=%lu kHz\n", tload, tclk/1000);

	/* check if we have a valid configuration */
	if ((tload == 0) || (tload >= TLOAD_MAX)) {
		panic("timer_init: bad HZ, cannot configure timer !");
		return;
	}
	__raw_writel(tload, PARROT6_VA_SYS+_P6_SYS_TIM3LD);

	/* compute how many ticks we have in 16 useconds */
	timer_16usec_ticks = (16*tclk)/1000000;

	/* enable timer */
	__raw_writel(P6_SYS_TIMXCTL_PRESCALE16|
		     P6_SYS_TIMXCTL_PERIODIC|
		     P6_SYS_TIMXCTL_ENABLE,
		     PARROT6_VA_SYS+_P6_SYS_TIM3CTL);

	/* enable timer interrupt */
	iten = __raw_readl(PARROT6_VA_SYS+_P6_SYS_ITEN)|P6_SYS_ITEN_TIM3;
	__raw_writel(iten, PARROT6_VA_SYS+_P6_SYS_ITEN);

	setup_irq(IRQ_P6_TIMER3, &p6_timer_irq);
}

struct sys_timer p6_timer = {
	.init		= p6_timer_init,
	.offset		= p6_gettimeoffset,
};
#endif
