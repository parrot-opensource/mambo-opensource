/*
 *  linux/arch/arm/mach-parrot/timer.c
 *
 *  Copyright (C) 2007 Parrot SA
 *
 * @author     ivan.djelic@parrot.com
 * @date       2007-06-11
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

static unsigned long timer_reload;

/* for clocksource and clock event, tune scale and shift */
/* only for p5+ */
#if defined(CONFIG_GENERIC_TIME) && defined(CONFIG_GENERIC_CLOCKEVENTS)
static unsigned long timer_scale;

#include <mach/parrot.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/cnt32_to_63.h>

/* the counter will wrap every year */
#define TCR2NS_SCALE_FACTOR 8

static unsigned long tcr2ns_scale;

static cycle_t parrot5_get_cycles(void)
{
	return ~readl(PARROT5_VA_SYS+_P5_SYS_TIM2VAL);
}

static struct clocksource clocksource_parrot5 = {
	.name 		= "timer2",
 	.rating		= 200,
 	.read		= parrot5_get_cycles,
	.mask		= CLOCKSOURCE_MASK(32),
 	.shift 		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init parrot5_clocksource_init(void)
{
/* near 6 Mhz clock like parrot5 for 1/6 us resolution and 71 min wrap */
/* SCALE 0 mean 104 Mhz and 41 s wrap */
#define SCALE 0
	unsigned long ahb_clk;

	/* setup timer3 as free-running clocksource */
	writel(0, PARROT5_VA_SYS+_P5_SYS_TIM2CTL);
	writel(0xffffffff, PARROT5_VA_SYS+_P5_SYS_TIM2LD);
	/* no interrupt free running */
	writel(SCALE | P5_SYS_TIMXCTL_ENABLE,
	       PARROT5_VA_SYS+_P5_SYS_TIM2CTL);

	ahb_clk = clk_get_rate(NULL);

 	clocksource_parrot5.mult =
 		clocksource_khz2mult(ahb_clk/(1 << (4*SCALE))/1000, clocksource_parrot5.shift);
 	clocksource_register(&clocksource_parrot5);

	tcr2ns_scale = clocksource_khz2mult(ahb_clk/(1 << (4*SCALE))/1000, TCR2NS_SCALE_FACTOR);
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
	unsigned long long v = cnt32_to_63(parrot5_get_cycles());
	return (v * tcr2ns_scale) >> TCR2NS_SCALE_FACTOR;
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *clk)
{
	unsigned long ctrl;

	switch(mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel(timer_reload, PARROT5_VA_SYS+_P5_SYS_TIM3LD);

		ctrl = P5_SYS_TIMXCTL_PERIODIC;
		ctrl |= timer_scale | P5_SYS_TIMXCTL_ENABLE;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		ctrl = 0;
		ctrl |= timer_scale;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		ctrl = 0;
	}

	writel(ctrl, PARROT5_VA_SYS+_P5_SYS_TIM3CTL);
}

static int timer_set_next_event(unsigned long evt,
				struct clock_event_device *unused)
{
	unsigned long ctrl = readl(PARROT5_VA_SYS+_P5_SYS_TIM3CTL);

	writel(evt, PARROT5_VA_SYS+_P5_SYS_TIM3LD);
	writel(ctrl | P5_SYS_TIMXCTL_ENABLE, PARROT5_VA_SYS+_P5_SYS_TIM3CTL);

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
static irqreturn_t parrot5_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &timer3_clockevent;

	/* emulate oneshot mode, we should keep configuration as next event is
	 * used without timer reprogramming.
	 * We are in free running mode, so disabling the clock should be useless,
	 * if we assume the clock reprogramming is fast enough (less than
	 * 41s).
	 */
#if 0
	if (evt->mode == CLOCK_EVT_MODE_ONESHOT) {
		unsigned long ctrl = readl(PARROT5_VA_SYS+_P5_SYS_TIM3CTL);
		writel(ctrl & ~(P5_SYS_TIMXCTL_ENABLE),
				PARROT5_VA_SYS+_P5_SYS_TIM3CTL);
	}
#endif
	/* clear timer flag */
	writel(P5_SYS_FLAGS_TIM3, PARROT5_VA_SYS+_P5_SYS_FLAGS);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction parrot5_timer_irq = {
	.name		= "Parrot5 Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= parrot5_timer_interrupt,
};


static void __init parrot5_clockevent_init(void)
{
	int i, scale;
	unsigned long tload, ahb_clk, divider, tdivider;

	/* setup timer 3 */
	writel(0, PARROT5_VA_SYS+_P5_SYS_TIM3CTL);

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

	timer_reload = tload;
	timer_scale = scale;
	setup_irq(IRQ_P5_TIMER3, &parrot5_timer_irq);

	/* ahb = 100Mhz
	 * scale = 0 [10ns, 42s]
	 * scale = 0 [160ns, 71Min]
	 */
	timer3_clockevent.mult =
		div_sc(ahb_clk/tdivider, NSEC_PER_SEC, timer3_clockevent.shift);
	timer3_clockevent.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &timer3_clockevent);
	timer3_clockevent.min_delta_ns =
		clockevent_delta2ns(0xf, &timer3_clockevent);

	timer3_clockevent.cpumask = cpumask_of_cpu(0);
	clockevents_register_device(&timer3_clockevent);
}

/**
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init parrot5_timer_init(void)
{
	/* only work with p5+ */
	BUG_ON(parrot_chip_is_p5p() == 0);
	parrot5_clockevent_init();
	parrot5_clocksource_init();
}

struct sys_timer parrot5_timer = {
	.init		= parrot5_timer_init,
};

#else /* old time code */
static unsigned long timer_256usec_ticks;
/*
 * Returns number of us since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
static unsigned long parrot5_gettimeoffset(void)
{
	u32 val, flags, fix = 0;

	val = readl(PARROT5_VA_SYS+_P5_SYS_TIM3VAL);
	flags = readl(PARROT5_VA_SYS+_P5_SYS_FLAGS);

	/* fix value when an interrupt is pending */
	if (flags & (1 << 4)) {
		/* re-read timer value to resolve wrapping issues */
		val = readl(PARROT5_VA_SYS+_P5_SYS_TIM3VAL);
		if (val) {
			fix = timer_reload+1;
		}
	}
	val = fix+timer_reload-val;

	/* convert ticks to micro-seconds */
	return ((val << 8)/timer_256usec_ticks);
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t parrot5_timer_interrupt(int irq, void *dev_id)
{
	//write_seqlock(&xtime_lock);

	/* clear timer flag */
	writel(P5_SYS_FLAGS_TIM3, PARROT5_VA_SYS+_P5_SYS_FLAGS);

	timer_tick();

	//write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;
}

static struct irqaction parrot5_timer_irq = {
	.name		= "Parrot5 Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= parrot5_timer_interrupt,
};

/**
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init parrot5_timer_init(void)
{
	int i, scale;
	unsigned long tload, ahb_clk, divider, tdivider;

	/* setup timer 3 */
	writel(0, PARROT5_VA_SYS+_P5_SYS_TIM3CTL);

	/* AHB bus clock */
	clk_enable(NULL);
	ahb_clk = clk_get_rate(NULL);

	/* choose proper prescaling (1, 16 or 256), starting from highest
	   resolution */
	tdivider = 0;
	scale = -1;

	for (i = 0; i < 3; i++) {

		/* divider is 2^(4*scale) */
		divider = 1 << (4*i);
		tload = ahb_clk/divider/HZ - 1;

		if (tload <= 0xffff) {
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

	timer_reload = tload;
	writel(tload, PARROT5_VA_SYS+_P5_SYS_TIM3LD);

	/* compute how many ticks we have in 256 useconds
	   note: this only works if AHB clock is a nice multiple of 1 MHz */
	timer_256usec_ticks = (ahb_clk/1000000)*(256/divider);

	/* enable timer */
	writel(scale|
	       P5_SYS_TIMXCTL_PERIODIC|
	       P5_SYS_TIMXCTL_ENABLE,
	       PARROT5_VA_SYS+_P5_SYS_TIM3CTL);

	setup_irq(IRQ_P5_TIMER3, &parrot5_timer_irq);
}

struct sys_timer parrot5_timer = {
	.init		= parrot5_timer_init,
	.offset		= parrot5_gettimeoffset,
};
#endif
