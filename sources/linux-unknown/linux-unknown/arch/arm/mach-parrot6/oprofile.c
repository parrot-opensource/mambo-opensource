#include <linux/types.h>
#include <linux/errno.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>

#include <asm/io.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>

#include "../oprofile/op_arm_model.h"

#if CONFIG_OPROFILE_PARROT_HZ < 100 || CONFIG_OPROFILE_PARROT_HZ > 10000
#error invalid CONFIG_OPROFILE_PARROT_HZ config
#endif
static int dummy(void)
{ 
	return 0;
}

static irqreturn_t parrot6_timer_interrupt(int irq, void *dev_id)
{
	struct pt_regs *regs = get_irq_regs();

	/* clear timer flag */
	writel(P6_SYS_ITACK_TIM1, PARROT6_VA_SYS+_P6_SYS_ITACK);


	oprofile_add_sample(regs, 0);

	return IRQ_HANDLED;
}

static struct irqaction parrot6_timer_irq = {
	.name		= "Oprofile (Timer 1)",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= parrot6_timer_interrupt,
};

static int start(void)
{
	unsigned long ahb_clk;
	unsigned long load;
	u32 iten;

	ahb_clk = clk_get_rate(NULL);
	load = ahb_clk/CONFIG_OPROFILE_PARROT_HZ;

	BUG_ON(load == 0);
	writel(load - 1, PARROT6_VA_SYS+_P6_SYS_TIM1LD);
	/* enable timer */
	writel(P6_SYS_TIMXCTL_PERIODIC|P6_SYS_TIMXCTL_ENABLE,
			PARROT6_VA_SYS+_P6_SYS_TIM1CTL);
	/* enable timer interrupt */
	iten = __raw_readl(PARROT6_VA_SYS+_P6_SYS_ITEN)|P6_SYS_ITEN_TIM1;
	__raw_writel(iten, PARROT6_VA_SYS+_P6_SYS_ITEN);
	setup_irq(IRQ_P6_TIMER1, &parrot6_timer_irq);
	return 0;
}

static void stop(void)
{
	writel(0, PARROT6_VA_SYS+_P6_SYS_TIM1CTL);
}

struct op_arm_model_spec parrot = {
	.init = dummy,
	.num_counters = 0,
	.setup_ctrs = dummy,
	.start = start,
	.stop = stop,
	"timer",
};
