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

static irqreturn_t parrot5_timer_interrupt(int irq, void *dev_id)
{
	struct pt_regs *regs = get_irq_regs();

	/* clear timer flag */
	writel(P5_SYS_FLAGS_TIM1, PARROT5_VA_SYS+_P5_SYS_FLAGS);


	oprofile_add_sample(regs, 0);

	return IRQ_HANDLED;
}

static struct irqaction parrot5_timer_irq = {
	.name		= "Oprofile (Timer 1)",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= parrot5_timer_interrupt,
};

static int start(void)
{
	unsigned long ahb_clk;
	unsigned long load;

	BUG_ON(parrot_chip_is_p5p() == 0);

	ahb_clk = clk_get_rate(NULL);
	load = ahb_clk/CONFIG_OPROFILE_PARROT_HZ;

	BUG_ON(load == 0);
	writel(load - 1, PARROT5_VA_SYS+_P5_SYS_TIM1LD);
	/* enable timer */
	writel(P5_SYS_TIMXCTL_PERIODIC|P5_SYS_TIMXCTL_ENABLE,
			PARROT5_VA_SYS+_P5_SYS_TIM1CTL);
	setup_irq(IRQ_P5_TIMER1, &parrot5_timer_irq);
	return 0;
}

static void stop(void)
{
	writel(0, PARROT5_VA_SYS+_P5_SYS_TIM1CTL);
}

struct op_arm_model_spec parrot = {
	.init = dummy,
	.num_counters = 0,
	.setup_ctrs = dummy,
	.start = start,
	.stop = stop,
	"timer",
};
