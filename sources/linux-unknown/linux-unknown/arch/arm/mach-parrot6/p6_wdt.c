/*
 *  Watchdog driver for Parrot 6
 *
 *  Copyright (C) 2010 Parrot SA Matthieu CASTET <matthieu.castet@parrot.com>
 *
 *  based on
 *
 *  Watchdog driver for Broadcom BCM47XX
 *
 *  Copyright (C) 2008 Aleksandar Radovanovic <biblbroks@sezampro.rs>
 *  Copyright (C) 2009 Matthieu CASTET <castet.matthieu@free.fr>
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/reboot.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>
#include <linux/jiffies.h>

#include <asm/io.h>

#include <mach/platform.h>
#include <mach/map.h>

#define DRV_NAME		"p6_wdt"

#define TICK_TO_SEC(x) ((((x)-1)*2)/3174)
/* tick=0 doesn't seem to work ... */
#define SEC_TO_TICK(x) (((x)*3174)/2+1)

#define WDT_DEFAULT_TIME	30	/* seconds */
#define WDT_MAX_TIME		TICK_TO_SEC(0xfffffff)	/* seconds */

static int wdt_time = WDT_DEFAULT_TIME;
static int nowayout = WATCHDOG_NOWAYOUT;

module_param(wdt_time, int, 0);
MODULE_PARM_DESC(wdt_time, "Watchdog time in seconds. (default="
				__MODULE_STRING(WDT_DEFAULT_TIME) ")");

#ifdef CONFIG_WATCHDOG_NOWAYOUT
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
		"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
#endif

static int wdt_tick;
static unsigned long p6_wdt_busy;
static char expect_release;

static inline void p6_wdt_pet(void)
{
	/* we need to do this otherwise counter is not reloaded */
	__raw_writel(0, PARROT6_VA_SYS + _P6_SYS_WDOGCTL);
	__raw_writel(wdt_tick| P6_SYS_WDOGCTL_TICKEN|P6_SYS_WDOGCTL_WDOGEN, PARROT6_VA_SYS + _P6_SYS_WDOGCTL);
}

static void p6_wdt_start(void)
{
	unsigned long flags;
	u32 iten;
	/* this is need even if we don't use IT */
	local_irq_save(flags);
	iten = __raw_readl(PARROT6_VA_SYS+_P6_SYS_ITEN)|P6_SYS_ITEN_TICK;
	__raw_writel(iten, PARROT6_VA_SYS+_P6_SYS_ITEN);
	local_irq_restore(flags);

	p6_wdt_pet();
}

static void p6_wdt_stop(void)
{
	u32 iten;
	unsigned long flags;

	local_irq_save(flags);
	iten = __raw_readl(PARROT6_VA_SYS+_P6_SYS_ITEN) & (~P6_SYS_ITEN_TICK);
	__raw_writel(iten, PARROT6_VA_SYS+_P6_SYS_ITEN);
	local_irq_restore(flags);

	__raw_writel(0, PARROT6_VA_SYS + _P6_SYS_WDOGCTL);
}

static int p6_wdt_settimeout(int new_time)
{
	if ((new_time <= 0) || (new_time > WDT_MAX_TIME))
		return -EINVAL;

	wdt_time = new_time;
	wdt_tick = SEC_TO_TICK(wdt_time);
	return 0;
}

static int p6_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &p6_wdt_busy))
		return -EBUSY;

	p6_wdt_start();
	return nonseekable_open(inode, file);
}

static int p6_wdt_release(struct inode *inode, struct file *file)
{
	if (expect_release == 42) {
		p6_wdt_stop();
	} else {
		printk(KERN_CRIT DRV_NAME
			": Unexpected close, not stopping watchdog!\n");
		p6_wdt_start();
	}

	clear_bit(0, &p6_wdt_busy);
	expect_release = 0;
	return 0;
}

static ssize_t p6_wdt_write(struct file *file, const char __user *data,
				size_t len, loff_t *ppos)
{
	if (len) {
		if (!nowayout) {
			size_t i;

			expect_release = 0;

			for (i = 0; i != len; i++) {
				char c;
				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					expect_release = 42;
			}
		}
		p6_wdt_pet();
	}
	return len;
}

static const struct watchdog_info p6_wdt_info = {
	.identity 	= DRV_NAME,
	.options 	= WDIOF_SETTIMEOUT |
				WDIOF_KEEPALIVEPING |
				WDIOF_MAGICCLOSE,
};

static long p6_wdt_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value, retval = -EINVAL;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &p6_wdt_info,
				sizeof(p6_wdt_info)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);

	case WDIOC_SETOPTIONS:
		if (get_user(new_value, p))
			return -EFAULT;

		if (new_value & WDIOS_DISABLECARD) {
			p6_wdt_stop();
			retval = 0;
		}

		if (new_value & WDIOS_ENABLECARD) {
			p6_wdt_start();
			retval = 0;
		}

		return retval;

	case WDIOC_KEEPALIVE:
		p6_wdt_pet();
		return 0;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;

		if (p6_wdt_settimeout(new_value))
			return -EINVAL;

		p6_wdt_pet();
		return 0;

	case WDIOC_GETTIMEOUT:
		return put_user(wdt_time, p);

	default:
		return -ENOTTY;
	}
}

static int p6_wdt_notify_sys(struct notifier_block *this,
	unsigned long code, void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT)
		p6_wdt_stop();
	return NOTIFY_DONE;
}

static const struct file_operations p6_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.unlocked_ioctl	= p6_wdt_ioctl,
	.open		= p6_wdt_open,
	.release	= p6_wdt_release,
	.write		= p6_wdt_write,
};

static struct miscdevice p6_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &p6_wdt_fops,
};

static struct notifier_block p6_wdt_notifier = {
	.notifier_call = p6_wdt_notify_sys,
};

static int __init p6_wdt_init(void)
{
	int ret;

	if (p6_wdt_settimeout(wdt_time)) {
		p6_wdt_settimeout(WDT_DEFAULT_TIME);
		printk(KERN_INFO DRV_NAME ": "
			"wdt_time value must be 0 < wdt_time < %d, using %d\n",
			(WDT_MAX_TIME + 1), wdt_time);
	}

	ret = register_reboot_notifier(&p6_wdt_notifier);
	if (ret)
		return ret;

	ret = misc_register(&p6_wdt_miscdev);
	if (ret) {
		unregister_reboot_notifier(&p6_wdt_notifier);
		return ret;
	}

	printk(KERN_INFO "P6 Watchdog driver loaded default (%d seconds%s)\n",
				wdt_time, nowayout ? ", nowayout" : "");
	return 0;
}

static void __exit p6_wdt_exit(void)
{
	if (!nowayout)
		p6_wdt_stop();

	misc_deregister(&p6_wdt_miscdev);

	unregister_reboot_notifier(&p6_wdt_notifier);
}

module_init(p6_wdt_init);
module_exit(p6_wdt_exit);

MODULE_AUTHOR("Aleksandar Radovanovic");
MODULE_DESCRIPTION("Watchdog driver for Broadcom P6");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
