/**
 ********************************************************************************
 * @file pwm.c
 * @brief pwm generic driver
 *
 * Copyright (C) 2007 Parrot S.A.
 *
 * @author     Matthieu CASTET <matthieu.castet@parrot.com>
 * @date       2007-06-15
 ********************************************************************************
 */


#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/init.h>
//XXX for kfree ???
#include <linux/slab.h>
#include "pwm_ops.h"
#include "pwm_ioctl.h"

#define PWM_ID_NONE 0xffffffff

/* generic pwm interface */
struct pwm_device {
	struct mutex lock; /* semaphore to serialise access */
	struct pwm_ops *pwm_ops;
};

static struct pwm_device device;

struct pwm_info {
	int id_used; /*!< pwm number in used */
	struct pwm_device *dev;
};

/**
 * request a pwm an select it for the current file descriptor
 *
 * @param info private per file info
 * @id pwm id
 *
 * @return error code (0 = ok, < 0 error)
 *         -EBUSY : the pwm is is already used or we already have a
 *                     pwm selected on our file descriptor
 */
/* TODO do a whitelist mode */
static int ioctl_pwm_request(struct pwm_info *info, unsigned int id)
{
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int err;

	/* check if we don't have already requested a pwm */
	if (info->id_used != PWM_ID_NONE) {
		err = -EBUSY;
		goto exit;
	}

	/* request the pwm */
	err = ops->pwm_request(id);
	if (err) {
		err = -EBUSY;
		goto exit;
	}

	/* set the pwm as used */
	info->id_used = id;
exit:
	return err;

}

/**
 * release a pwm
 *
 * This will stop a pwm and release it.
 *
 * @param info private per file info
 * @id pwm id
 *
 * @return error code (0 = ok, < 0 error)
 *          -EINVAL : id is invalid
 */
static int ioctl_pwm_release(struct pwm_info *info, unsigned int id)
{
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int err = 0;

	/* check if we have requested the pwm we want to free */
	if (info->id_used != id) {
		err = -EINVAL;
		goto exit;
	}

	/* this can failed if id is invalied, but we already check it */
#ifdef STOP_ON_EXIT
	ops->pwm_stop(id);
#endif
	ops->pwm_release(id);
	info->id_used = PWM_ID_NONE;

exit:
	return err;
}

/**
 * start a pwm signal
 *
 * @param info private per file info
 *
 * @return error code (0 = ok, < 0 error)
 *          -EINVAL : id is invalid
 */
static int ioctl_pwm_start(struct pwm_info *info)
{
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int err;

	if (info->id_used == PWM_ID_NONE) {
		err = -EINVAL;
		goto exit;
	}

	err = ops->pwm_start(info->id_used);
exit:
	return err;
}

/**
 * stop a pwm signal
 *
 * @param info private per file info
 *
 * @return error code (0 = ok, < 0 error)
 *          -EINVAL : id is invalid
 */
static int ioctl_pwm_stop(struct pwm_info *info)
{
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int err;

	if (info->id_used == PWM_ID_NONE) {
		err = -EINVAL;
		goto exit;
	}

	err = ops->pwm_stop(info->id_used);
exit:
	return err;
}

/**
 * configure the frequency of the pwm
 *
 * @param info private per file info
 * @param freq frequency of the pwm (in HZ)
 *
 * @return error code (0 = ok, < 0 error)
 *          -EINVAL : id is invalid
 *                    frequency is out of range
 */
static int ioctl_pwm_set_freq(struct pwm_info *info, unsigned int freq)
{
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int err;

	if (info->id_used == PWM_ID_NONE) {
		err = -EINVAL;
		goto exit;
	}

	err = ops->pwm_set_freq(info->id_used, freq);
exit:
	return err;
}

/**
 * get the current pwm frequency
 *
 * @param info private per file info
 * @param freq frequency of the pwm
 *
 * @return error code (0 = ok, < 0 error)
 *          -EINVAL : id is invalid
 */
static int ioctl_pwm_get_freq(struct pwm_info *info, unsigned int *freq)
{
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int err;

	if (info->id_used == PWM_ID_NONE) {
		err = -EINVAL;
		goto exit;
	}

	err = ops->pwm_get_freq(info->id_used, freq);
exit:
	return err;
}

/**
 * configure the duty cycle
 *
 * @param info private per file info
 * @param ratio (percentage * 100 of the dc) (range 0 ... PWM_WIDTH_MAX)
 *
 * @return error code (0 = ok, < 0 error)
 *          -EINVAL : id is invalid
 */
static int ioctl_pwm_set_width(struct pwm_info *info, unsigned int ratio)
{
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int err;

	if (info->id_used == PWM_ID_NONE) {
		err = -EINVAL;
		goto exit;
	}

	err = ops->pwm_set_width(info->id_used, ratio);
exit:
	return err;
}

/**
 * send private ioctl to driver
 *
 * @param info private per file info
 * @param cmd
 * @param arg
 *
 * @return error code (0 = ok, < 0 error)
 *          -EINVAL : id is invalid
 */
static int ioctl_pwm_ioctl(struct pwm_info *info, unsigned int cmd, unsigned long arg)
{
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int err;

	if (!ops->pwm_ioctl) {
		err = -ENOTTY;
		goto exit;
	}

	err = ops->pwm_ioctl(info->id_used, cmd, arg);
	if ( (!err) && (info->id_used == PWM_ID_NONE) )
	{
		info->id_used = 0x0F;
	}
exit:
	return err;
}

/**
 * get the current duty cycle
 *
 * @param info private per file info
 * @param ratio (percentage * 100 of the dc) (range 0 ... PWM_WIDTH_MAX)
 *
 * @return error code (0 = ok, < 0 error)
 *          -EINVAL : id is invalid
 */
static int ioctl_pwm_get_width(struct pwm_info *info, unsigned int *ratio)
{
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int err;

	if (info->id_used == PWM_ID_NONE) {
		err = -EINVAL;
		goto exit;
	}

	err = ops->pwm_get_width(info->id_used, ratio);
exit:
	return err;
}

/**
 * method called when the device is opened
 * Do check and alloc.
 */
static int pwm_open(struct inode *inode, struct file *filp)
{
	struct pwm_device *dev = &device;
	struct pwm_info *info = NULL;
	struct pwm_ops *ops;
	int err = 0;

	/* mutex to protect pwm_ops check and try_module_get */
	if (mutex_lock_interruptible(&dev->lock))
		return -ERESTARTSYS;

	ops = dev->pwm_ops;
	if (!ops) {
		err = -EINVAL;
		goto exit;
	}

	info = kzalloc(sizeof(struct pwm_info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto exit;
	}

	if (try_module_get(ops->owner) == 0) {
		err = -EBUSY;
		goto exit;
	}

	info->id_used = PWM_ID_NONE;
	info->dev = dev;
	filp->private_data = info;

exit:
	mutex_unlock(&dev->lock);
	if (err)
		kfree(info);
	return err;
}

/**
 * Method called when the file is closed
 */
static int pwm_release(struct inode *inode, struct file *filp)
{
	struct pwm_info *info = (struct pwm_info *)filp->private_data;
	struct pwm_ops *ops = info->dev->pwm_ops;

	/* release remaining pwm users */
	if (info->id_used != PWM_ID_NONE) {
		/* don't check return value as we checked that the id is valid */
		ioctl_pwm_release(info, info->id_used);
	}
	kfree(info);
	filp->private_data = NULL;
	module_put(ops->owner);
	return 0;
}

static long pwm_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct pwm_info *info = filp->private_data;
	struct pwm_device *dev = info->dev;
	struct pwm_ops *ops = dev->pwm_ops;
	int ret = 0;

	if (!ops)
		return -EINVAL;

	if (mutex_lock_interruptible(&dev->lock))
		return -ERESTARTSYS;


	switch (cmd) {
		case PWM_REQUEST:
		case PWM_RELEASE:
			{
				unsigned int pwm;
				if (get_user(pwm, (int __user *) arg)) {
					ret = -EFAULT;
				}
				if (cmd == PWM_REQUEST)
					ret = ioctl_pwm_request(info, pwm);
				else
					ret = ioctl_pwm_release(info, pwm);
				break;
			}
		case PWM_START:
			ret = ioctl_pwm_start(info);
			break;
		case PWM_STOP:
			ret = ioctl_pwm_stop(info);
			break;
		case PWM_SET_FREQ:
			{
				unsigned int pwm_freq;
				if (get_user(pwm_freq, (int __user *) arg)) {
					ret = -EFAULT;
					break;
				}
				ret = ioctl_pwm_set_freq(info, pwm_freq);
				break;
			}
		case PWM_SET_WIDTH:
			{
				unsigned int pwm_width;
				if (get_user(pwm_width, (int __user *) arg)) {
					ret = -EFAULT;
					break;
				}
				ret = ioctl_pwm_set_width(info, pwm_width);
				break;
			}
		case PWM_GET_FREQ:
			{
				unsigned int pwm_freq;
				ret = ioctl_pwm_get_freq(info, &pwm_freq);
				if (put_user(pwm_freq, (int __user *) arg)) {
					ret = -EFAULT;
				}
				break;
			}
		case PWM_GET_WIDTH:
			{
				unsigned int pwm_width;
				ret = ioctl_pwm_get_width(info, &pwm_width);
				if (put_user(pwm_width, (int __user *) arg)) {
					ret = -EFAULT;
				}
				break;
			}
		case PWM_MAX:
			{
				if (put_user(ops->pwm_max, (int __user *) arg))
					ret = -EFAULT;
				break;
			}
		default:
			{
				ret = ioctl_pwm_ioctl(info, cmd, arg);
				break;
			}
	}
	mutex_unlock(&dev->lock);
	return ret;
}

struct file_operations pwm_fops = {
	.unlocked_ioctl =     pwm_ioctl,
	.open =      pwm_open,
	.release =   pwm_release,
};

static struct miscdevice pwm_miscdev = {
	.minor = 64 + 0,
	.name = "pwm",
	.fops = &pwm_fops,
};

static int init_device(void)
{
	return misc_register(&pwm_miscdev);
}

static int __devinit pwm_init(void)
{
	mutex_init(&device.lock);
	return init_device();
}

static void __exit pwm_exit(void)
{
	/* we are protected by module refcount.
	 * this call is possible only if there is no user
	 */
	misc_deregister(&pwm_miscdev);
}

module_init(pwm_init);
module_exit(pwm_exit);

/**
 * register_pwm : register pwm platform driver in the pwm core
 *
 * @param pwm_ops : pwm platform driver callbacks
 * @return 0 or an error code
 */
int register_pwm(struct pwm_ops* pwm_ops)
{
	if (device.pwm_ops)
		return -EBUSY;
	/* check if everthing is initialised */
	if (!(pwm_ops->pwm_max && pwm_ops->pwm_start && pwm_ops->pwm_stop
				&& pwm_ops->pwm_request && pwm_ops->pwm_release
				&& pwm_ops->pwm_set_width && pwm_ops->pwm_set_freq
				&& pwm_ops->pwm_get_width && pwm_ops->pwm_get_freq ))
		return -EINVAL;

	device.pwm_ops = pwm_ops;
	return 0;
}
EXPORT_SYMBOL(register_pwm);

/**
 * unregister_pwm : unregister pwm platform subdriver in the pwm core
 *
 * This should be only called when pwm_ops->owner refcount is zero, ie only
 * platform subdriver module_exit.
 *
 * @param pwm_ops : pwm platform driver callbacks
 * @return 0 or an error code
 */
int unregister_pwm(struct pwm_ops* pwm_ops)
{
	if (device.pwm_ops != pwm_ops)
		return -EINVAL;
	device.pwm_ops = NULL;
	return 0;
}
EXPORT_SYMBOL(unregister_pwm);

MODULE_AUTHOR("PARROT SA by Matthieu CASTET <matthieu.castet@parrot.com>");
MODULE_DESCRIPTION("pwm driver");
MODULE_LICENSE("GPL");
