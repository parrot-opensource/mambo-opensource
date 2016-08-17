/**
********************************************************************************
* @file dmamem.c
* @brief dmamem generic driver
*
* Copyright (C) 2009 Parrot S.A.
*
* @author     Matthieu CASTET <matthieu.castet@parrot.com>
* @date       2009-07-10
********************************************************************************
*/

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>

#include "dmamem_ioctl.h"

#define DMAMEM_DRIVER_NAME "dmamem driver"

/* generic dmamem interface */
struct device   *dmamem_dev;

struct dmamem_list {
	struct  list_head list;
	struct dmamem_alloc data;
};

static int dmamem_open(struct inode *inode, struct file *filp)
{
	struct  list_head *head;

    head = kzalloc(sizeof(struct list_head), GFP_KERNEL);
	INIT_LIST_HEAD(head);

    filp->private_data = head;
    return 0;
}

static int dmamem_release(struct inode *inode, struct file *filp)
{
	struct list_head *head = filp->private_data;
	struct dmamem_list *ldata;

	list_for_each_entry(ldata, head, list) {
		struct dmamem_alloc *data = &ldata->data;
		dma_free_coherent(dmamem_dev, data->size, data->cpu_addr, (dma_addr_t)data->phy_addr);
		kfree(ldata);
	}
	kfree(head);
    filp->private_data = NULL;
    return 0;
}

/* copied from arm/mm/trap.c */
static inline void
do_cache_op(unsigned long start, unsigned long end, int flags)
{
	struct vm_area_struct *vma;

	if (end < start || flags)
		return;

	vma = find_vma(current->active_mm, start);
	if (vma && vma->vm_start < end) {
		if (start < vma->vm_start)
			start = vma->vm_start;
		if (end > vma->vm_end)
			end = vma->vm_end;

		//flush_cache_user_range(vma, start, end);
		flush_cache_range(vma, start, end);
	}
}

static int dmamem_ioctl(struct inode *inode, struct file *filp,
        unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    switch (cmd)
    {
        case DMAMEM_ALLOC:
			{
				struct  list_head *head = filp->private_data;
				struct dmamem_list *ldata;
				struct dmamem_alloc *data;
				dma_addr_t dma_handle;
				void *cpu_addr;

				ldata = kzalloc(sizeof(struct dmamem_list), GFP_KERNEL);
				if (!ldata) {
					ret = -ENOMEM;
					break;
				}
				INIT_LIST_HEAD(&ldata->list);
				data = &ldata->data;

				if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
					ret = -EFAULT;
					kfree(ldata);
					break;
				}

				cpu_addr = dma_alloc_coherent(dmamem_dev, data->size, &dma_handle, GFP_KERNEL);
				if (!cpu_addr) {
					ret = -ENOMEM;
					kfree(ldata);
					break;
				}
				data->cpu_addr = cpu_addr;
				data->phy_addr = (void *)dma_handle;
				if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
					ret = -EFAULT;
					kfree(ldata);
					break;
				}
				list_add_tail(&ldata->list, head);
			}
            break;
        case DMAMEM_ARM_FLUSH_INV:
			{
				struct dmamem_flush_inv data;
				if (copy_from_user(&data, (void __user *)arg, sizeof(data))) {
					ret = -EFAULT;
					break;
				}
				do_cache_op(data.start, data.start+data.len, 0);
			}
			break;
        default:
            ret = -ENOTTY;
    }
    return ret;
}

struct file_operations dmamem_fops = {
    .ioctl =     dmamem_ioctl,
    .open =      dmamem_open,
    .release =   dmamem_release,
};

static struct miscdevice dmamem_miscdev = {
	.minor = 64 + 3,
	.name = "dmamem",
	.fops = &dmamem_fops,
};

static int init_device(void)
{
	return misc_register(&dmamem_miscdev);
}

static int dmamem_probe(struct platform_device *pdev)
{
	struct resource *res;
    int err = 0;

    dmamem_dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		err = dma_declare_coherent_memory(&pdev->dev, res->start,
						  res->start,
						  (res->end - res->start) + 1,
						  DMA_MEMORY_MAP |
						  DMA_MEMORY_EXCLUSIVE);
		if (!err) {
			dev_err(&pdev->dev, "Unable to declare memory.\n");
			err = -ENXIO;
			goto err2;
		}
	}
	else {
		dev_info(&pdev->dev, "No static mem pool : big memory allocation can fail\n");
	}

    err = init_device();
    if (err < 0)
        goto err;
 
    return 0;
err:
	dma_release_declared_memory(&pdev->dev);
err2:
    return err;
}

static int dmamem_remove(struct platform_device *pdev)
{

    /* we are protected by module refcount.
     * this call is possible only if there is no user
     */
	misc_deregister(&dmamem_miscdev);
	dma_release_declared_memory(&pdev->dev);
    dev_info(&pdev->dev, "driver removed\n");

    return 0;
}

static struct platform_driver dmamem_driver = {
    .probe		= dmamem_probe,
    .remove		= dmamem_remove,
#if 0
    .suspend	= dmamem_suspend,
    .resume		= dmamem_resume,
#endif
    .driver		= {
        .name	= "dmamem",
    },
};

static int __devinit dmamem_init(void)
{
    return platform_driver_register(&dmamem_driver);
}

static void __exit dmamem_exit(void)
{
    platform_driver_unregister(&dmamem_driver);
}

module_init(dmamem_init);
module_exit(dmamem_exit);

MODULE_AUTHOR("PARROT SA by Matthieu CASTET <matthieu.castet@parrot.com>");
MODULE_DESCRIPTION("dmamem driver");
MODULE_LICENSE("GPL");
