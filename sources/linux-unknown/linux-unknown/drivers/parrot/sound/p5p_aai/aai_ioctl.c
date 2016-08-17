/*
 * linux/drivers/io/aai_ioctl.c
 *	Copyright (c) Parrot SA
 *
 *	Written by Gregoire ETIENNE <gregoire.etienne@parrot.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    Parrot Advanced Audio Interface Driver
 *
 */



#include "aai_ioctl.h"
#include "aai.h"
#include "aai_hw.h"


/** Get a data from user space
 *
 * @param src ptr to userspace data
 * @param dst ptr to kernelspace data
 * @param size of data
 * @return error code
 */
static int get_data(void __user * src, void* dst, int size)
{
   int err=0;
   if (copy_from_user(dst, src, size))
			err = -EFAULT;

   return err;
}

/** Put data to userspace
 *
 * @param src ptr to userspace data
 * @param dst ptr to kernelspace data
 * @param size of data
 * @return error code
 */
static int put_data(void* src, void __user * dst, int size)
{
   int err=0;
   if (copy_to_user(dst, src, size))
      err = -EFAULT;

   return err;
}




/*
 * ioctl for hwdep device:
 */
static int aai_hwdep_ioctl(struct snd_hwdep * hwdep, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned int reg=0, val=0;
	struct card_data_t *aai = hwdep->private_data;

	switch (cmd) {
	case AAI_IOR_STATUS:
      aai_hw_status(aai);
      //err = put_data(&val, (void __user *) arg, sizeof(unsigned int));
		break;

	case AAI_IOWR_READREG:
	   err = get_data((void __user *) arg, &reg, sizeof(unsigned int));
	   if(err == 0){
         val = aai_readreg(aai, reg );
         err = put_data(&val, (void __user *) arg, sizeof(unsigned int));
      }
		break;
	case AAI_IOW_AAISLAVE:
	   err = get_data((void __user *) arg, &val, sizeof(unsigned int));
	   if(err == 0)
         aai_setbit(aai, _AAI_CFG, AAI_CFG_RATE_SLAVE, (!!val) );
		break;
	case AAI_IOW_MASTERCLK:
		err = get_data((void __user *) arg, &val, sizeof(unsigned int));
		if(err == 0)
			aai_set_nbit(aai, _AAI_CFG, AAI_CFG_MASTER_CLK, AAI_CFG_MASTER_CLK_SHIFT, val);
		break;
	default:
      cmd = AAI_IOR_STATUS;
      aai_err(aai->dev,"unknown key 0x%08x\n", cmd);
		err = -EINVAL;
		break;
	}
	return err;
}



static int aai_hwdep_open(struct snd_hwdep * hw, struct file *file)
{
	return 0;
}

static int aai_hwdep_release(struct snd_hwdep * hw, struct file *file)
{
	return 0;
}

int __init aai_ioctl_hwdep_new(struct card_data_t  *aai, char * id, int device)
{
	struct snd_hwdep * rhwdep;
	int err;

	if ((err = snd_hwdep_new(aai->card, "AAIhwdep", 0, &rhwdep)) < 0)
		return err;

	rhwdep->private_data = aai;
	rhwdep->ops.open	 	= aai_hwdep_open;
	rhwdep->ops.ioctl 	= aai_hwdep_ioctl;
	rhwdep->ops.release  = aai_hwdep_release;
	
	return 0;
}
