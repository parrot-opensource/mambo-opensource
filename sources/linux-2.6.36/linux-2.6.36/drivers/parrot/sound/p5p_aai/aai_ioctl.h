#ifndef INCONCE_AAI_IO_H
#define INCONCE_AAI_IO_H

/*
 * linux/drivers/io/aai_ioctl.h
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


#include <linux/ioctl.h>

#define AAI_IOW_AAISLAVE       _IOW('p', 1, int)
#define AAI_IOW_MASTERCLK      _IOW('p', 6, int)

#define AAI_IOR_STATUS         _IOR('p', 96, int)

#define AAI_IOWR_READREG       _IOWR('p', 100, int)
#endif //INCONCE_AAI_IO_H









