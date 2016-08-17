/*
 * linux/drivers/video/p6fb.h
 * Copyright (c) Arnaud Patard
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    PARROT6 LCD Controller Frame Buffer Driver
 *	    based on skeletonfb.c, sa1100fb.h
 *
 */

#ifndef __PARROT6FB_H
#define __PARROT6FB_H

#include <linux/wait.h>
#include <mach/fb.h>

struct p6fb_hw {
	struct device		*dev;
	struct clk		*clk; /* dummy clock */
	struct mutex lock;

	struct p6fb_mach_info *mach_info; /* pointer to platform data */
	
    wait_queue_head_t vbl_wait;
    unsigned int vbl_cnt;

	struct timer_list timer;

	struct resource *mem;
	void __iomem *base;
	struct fb_info *info[3];
};

/* parrot6 lcd private structure */
struct p6fb_info {
	struct fb_info		*fb; /* fb info */
	int plane;
	/* raw memory addresses */
	dma_addr_t		map_dma;	/* physical */
	u_char *		map_cpu;	/* virtual */
	u_int			map_size;

	u32         pseudo_pal[16];

	struct p6fb_hw *hw;
};

#endif
