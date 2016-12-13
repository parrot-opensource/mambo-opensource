/*
 *
 * Copyright (c) 2008 Parrot SA
 *
 * Inspired by pxafb.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARM_FB_H
#define __ASM_ARM_FB_H
#include <asm/types.h>

struct p6fb_val {
	__u32	defval;
	__u32	min;
	__u32	max;
};

struct p6fb_mach_info {
	/* Screen size */
	__u32		width;
	__u32		height;

	/* Screen info */
	struct p6fb_val xres;
	struct p6fb_val yres;
	struct p6fb_val bpp;
	struct p6fb_val pixclock;

	 __u32 smem_len[3];

	__u32 left_margin;		/* time from sync to picture	*/
	__u32 right_margin;		/* time from picture to sync	*/
	__u32 upper_margin;		/* time from sync to picture	*/
	__u32 lower_margin;
	__u32 hsync_len;		/* length of horizontal sync	*/
	__u32 vsync_len;		/* length of vertical sync	*/

	__u32 panel;


	/* for mem allocated by bootloader */
	void *map_base;
	unsigned long map_size;
};

#endif /* __ASM_ARM_FB_H */
