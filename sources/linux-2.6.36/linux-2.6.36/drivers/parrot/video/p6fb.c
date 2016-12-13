/*
 * linux/drivers/video/p6fb.c
 *	Copyright (c) Parrot SA
 *
 *	Written by Matthieu CASTET <matthieu.castet@parrot.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	    Parrot6 LCD Controller Frame Buffer Driver
 *	    based on p6fb, skeletonfb.c, sa1100fb.c and others
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <mach/regs-lcdc-p6.h>

#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/div64.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include <asm/mach-types.h>

#include "p6fb.h"
#include "p6fb_ioctl.h"
#define P6FB_NBR_VIDEO_BUFFERS 2
#define P6FB_NBR_VIDEO_PLANE 3
//#define CONFIG_FB_PARROT6_DEBUG

static struct p6fb_mach_info *mach_info;

/* Debugging stuff */
#ifdef CONFIG_FB_PARROT6_DEBUG
static int debug	   = 1;
#else
static int debug;
#endif

//#define dprintk(msg...)	if (debug) { printk(KERN_DEBUG "p6fb: " msg); }
#define dprintk(msg...)	if (debug) { printk(KERN_INFO "p6fb: " msg); }
#define vprintk(msg...)

#define ROUNDUP(x,y) (((x)+(y)-1)/(y))

/* useful functions */

#define p6fb_bit_set(addr, offset, mask, value) \
do { \
	BUG_ON((value) & ~(mask)); /* check we don't overflow */\
	addr &= ~((mask)<<(offset)); \
	addr |= (value)<<(offset); \
} while (0)

/* rnb4 */
static u_char		*rnb4_mask_cpu;
static dma_addr_t	rnb4_mask_dma;

static const u16 rnb4_mask_compress[] =
{
	396, 394, 393, 391, 389, 388, 386, 384, 383, 381, 379, 378, 376, 374, 373, 371, 370, 368, 366, 365,
	363, 361, 360, 358, 356, 355, 353, 351, 350, 348, 346, 345, 343, 341, 340, 338, 336, 335, 333, 331,
	330, 328, 326, 325, 323, 321, 320, 318, 316, 315, 313, 311, 310, 308, 306, 305, 303, 301, 300, 298,
	296, 295, 293, 291, 290, 288, 287, 285, 283, 282, 280, 278, 277, 275, 273, 272, 270, 268, 267, 265,
	263, 262, 260, 258, 257, 255, 253, 252, 250, 248, 247, 245, 243, 242, 240, 238, 237, 235, 233, 232,
	230, 228, 227, 225, 223, 222, 220, 218, 217, 215, 213, 212, 210, 209, 207, 205, 204, 202, 200, 199,
	197, 195, 194, 192, 190, 189, 187, 185, 184, 182, 180, 179, 177, 175, 174, 172, 170, 169, 167, 165,
	164, 162, 160, 159, 157, 155, 154, 152, 150, 149, 147, 145, 144, 142, 140, 139, 137, 135, 134, 132,
	130, 129, 127, 126, 124, 122, 121, 119, 117, 116, 114, 112, 111, 109, 107, 106, 104, 102, 101, 99,
	97, 96, 94, 92, 91, 89, 87, 86, 84, 82, 81, 79, 77, 76, 74, 72, 71, 69, 67, 66,
	64, 62, 61, 59, 57, 56, 54, 52, 51, 49, 48, 46, 44, 43, 41,  39, 38, 36, 34, 33,
	31, 29, 28, 26, 24, 23, 21, 19, 18, 16, 14, 13, 11, 9, 8, 6, 4, 3, 1, 0
};

/* p6fb_set_lcdaddr
 *
 * initialise lcd controller address pointers
*/

static int p6fb_wait_vbl1(struct p6fb_info *fbi)
{
	unsigned long cnt;
	int ret;
	cnt = fbi->hw->vbl_cnt;
	ret = wait_event_interruptible_timeout(fbi->hw->vbl_wait, cnt != fbi->hw->vbl_cnt, HZ);
	return ret > 0;
}
#define p6fb_wait_vbl(x) ({ \
		int r; \
		vprintk(KERN_DEBUG"wait vbl start %s %d\n", __func__, __LINE__); \
		r = p6fb_wait_vbl1(x); \
		vprintk(KERN_DEBUG"wait vbl stop %s\n", __func__); \
	r;})

static void sync_reg(struct p6fb_info *fbi)
{
	int ret;
	ret = p6fb_wait_vbl(fbi);
	WARN_ON(ret == 0);
	ret = p6fb_wait_vbl(fbi);
	WARN_ON(ret == 0);
}

#define stop_plane(x,y) do { \
		vprintk(KERN_DEBUG"stop plane start %s %d\n", __func__, __LINE__); \
		stop_plane1(x,y); \
		vprintk(KERN_DEBUG"stop plane stop %s\n", __func__); \
	} while (0)

static void stop_plane1(struct p6fb_info *fbi, int plane)
{
	void __iomem *base = fbi->hw->base;
	unsigned int ctrl;
	unsigned int bit;
	if (plane == 0) {
		bit = P6_LCDC_CTRL_RGB_EN;
	}
	else {
		bit = P6_LCDC_CTRL_YCCX_EN(plane);
	}
	ctrl = __raw_readl(base+P6_LCDC_CTRL);
	p6fb_bit_set(ctrl, 0, bit, 0);
	/* disable plane */
	__raw_writel(ctrl, base+P6_LCDC_CTRL);
	/* only need to sync register if output enabled */
	if (ctrl & P6_LCDC_CTRL_OUTEN)
		sync_reg(fbi);
}

static int p6fb_fifo_threshold(struct fb_var_screeninfo *var, int plane)
{
	int ret;
	if (var->bits_per_pixel == 32) {
		if (var->xres == 800) {
			//ret = 0x38;
			ret = 0xf0;
		}
		else if (var->xres < 288) {
			ret = 0x80;
		}
		else {
			ret = 0xf0;
		}
	}
	else if (plane == 0) {
		if (var->xres == 800) {
			//ret = 0xb8;
			ret = 0xe0;
		}
		else if (var->xres < 544) {
			ret = 0x80;
		}
		else if (var->xres == 544) {
			ret = 0xf8;
		}
		else {
			ret = 0xf0;
		}
	}
	else {
		if (var->xres == 800) {
			ret = 0x38;
		}
		else if (var->xres < 544) {
			ret = 0x40;
		}
		else if (var->xres == 544) {
			ret = 0x78;
		}
		else {
			ret = 0x70;
		}
	}

	if (ret >= 0xf0 || (var->bits_per_pixel != 32 && plane && ret >= 0x70))
		printk(KERN_WARNING"using not optimal thresold 0x%x for plane %d\n", 
			ret, plane);
	return ret;
}

static void p6fb_set_lcdaddr(struct fb_var_screeninfo *var, struct p6fb_info *fbi)
{
	unsigned long addr;
	void __iomem *base = fbi->hw->base;
	/* we are in double buffer mode, but we don't use it
	 * we use the fact that the address are updated at vsync
	 * by the dma
	 */
	BUG_ON(var->xoffset);

	addr  = fbi->fb->fix.smem_start;

	if (fbi->plane == 0) {
		addr += (var->xres_virtual * var->yoffset + var->xoffset) * var->bits_per_pixel / 8;

		if (machine_is_rnb4()) {
			static char first_time = 0;
			unsigned long old_addr = __raw_readl(base+P6_LCDC_RGB_A1);

			if(first_time == 0)
			{
				first_time = 1;
				old_addr = addr;
				__raw_writel(addr, base+P6_LCDC_RGB_A1);
				__raw_writel(addr, base+P6_LCDC_RGB_A2);
			}

			/* rnb4 */
			if (addr != old_addr)
			{
				unsigned int ctrl = __raw_readl(base+P6_LCDC_CTRL);
				/* first draw old image on plane 1 */
				int curr_buff = __raw_readl(base+P6_LCDC_YCC1_BUFFER+(fbi->plane-1)*4);
				int plane = 1;
				unsigned long flags;

				unsigned int bit = P6_LCDC_CTRL_YCCX_EN(1);

				/* check that plane 1 is off */
				BUG_ON(ctrl & bit);

				/* update address */
				__raw_writel(old_addr, base+P6_LCDC_Y1_A1+(plane-1)*4*12+curr_buff*4);
				/* enable plane 1 */
				__raw_writel(ctrl|bit, base+P6_LCDC_CTRL);
				local_irq_save(flags);
				/* activate osd pixel */
				__raw_writel(P6_LCDC_OSD_CTL_XPIXEL(plane)|P6_LCDC_OSD_CTL_X_EN(plane), base+P6_LCDC_OSD_CTL);

				/* plan 0 : new image : this will be updated at next vbl */
				__raw_writel(addr, base+P6_LCDC_RGB_A1);
				__raw_writel(addr, base+P6_LCDC_RGB_A2);
				local_irq_restore(flags);

				/* wait draw of screen */
				/* there can be a small race here : between P6_LCDC_RGB_A1
				   and the wait vbl can have happened ...
				   But we disable osd in the irq handler.
				 */
				p6fb_wait_vbl(fbi);

				/* disable osd pixel */
				__raw_writel(0, base+P6_LCDC_OSD_CTL);
				/* disable plane 1 */
				__raw_writel(ctrl, base+P6_LCDC_CTRL);
			}
		}
		else {
			unsigned long flags;
			/* this is need because we shouldn't have more than 1 frame
			   between A1 and A2 write */
			local_irq_save(flags);
			__raw_writel(addr, base+P6_LCDC_RGB_A1);
			__raw_writel(addr, base+P6_LCDC_RGB_A2);
			local_irq_restore(flags);
		}
	}
	else {
		int curr_buff = __raw_readl(base+P6_LCDC_YCC1_BUFFER+(fbi->plane-1)*4);
		unsigned long off_yc = 0, off_cb = 0, off_cr = 0, off_alpha = 0;
		unsigned long dmaoffset_y = 0, dmaoffset_c = 0;
		unsigned long flags;
		u32 dmaoffset;
		curr_buff &= P6_LCDC_YCC1_BUFFER_MASK;

		curr_buff = (curr_buff + 1) & 1;

		off_yc = (var->xres_virtual * var->yoffset + var->xoffset) * var->bits_per_pixel / 8;
		dmaoffset_y = (var->xres_virtual - var->xres) * 64 / var->bits_per_pixel;
		switch (var->nonstd) {
			case NOSTD_RGB:
				off_alpha = off_cb = off_cr = 0xdeadbeef - addr;
				if (var->bits_per_pixel != 32) {
					off_alpha = var->xres_virtual * var->yres_virtual * var->bits_per_pixel / 8;
					/* add virtual offset */
					off_alpha += off_yc * 8 / var->bits_per_pixel;
				}
				/* x offset */
				dmaoffset_c = 0xDE;
				break;
			case NOSTD_YUV420:
				off_cb = (var->xres_virtual * var->yres_virtual);
				off_cr = off_cb + off_cb/4;
				off_alpha = off_cr + off_cb/4;
				/* add virtual offset */
				off_cb += off_yc/4;
				off_cr += off_yc/4;
				off_alpha += off_yc;
				/* x offset */
				dmaoffset_c = dmaoffset_y / 2;
				break;
			case NOSTD_YVU420:
				off_cr = (var->xres_virtual * var->yres_virtual);
				off_cb = off_cr + off_cr/4;
				off_alpha = off_cb + off_cr/4;
				/* add virtual offset */
				off_cb += off_yc/4;
				off_cr += off_yc/4;
				off_alpha += off_yc;
				/* x offset */
				dmaoffset_c = dmaoffset_y / 2;
				break;
		}

		local_irq_save(flags);
		dmaoffset = __raw_readl(base+P6_LCDC_DMAO);
		p6fb_bit_set(dmaoffset, 16*(fbi->plane-1), 0xffff,
				dmaoffset_y|(dmaoffset_c<<8));
		__raw_writel(dmaoffset, base+P6_LCDC_DMAO);
		local_irq_restore(flags);
		/* update address */
		__raw_writel(addr + off_yc, base+P6_LCDC_Y1_A1+(fbi->plane-1)*4*12+curr_buff*4);
		__raw_writel(addr + off_cb, base+P6_LCDC_CB1_A1+(fbi->plane-1)*4*12+curr_buff*4);
		__raw_writel(addr + off_cr, base+P6_LCDC_CR1_A1+(fbi->plane-1)*4*12+curr_buff*4);
		__raw_writel(addr + off_alpha, base+P6_LCDC_OSD1_A1+(fbi->plane-1)*4*12+curr_buff*4);


		/* switch buffer */
		__raw_writel(curr_buff|P6_LCDC_YCC1_SRC, base+P6_LCDC_YCC1_BUFFER+(fbi->plane-1)*4);
	}
}

/* p6fb_calc_pixclk()
 *
 * calculate divisor for clk->pixclk
*/

static unsigned int p6fb_calc_pixclk(struct p6fb_info *fbi,
					  unsigned long pixclk)
{
	unsigned long clk = clk_get_rate(fbi->hw->clk);
	unsigned long long div;

	/* pixclk is in picoseoncds, our clock is in Hz
	 *
	 * Hz -> picoseconds is / 10^-12
	 */

	div = (unsigned long long)clk * pixclk;
	do_div(div, 1000000UL);
	do_div(div, 1000000UL);

	dprintk("pixclk %ld, divisor is %ld\n", pixclk, (long)div);
	return div;
}

static unsigned int p6fb_calc_clkdiv(struct p6fb_info *fbi,
					  struct fb_var_screeninfo *var)
{
	int clkdiv = p6fb_calc_pixclk(fbi, var->pixclock);

	clkdiv = (clkdiv / 2) - 1;
	/* clkdiv == 0 is forbiden */
	if (clkdiv < 1)
		clkdiv = 1;
	dprintk("hardware div %d\n", clkdiv);
	return clkdiv;
}

static unsigned int p6fb_get_pixclk(struct p6fb_info *fbi,
					  struct fb_var_screeninfo *var)
{
	unsigned long long div;
	unsigned long clk = clk_get_rate(fbi->hw->clk);
	unsigned int clkdiv = p6fb_calc_clkdiv(fbi, var);

	clkdiv = (clkdiv + 1) * 2;

	div = clkdiv * 1000000UL;
	div =    div * 1000000UL;

	do_div(div, clk);
	/* avoid rounding issue with + 1*/
	return div + 1;
}

/*
 *	p6fb_check_var():
 *	Get the video params out of 'var'. If a value doesn't fit, round it up,
 *	if it's too big, return -EINVAL.
 *
 */
static int p6fb_ycc_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct p6fb_info *fbi = info->par;
	void __iomem *base = fbi->hw->base;
	int plane = fbi->plane;
	u32 pos, osd;
	int pos_x, pos_y;
	int real_bits_per_pixel;

	dprintk("check_var(var=%p, info=%p)\n", var, info);

	dprintk("%s: var->xres  = %d\n", __func__, var->xres);
	dprintk("%s: var->yres  = %d\n", __func__, var->yres);
	dprintk("%s: var->xres_v= %d\n", __func__, var->xres_virtual);
	dprintk("%s: var->yres_v= %d\n", __func__, var->yres_virtual);
	dprintk("%s: var->bpp   = %d\n", __func__, var->bits_per_pixel);

	pos = __raw_readl(base+P6_LCDC_YCC1_P+(plane-1)*4);
	pos_x = pos & 0xffff;
	pos_y = pos >> 16;

	var->xoffset = 0;

	if (var->yres > fbi->hw->info[0]->var.yres - pos_y)
		var->yres = fbi->hw->info[0]->var.yres - pos_y;
	else if (var->yres < 0x20)
		var->yres = 0x20;

	if (var->xres > fbi->hw->info[0]->var.xres - pos_x)
		var->xres = fbi->hw->info[0]->var.xres - pos_x;
	else if (var->xres < 0x20)
		var->xres = 0x20;

	var->xres &= ~(0x1f);
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;
	/* should we report EINVAL */
	var->xres_virtual = var->xres;
	if (var->xoffset + var->xres > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yoffset + var->yres > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;
	/* hack : the controller support 24 bit mode, but it
	 * look like 32 bit in memory
	 */
	if (var->bits_per_pixel == 24)
		var->bits_per_pixel = 32;

	real_bits_per_pixel = var->bits_per_pixel;

	if (var->nonstd != NOSTD_RGB) {
		var->bits_per_pixel = 8;
		real_bits_per_pixel = 12;
	}
	else {
		/* validate bpp for rgb*/
		if (var->bits_per_pixel != 16 && var->bits_per_pixel != 32)
			return -EINVAL;
	}

	/* XXX we don't check the enable bit */
	osd = __raw_readl(base+P6_LCDC_OSD_CTL);
	osd &= P6_LCDC_OSD_CTL_XPIXEL(plane);
	if (osd && var->bits_per_pixel != 32)
		real_bits_per_pixel += 8;

	if (ROUNDUP(var->yres_virtual * var->xres_virtual *
			real_bits_per_pixel , 8) > fbi->fb->fix.smem_len)
		return -EINVAL;

	/* get timing of plane 0 */
	var->pixclock     = fbi->hw->info[0]->var.pixclock;

	var->upper_margin = fbi->hw->info[0]->var.upper_margin;
	var->lower_margin = fbi->hw->info[0]->var.lower_margin;
	var->vsync_len    = fbi->hw->info[0]->var.vsync_len;

	var->left_margin  = fbi->hw->info[0]->var.left_margin;
	var->right_margin = fbi->hw->info[0]->var.right_margin;
	var->hsync_len    = fbi->hw->info[0]->var.hsync_len;

	dprintk("%s: ok\n", __func__);
	dprintk("%s: var->xres  = %d\n", __func__, var->xres);
	dprintk("%s: var->yres  = %d\n", __func__, var->yres);

	/* set r/g/b positions */
	switch (var->bits_per_pixel) {
		case 16:
			/* 16 bpp, 5551 format */
			var->red.offset		= 10;
			var->green.offset	= 5;
			var->blue.offset	= 0;
			var->red.length		= 5;
			var->green.length	= 5;
			var->blue.length	= 5;
			var->transp.length	= 0;
			break;
		case 32:
			/* 24 bpp 888 */
			var->transp.length	= 8;
			var->transp.offset	= 24;
			var->red.length		= 8;
			var->red.offset		= 16;
			var->green.length	= 8;
			var->green.offset	= 8;
			var->blue.length	= 8;
			var->blue.offset	= 0;
			break;
		default:
			/* ycc */
			var->transp.length	= 0;
			var->transp.offset	= 0;
			var->red.length		= 0;
			var->red.offset		= 0;
			var->green.length	= 0;
			var->green.offset	= 0;
			var->blue.length	= 0;
			var->blue.offset	= 0;
	}

	var->activate &= ~FB_ACTIVATE_FORCE;
	return 0;
}

static int p6fb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	struct p6fb_info *fbi = info->par;

	if (fbi->plane != 0)
		return p6fb_ycc_check_var(var, info);

	dprintk("check_var(var=%p, info=%p)\n", var, info);

	dprintk("%s: var->xres  = %d\n", __func__, var->xres);
	dprintk("%s: var->yres  = %d\n", __func__, var->yres);
	dprintk("%s: var->xres_v= %d\n", __func__, var->xres_virtual);
	dprintk("%s: var->yres_v= %d\n", __func__, var->yres_virtual);
	dprintk("%s: var->bpp   = %d\n", __func__, var->bits_per_pixel);

	/* validate pixel clock */
	if (var->pixclock > fbi->hw->mach_info->pixclock.max)
		var->pixclock = fbi->hw->mach_info->pixclock.max;
	else if (var->pixclock < fbi->hw->mach_info->pixclock.min)
		var->pixclock = fbi->hw->mach_info->pixclock.min;

	var->pixclock = p6fb_get_pixclk(fbi, var);
	/* validate x/y resolution */

	var->xoffset = 0;

	if (var->yres > fbi->hw->mach_info->yres.max)
		var->yres = fbi->hw->mach_info->yres.max;
	else if (var->yres < fbi->hw->mach_info->yres.min)
		var->yres = fbi->hw->mach_info->yres.min;

	if (var->xres > fbi->hw->mach_info->xres.max)
		var->xres = fbi->hw->mach_info->xres.max;
	else if (var->xres < fbi->hw->mach_info->xres.min)
		var->xres = fbi->hw->mach_info->xres.min;

	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;
	/* should we report EINVAL */
	var->xres_virtual = var->xres;
	if (var->xoffset + var->xres > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yoffset + var->yres > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;
	/* validate bpp */

	if (var->bits_per_pixel > fbi->hw->mach_info->bpp.max)
		var->bits_per_pixel = fbi->hw->mach_info->bpp.max;
	else if (var->bits_per_pixel < fbi->hw->mach_info->bpp.min)
		var->bits_per_pixel = fbi->hw->mach_info->bpp.min;
	/* hack : the controller support 24 bit mode, but it
	 * look like 32 bit in memory
	 */
	if (var->bits_per_pixel == 24)
		var->bits_per_pixel = 32;

	if (var->yres_virtual * var->xres_virtual *
			var->bits_per_pixel / 8 > fbi->fb->fix.smem_len)
		return -EINVAL;

	dprintk("%s: ok\n", __func__);
	dprintk("%s: var->xres  = %d\n", __func__, var->xres);
	dprintk("%s: var->yres  = %d\n", __func__, var->yres);

	/* set r/g/b positions */
	switch (var->bits_per_pixel) {
		default:
		case 16:
			/* 16 bpp, 5551 format */
			var->red.offset		= 10;
			var->green.offset	= 5;
			var->blue.offset	= 0;
			var->red.length		= 5;
			var->green.length	= 5;
			var->blue.length	= 5;
			var->transp.length	= 0;
			break;
		case 32:
			/* 24 bpp 888 */
			var->red.length		= 8;
			var->red.offset		= 16;
			var->green.length	= 8;
			var->green.offset	= 8;
			var->blue.length	= 8;
			var->blue.offset	= 0;
			var->transp.length	= 0;
			break;
	}
	var->activate &= ~FB_ACTIVATE_FORCE;
	return 0;
}

/* p6fb_activate_var
 *
 * activate (set) the controller from the given framebuffer
 * information
*/

static void p6fb_activate_var(struct p6fb_info *fbi,
				   struct fb_var_screeninfo *var)
{
	void __iomem *base = fbi->hw->base;
	unsigned int ctrl, rgb;
	u32 fifo_threshold;
	static int init;

	dprintk("%s: var->xres  = %d\n", __func__, var->xres);
	dprintk("%s: var->yres  = %d\n", __func__, var->yres);
	dprintk("%s: var->bpp   = %d\n", __func__, var->bits_per_pixel);

	ctrl = __raw_readl(base+P6_LCDC_CTRL);

	rgb = __raw_readl(base+P6_LCDC_RGB_CTL);
	if (init == 0 && (rgb & P6_LCDC_RGB_CTL_UNIFILL))
	{
		if (var->bits_per_pixel == 32) {
			int i;
			for (i = 0; i < fbi->fb->fix.smem_len; i+=4) {
				fbi->map_cpu[i] = rgb & 0xffffff;
			}
		}
		else {
#define R32(color) ((color >> (16+3)) & 0x1f)
#define G32(color) ((color >>  (8+3)) & 0x1f)
#define B32(color) ((color >>  (0+3)) & 0x1f)
			int val = (R32(rgb) << 10) | (G32(rgb) << 5) | B32(rgb);
			memset(fbi->map_cpu, val, fbi->fb->fix.smem_len);
#undef R32
#undef G32
#undef B32
		}
		__raw_writel(0, base+P6_LCDC_RGB_CTL);
	}
	/* we only change timing if the controller is off,
	 * because it is not simple to turn off the controller ...
	 *
	 * TODO : we could check if the value need to be updated
	 * and force the reset in this case.
	 * But in real usage it doesn't make sense to change these
	 * parameters once the controller is started
	 */
	if ((ctrl & P6_LCDC_CTRL_OUTEN) == 0 && init == 0) {
		init = 1;

		/* set timming */
		dprintk("setting vert: up=%d, low=%d, sync=%d\n",
				var->upper_margin, var->lower_margin,
				var->vsync_len);

		var->vsync_len = max(var->vsync_len, 1U);
		var->lower_margin = max(var->lower_margin, 1U);
		var->upper_margin = max(var->upper_margin, 1U);
		BUG_ON(var->vsync_len >= 0xff);
		BUG_ON(var->lower_margin >= 0xff);
		BUG_ON(var->upper_margin >= 0xff);
		__raw_writel((var->vsync_len-1)|
				((var->lower_margin-1)<<8)|
				((var->upper_margin-1)<<16), base+P6_LCDC_VTR);

		dprintk("setting horz: lft=%d, rt=%d, sync=%d\n",
				var->left_margin, var->right_margin,
				var->hsync_len);
		var->hsync_len = max(var->hsync_len, 1U);
		var->right_margin = max(var->right_margin, 1U);
		var->left_margin = max(var->left_margin, 1U);
		BUG_ON(var->hsync_len >= 0xff);
		BUG_ON(var->right_margin >= 0xff);
		BUG_ON(var->left_margin >= 0xff);
		__raw_writel((var->hsync_len-1)|
				((var->right_margin-1)<<8)|
				((var->left_margin-1)<<16), base+P6_LCDC_HTR);

		/* set a threshold of 3/4 of fifo_size (256) */
		fifo_threshold = __raw_readl(base+P6_LCDC_FIFO);
		p6fb_bit_set(fifo_threshold, 0, 0xff, 0xc0);
		__raw_writel(fifo_threshold, base+P6_LCDC_FIFO);

		__raw_writel(fbi->hw->mach_info->panel, base+P6_LCDC_PCR);

		__raw_writel(P6_LCDC_DMA_INCR16, base+P6_LCDC_DMA);

		/* update X/Y info */
		BUG_ON(var->xres > 0x7ff);
		BUG_ON(var->yres > 0x7ff);
		__raw_writel(var->yres<<16|var->xres, base+P6_LCDC_PSR);

		/* program pixel clock */
		if (var->pixclock > 0) {
			int clkdiv = p6fb_calc_clkdiv(fbi, var);
			unsigned int pcr = __raw_readl(base+P6_LCDC_PCR);
			p6fb_bit_set(pcr, 0, P6_LCDC_PCR_MSK, clkdiv);
			__raw_writel(pcr, base+P6_LCDC_PCR);
		}
		if (machine_is_rnb4()) {
			__raw_writel(var->xres<<16|var->yres, base+P6_LCDC_PSR);
		}
	}
	/* rnb4 hack */
	if (machine_is_rnb4() && !(var->activate & FB_ACTIVATE_VBL)) {
		int plane = 1;
		u32 size, fifo_size, fifo_threshold;

		stop_plane(fbi, plane);

		/* position */
		__raw_writel(0, base+P6_LCDC_YCC1_P+(plane-1)*4);
		/* conf plan : rgb16 */
		p6fb_bit_set(ctrl, 0, P6_LCDC_CTRL_YCCX_RGB(plane)|P6_LCDC_CTRL_YCCX_RGB24(plane), P6_LCDC_CTRL_YCCX_RGB(plane));

		/* taille */
		size = __raw_readl(base+P6_LCDC_YCC1_S+(plane-1)*4);
		p6fb_bit_set(size, 0, 0x7ff, var->yres);
		p6fb_bit_set(size, 16, 0x7ff, var->xres);
		__raw_writel(size, base+P6_LCDC_YCC1_S+(plane-1)*4);


		/* fifo */
		fifo_size = p6fb_fifo_threshold(var, plane);
		fifo_threshold = __raw_readl(base+P6_LCDC_FIFO);
		p6fb_bit_set(fifo_threshold, 8*plane, 0xff, fifo_size);
		__raw_writel(fifo_threshold, base+P6_LCDC_FIFO);

		/* manual switch buffer */
		__raw_writel(0|P6_LCDC_YCC1_SRC, base+P6_LCDC_YCC1_BUFFER+(plane-1)*4);
		/* our osd buffer got a fixed size */
		BUG_ON(var->xres != 400);
		BUG_ON(var->yres != 240);
		/* OSD adress (triangle) */
		__raw_writel(rnb4_mask_dma, base+P6_LCDC_OSD1_A1+(plane-1)*4*12);
	}

	/* HACK Android bug workaround */
	if (!(var->activate & FB_ACTIVATE_VBL)) {
		/* XXX fifo bug */
		stop_plane(fbi, 0);

		fifo_threshold = __raw_readl(base+P6_LCDC_FIFO);

		p6fb_bit_set(fifo_threshold, 0, 0xff, p6fb_fifo_threshold(var, 0));

		__raw_writel(fifo_threshold, base+P6_LCDC_FIFO);

		ctrl |= P6_LCDC_CTRL_RGB_EN;
		ctrl &= ~P6_LCDC_CTRL_RGB_24;
		switch (var->bits_per_pixel) {
		case 16:
			break;
		case 32:
			ctrl |= P6_LCDC_CTRL_RGB_24;
			break;
		default:
			/* invalid pixel depth */
			dev_err(fbi->hw->dev, "invalid bpp %d\n", var->bits_per_pixel);
		}
	}

	/* set lcd address pointers */
	p6fb_set_lcdaddr(var, fbi);

	/* enable lcd controller */
	if (ctrl & P6_LCDC_CTRL_OUTEN)
		vprintk("enabling lcd\n");
	__raw_writel(ctrl|P6_LCDC_CTRL_OUTEN, base+P6_LCDC_CTRL);
	if (var->activate & FB_ACTIVATE_VBL) {
		p6fb_wait_vbl(fbi);
	}
}

static void p6fb_ycc_activate_var(struct p6fb_info *fbi,
				   struct fb_var_screeninfo *var)
{
	struct p6fb_hw *hw = fbi->hw;
	void __iomem *base = hw->base;
	unsigned int ctrl, new_ctrl = 0;
	u32 fifo_threshold, size;
	int plane = fbi->plane;
	unsigned long flags;
	int fifo_size;

	dprintk("%s: var->xres  = %d\n", __func__, var->xres);
	dprintk("%s: var->yres  = %d\n", __func__, var->yres);
	dprintk("%s: var->bpp   = %d\n", __func__, var->bits_per_pixel);

#if 0
	ycc yuvtorgb
	 from mpeg play
		CB -= 128; CR -= 128;
		  }
	/* was
		  Cr_r_tab[i] =  1.596 * CR;
		  Cr_g_tab[i] = -0.813 * CR;
		  Cb_g_tab[i] = -0.391 * CB;
		  Cb_b_tab[i] =  2.018 * CB;
	  but they were just messed up.
	  Then was (_Video Deymstified_):
		  Cr_r_tab[i] =  1.366 * CR;
		  Cr_g_tab[i] = -0.700 * CR;
		  Cb_g_tab[i] = -0.334 * CB;
		  Cb_b_tab[i] =  1.732 * CB;
	  but really should be:
	   (from ITU-R BT.470-2 System B, G and SMPTE 170M )
	*/
		  Cr_r_tab[i] =  (0.419/0.299) * CR;
		  Cr_g_tab[i] = -(0.299/0.419) * CR;
		  Cb_g_tab[i] = -(0.114/0.331) * CB;
		  Cb_b_tab[i] =  (0.587/0.331) * CB;

	/*
	  though you could argue for:
		SMPTE 240M
		  Cr_r_tab[i] =  (0.445/0.212) * CR;
		  Cr_g_tab[i] = -(0.212/0.445) * CR;
		  Cb_g_tab[i] = -(0.087/0.384) * CB;
		  Cb_b_tab[i] =  (0.701/0.384) * CB;
		FCC
		  Cr_r_tab[i] =  (0.421/0.30) * CR;
		  Cr_g_tab[i] = -(0.30/0.421) * CR;
		  Cb_g_tab[i] = -(0.11/0.331) * CB;
		  Cb_b_tab[i] =  (0.59/0.331) * CB;
		ITU-R BT.709
		  Cr_r_tab[i] =  (0.454/0.2125) * CR;
		  Cr_g_tab[i] = -(0.2125/0.454) * CR;
		  Cb_g_tab[i] = -(0.0721/0.386) * CB;
		  Cb_b_tab[i] =  (0.7154/0.386) * CB;
	*/
#endif
	/* XXX do we need to desactivate this plane for configuration ??? */
	__raw_writel(0x1a4b0b36, base+P6_LCDC_YCC1TORGB_CR+(plane-1)*8);
	__raw_writel(0x0e301d3f, base+P6_LCDC_YCC1TORGB_CB+(plane-1)*8);

	/* update X/Y info */
	BUG_ON(var->xres & 0x1f);
	size = __raw_readl(base+P6_LCDC_YCC1_S+(plane-1)*4);
	p6fb_bit_set(size, 0, 0x7ff, var->xres);
	p6fb_bit_set(size, 16, 0x7ff, var->yres);
	__raw_writel(size, base+P6_LCDC_YCC1_S+(plane-1)*4);

	stop_plane(fbi, plane);

	if (var->nonstd == NOSTD_RGB) {
		new_ctrl |= P6_LCDC_CTRL_YCCX_RGB(plane);
		if (var->bits_per_pixel == 32) {
			new_ctrl |= P6_LCDC_CTRL_YCCX_RGB24(plane);
		}
		fifo_size = p6fb_fifo_threshold(var, plane);
	}
	else {
		fifo_size = 64;
		fifo_size = fifo_size * 3 / 4;
	}

	/* don't touch osd */
#if 0
	osd = __raw_readl(base+P6_LCDC_OSD_CTL);
	osd &= ~(P6_LCDC_OSD_CTL_X_EN(plane)|P6_LCDC_OSD_CTL_XPIXEL(plane));
	if (var->nonstd & NOSTD_ALPHA) {
		osd |= P6_LCDC_OSD_CTL_X_EN(plane)|P6_LCDC_OSD_CTL_XPIXEL(plane);
	}
	__raw_writel(osd, base+P6_LCDC_OSD_CTL);
#endif

	/* set lcd address pointers */
	p6fb_set_lcdaddr(var, fbi);

	local_irq_save(flags);
	fifo_threshold = __raw_readl(base+P6_LCDC_FIFO);
	p6fb_bit_set(fifo_threshold, 8*plane, 0xff, fifo_size);
	__raw_writel(fifo_threshold, base+P6_LCDC_FIFO);

	ctrl = __raw_readl(base+P6_LCDC_CTRL);
	ctrl |= P6_LCDC_CTRL_YCCX_EN(plane);
	p6fb_bit_set(ctrl, 0, P6_LCDC_CTRL_YCCX_RGB(plane)|P6_LCDC_CTRL_YCCX_RGB24(plane), new_ctrl);
	/* enable lcd controller */
	__raw_writel(ctrl, base+P6_LCDC_CTRL);
	local_irq_restore(flags);
}

/*
 *      p6fb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 */
static int p6fb_set_par(struct fb_info *info)
{
	struct p6fb_info *fbi = info->par;
	struct fb_var_screeninfo *var = &info->var;

	switch (var->bits_per_pixel) {
		case 32:
		case 16:
			fbi->fb->fix.visual = FB_VISUAL_TRUECOLOR;
			break;
		case 1:
			fbi->fb->fix.visual = FB_VISUAL_MONO01;
			 break;
		default:
			fbi->fb->fix.visual = FB_VISUAL_PSEUDOCOLOR;
			 break;
	}

	fbi->fb->fix.line_length = (var->xres_virtual*var->bits_per_pixel)/8;

	/* activate this new configuration */

	if (fbi->plane == 0)
		p6fb_activate_var(fbi, var);
	else
		p6fb_ycc_activate_var(fbi, var);
	return 0;
}

/* from pxafb.c */
static inline unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

/*
 * this is needed for fbcon ...
 * Why can't it do it on its own if it not set
 */
static int p6fb_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	struct p6fb_info *fbi = info->par;
	unsigned int val;

	/* dprintk("setcol: regno=%d, rgb=%d,%d,%d\n", regno, red, green, blue); */

	switch (fbi->fb->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/* true-colour, use pseudo-palette */

		if (regno < 16) {
			u32 *pal = fbi->fb->pseudo_palette;

			val  = chan_to_field(red,   &fbi->fb->var.red);
			val |= chan_to_field(green, &fbi->fb->var.green);
			val |= chan_to_field(blue,  &fbi->fb->var.blue);

			pal[regno] = val;
		}
		break;

	default:
		return 1;   /* unknown type */
	}

	return 0;
}

/**
 *  p6fb_blank (optional)
 *      @blank_mode: the blank mode we want.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      Blank the screen if blank_mode != FB_BLANK_UNBLANK, else unblank.
 *      Return 0 if blanking succeeded, != 0 if un-/blanking failed due to
 *      e.g. a video mode which doesn't support it.
 *
 *      Implements VESA suspend and powerdown modes on hardware that supports
 *      disabling hsync/vsync:
 *
 *      FB_BLANK_NORMAL = display is blanked, syncs are on.
 *      FB_BLANK_HSYNC_SUSPEND = hsync off
 *      FB_BLANK_VSYNC_SUSPEND = vsync off
 *      FB_BLANK_POWERDOWN =  hsync and vsync off
 *
 *      If implementing this function, at least support FB_BLANK_UNBLANK.
 *      Return !0 for any modes that are unimplemented.
 *
 */
static int p6fb_ycc_blank(int blank_mode, struct fb_info *info)
{
	struct p6fb_info *fbi = info->par;
	struct p6fb_hw *hw = fbi->hw;
	void __iomem *base = hw->base;
	unsigned long ctl, fifo;
	unsigned long flags;

	/* if fifo is not configured, assume we did not configured yet the plane */
	fifo = __raw_readl(base+P6_LCDC_FIFO);
	fifo = (fifo>>(8*fbi->plane)) & 0xff;
	if (fifo == 0)
		return -EINVAL;

	local_irq_save(flags);
	ctl = __raw_readl(base+P6_LCDC_CTRL);
	if (blank_mode == FB_BLANK_UNBLANK)
		/* desactivate plane */
		ctl |= P6_LCDC_CTRL_YCCX_EN(fbi->plane);
	else {
		ctl &= ~P6_LCDC_CTRL_YCCX_EN(fbi->plane);
	}
	__raw_writel(ctl, base+P6_LCDC_CTRL);
	local_irq_restore(flags);

	return 0;
}

static int p6fb_blank(int blank_mode, struct fb_info *info)
{
	struct p6fb_info *fbi = info->par;
	void __iomem *base = fbi->hw->base;
	dprintk("blank(mode=%d, info=%p)\n", blank_mode, info);

	if (fbi->plane != 0)
		return p6fb_ycc_blank(blank_mode, info);

	if (blank_mode == FB_BLANK_UNBLANK)
		__raw_writel(0, base+P6_LCDC_RGB_CTL);
	else {
		/* fill the screen with black */
		__raw_writel(P6_LCDC_RGB_CTL_UNIFILL|0, base+P6_LCDC_RGB_CTL);
	}

	return 0;
}
static void p6fb_get_vbl(struct p6fb_info *fbi,
		struct fb_vblank *vblank)
{
	void __iomem *base = fbi->hw->base;
	int status = __raw_readl(base+P6_LCDC_STAT);
	memset(vblank, 0, sizeof(*vblank));
	vblank->flags = FB_VBLANK_HAVE_VBLANK|
					FB_VBLANK_HAVE_HBLANK|
					FB_VBLANK_HAVE_VSYNC;
	if ((status & P6_LCDC_STAT_HACTD) == 0)
		vblank->flags |= FB_VBLANK_HBLANKING;
	if ((status & P6_LCDC_STAT_VACTD) == 0)
		vblank->flags |= FB_VBLANK_VBLANKING;
	if (status & P6_LCDC_STAT_VSYNC)
		vblank->flags |= FB_VBLANK_VSYNCING;

	vblank->flags |= FB_VBLANK_HAVE_COUNT;
	vblank->count = fbi->hw->vbl_cnt;
}

static int p6fb_set_rgb_ctl(struct p6fb_info *fbi, struct p6fb_rgb_ctl *rgb_ctl)
{
	void __iomem *base = fbi->hw->base;
	if (fbi->plane != 0)
		return -EINVAL;
	__raw_writel(rgb_ctl->blue|(rgb_ctl->green<<8)|(rgb_ctl->red<<16)|(rgb_ctl->mode<<24), base+P6_LCDC_RGB_CTL);
	return 0;
}

static int p6fb_get_rgb_ctl(struct p6fb_info *fbi, struct p6fb_rgb_ctl *rgb_ctl)
{
	void __iomem *base = fbi->hw->base;
	u32 rgb;
	if (fbi->plane != 0)
		return -EINVAL;
	rgb = __raw_readl(base+P6_LCDC_RGB_CTL);

	rgb_ctl->blue = rgb & 0xff;
	rgb_ctl->green = (rgb >> 8) & 0xff;
	rgb_ctl->red = (rgb >> 16) & 0xff;
	rgb_ctl->mode = (rgb >> 24) & 0xff;
	return 0;
}

static int p6fb_setup_plane(struct p6fb_info *fbi, struct p6fb_plane_info *plane_info)
{
	struct fb_info *info = fbi->fb;
	struct p6fb_hw *hw = fbi->hw;
	void __iomem *base = hw->base;
	int plane = fbi->plane;
	u32 osd, size;
	unsigned long flags;

	if (fbi->plane == 0)
		return -EINVAL;
	/* don't got outside of the screen */
	if (info->var.xres + plane_info->pos_x > fbi->hw->info[0]->var.xres)
		return -EINVAL;
	if (info->var.yres + plane_info->pos_y > fbi->hw->info[0]->var.yres)
		return -EINVAL;

	/* check if we got enought buffer for osd */
	if (plane_info->osdmode == P6FB_OSDMODE_PIXEL) {
		int real_bits_per_pixel = info->var.bits_per_pixel;
		if (real_bits_per_pixel == 8)
			real_bits_per_pixel = 12;
		real_bits_per_pixel += 8;
		if (ROUNDUP(info->var.yres_virtual * info->var.xres_virtual *
				real_bits_per_pixel , 8) > fbi->fb->fix.smem_len)
			return -EINVAL;
	}

	BUG_ON(plane_info->pos_x > 0x7ff);
	BUG_ON(plane_info->pos_y > 0x7ff);
	__raw_writel(plane_info->pos_y<<16|plane_info->pos_x, base+P6_LCDC_YCC1_P+(plane-1)*4);

	local_irq_save(flags);

	osd = __raw_readl(base+P6_LCDC_OSD_CTL);
	size = __raw_readl(base+P6_LCDC_YCC1_S+(plane-1)*4);

	size = size & ~(P6_LCDC_YCC1_S_DS4|P6_LCDC_YCC1_S_DS16);
	if (plane_info->scale == 1)
		size |= P6_LCDC_YCC1_S_DS4;
	else if (plane_info->scale == 2)
		size |= P6_LCDC_YCC1_S_DS16;

	p6fb_bit_set(osd, 8*(plane-1), 0xff, plane_info->osdplane);

	osd &= ~(P6_LCDC_OSD_CTL_XPIXEL(plane)|P6_LCDC_OSD_CTL_X_EN(plane));

	if (plane_info->osdmode == P6FB_OSDMODE_PLANE) {
		osd |= P6_LCDC_OSD_CTL_X_EN(plane);
	}
	else if (plane_info->osdmode == P6FB_OSDMODE_PIXEL) {
		osd |= P6_LCDC_OSD_CTL_XPIXEL(plane)|P6_LCDC_OSD_CTL_X_EN(plane);
	}

	__raw_writel(size, base+P6_LCDC_YCC1_S+(plane-1)*4);
	__raw_writel(osd, base+P6_LCDC_OSD_CTL);

	local_irq_restore(flags);
	return 0;
}

static int p6fb_query_plane(struct p6fb_info *fbi, struct p6fb_plane_info *plane_info)
{
	struct p6fb_hw *hw = fbi->hw;
	void __iomem *base = hw->base;
	int plane = fbi->plane;
	u32 osd, size, pos;

	if (fbi->plane == 0)
		return -EINVAL;

	pos = __raw_readl(base+P6_LCDC_YCC1_P+(plane-1)*4);
	size = __raw_readl(base+P6_LCDC_YCC1_S+(plane-1)*4);
	osd = __raw_readl(base+P6_LCDC_OSD_CTL);

	plane_info->pos_y = pos >> 16;
	plane_info->pos_x = pos & 0xff;

	plane_info->scale = size >> 28;

	plane_info->osdplane = (osd >> (8*(plane-1))) & 0xff;
	if (osd & P6_LCDC_OSD_CTL_XPIXEL(plane))
		plane_info->osdmode = P6FB_OSDMODE_PIXEL;
	else if (osd & P6_LCDC_OSD_CTL_X_EN(plane))
		plane_info->osdmode = P6FB_OSDMODE_PLANE;
	else
		plane_info->osdmode = P6FB_OSDMODE_NONE;

	return 0;
}

static int p6fb_set_ycc_to_rgb(struct p6fb_info *fbi, struct p6fb_ycc_to_rgb *ycc_to_rgb)
{
	void __iomem *base = fbi->hw->base;
	if (fbi->plane == 0)
		return -EINVAL;
	__raw_writel(ycc_to_rgb->cr, base+P6_LCDC_YCC1TORGB_CR+(fbi->plane-1)*8);
	__raw_writel(ycc_to_rgb->cb, base+P6_LCDC_YCC1TORGB_CB+(fbi->plane-1)*8);
	return 0;
}

static int p6fb_get_ycc_to_rgb(struct p6fb_info *fbi, struct p6fb_ycc_to_rgb *ycc_to_rgb)
{
	void __iomem *base = fbi->hw->base;
	if (fbi->plane == 0)
		return -EINVAL;
	ycc_to_rgb->cr = __raw_readl(base+P6_LCDC_YCC1TORGB_CR+(fbi->plane-1)*8);
	ycc_to_rgb->cb = __raw_readl(base+P6_LCDC_YCC1TORGB_CB+(fbi->plane-1)*8);
	return 0;
}


/* fb_pan_display
 * Pan display in x and/or y as specified
 */
static int p6fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct p6fb_info *fbi = info->par;
	dprintk("p6fb_pan_display\n");
	p6fb_set_lcdaddr(var, fbi);
	return 0;
}

/* fb_ioctl
 * Wait for vbl signal
 */
static int p6fb_ioctl(struct fb_info *info,
			  unsigned int cmd, unsigned long arg)
{
	struct p6fb_info *fbi = info->par;
	void __user *argp = (void __user *)arg;
	int ret = 0;

	dprintk("p6fb_ioctl %x %p\n", cmd, argp);
	switch (cmd) {
		case FBIOPAN_DISPLAY:
			ret = p6fb_wait_vbl(fbi)==0 ? -EBUSY : 0;
			break;
		case FBIOGET_VBLANK:
		{
			struct fb_vblank vblank;
			p6fb_get_vbl(fbi, &vblank);
			if (copy_to_user(argp, &vblank, sizeof(vblank)))
				ret = -EFAULT;
			break;
		}
		case P6FB_SET_RGB_CTL:
		{
			struct p6fb_rgb_ctl rgb_ctl;
			if (copy_from_user(&rgb_ctl, argp, sizeof(rgb_ctl)))
				ret = -EFAULT;
			else
				ret = p6fb_set_rgb_ctl(fbi, &rgb_ctl);
			break;
		}
		case P6FB_GET_RGB_CTL:
		{
			struct p6fb_rgb_ctl rgb_ctl;
			ret = p6fb_get_rgb_ctl(fbi, &rgb_ctl);
			if (ret == 0 && copy_to_user(argp, &rgb_ctl, sizeof(rgb_ctl)))
				ret = -EFAULT;
			break;
		}
		case P6FB_QUERY_PLANE:
		{
			struct p6fb_plane_info plane_info;
			ret = p6fb_query_plane(fbi, &plane_info);
			if (ret == 0 && copy_to_user(argp, &plane_info, sizeof(plane_info)))
				ret = -EFAULT;
			break;
		}
		case P6FB_SETUP_PLANE:
		{
			struct p6fb_plane_info plane_info;
			if (copy_from_user(&plane_info, argp, sizeof(plane_info)))
				ret = -EFAULT;
			else
				ret = p6fb_setup_plane(fbi, &plane_info);
			break;
		}
		case P6FB_SET_YCC_TO_RGB:
		{
			struct p6fb_ycc_to_rgb ycc_to_rgb;
			if (copy_from_user(&ycc_to_rgb, argp, sizeof(ycc_to_rgb)))
				ret = -EFAULT;
			else
				ret = p6fb_set_ycc_to_rgb(fbi, &ycc_to_rgb);
			break;
		}
		case P6FB_GET_YCC_TO_RGB:
		{
			struct p6fb_ycc_to_rgb ycc_to_rgb;
			ret = p6fb_get_ycc_to_rgb(fbi, &ycc_to_rgb);
			if (ret == 0 && copy_to_user(argp, &ycc_to_rgb, sizeof(ycc_to_rgb)))
				ret = -EFAULT;
			break;
		}
		case P6FB_GET_FBINFO:
		{
			struct p6fb_fbinfo fbinfo;
			fbinfo.map_dma = fbi->map_dma;
			if (copy_to_user(argp, &fbinfo, sizeof(fbinfo)))
				ret = -EFAULT;
			break;
		}

		default:
			ret = -EINVAL;
	}

	return ret;
}

static int p6fb_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", debug ? "on" : "off");
}
static int p6fb_debug_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t len)
{
	if (mach_info == NULL)
		return -EINVAL;

	if (len < 1)
		return -EINVAL;

	if (strnicmp(buf, "on", 2) == 0 ||
	    strnicmp(buf, "1", 1) == 0) {
		debug = 1;
		printk(KERN_DEBUG "p6fb: Debug On");
	} else if (strnicmp(buf, "off", 3) == 0 ||
		   strnicmp(buf, "0", 1) == 0) {
		debug = 0;
		printk(KERN_DEBUG "p6fb: Debug Off");
	} else {
		return -EINVAL;
	}

	return len;
}

static DEVICE_ATTR(debug, 0666,
		   p6fb_debug_show,
		   p6fb_debug_store);

static struct fb_ops p6fb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= p6fb_check_var,
	.fb_set_par	= p6fb_set_par,
	.fb_setcolreg	= p6fb_setcolreg,
	.fb_blank	= p6fb_blank,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_pan_display = p6fb_pan_display,
	.fb_ioctl = p6fb_ioctl,
};

/*
 * p6fb_map_video_memory():
 *	Allocates the DRAM memory for the frame buffer.  This buffer is
 *	remapped into a non-cached, non-buffered, memory region to
 *	allow palette and pixel writes to occur without flushing the
 *	cache.  Once this area is remapped, all virtual memory
 *	access to the video memory should occur at the new region.
 */
static int __init p6fb_map_video_memory(struct fb_info *info)
{
	struct p6fb_info *fbi = info->par;
	struct p6fb_hw *hw = fbi->hw;
	dprintk("map_video_memory(fbi=%p)\n", fbi);

	fbi->map_size = PAGE_ALIGN(fbi->fb->fix.smem_len + PAGE_SIZE);

	if (hw->mach_info->map_base && fbi->plane == 0) {
		printk(KERN_DEBUG "using ram allocated by bootloader @%p-0x%lx\n",
				hw->mach_info->map_base,
				(long)(hw->mach_info->map_base) + hw->mach_info->map_size);
		fbi->map_dma = (dma_addr_t)hw->mach_info->map_base;
		if (fbi->map_size <= hw->mach_info->map_size)
			fbi->map_cpu  = ioremap_nocache(fbi->map_dma, fbi->map_size);
		else
			fbi->map_cpu  = NULL;
	}
	else
	{
		fbi->map_cpu  = dma_alloc_writecombine(hw->dev, fbi->map_size,
					       &fbi->map_dma, GFP_KERNEL);

		/* prevent initial garbage on screen */
		dprintk("map_video_memory: clear %p:%08x\n",
			fbi->map_cpu, fbi->map_size);
		if (fbi->map_cpu)
			memset(fbi->map_cpu, 0x1f, fbi->fb->fix.smem_len);
	}
	/* XXX what to do if bps doesn't match with bootloader ??? */

	if (fbi->map_cpu) {
		fbi->fb->screen_base	= fbi->map_cpu;
		fbi->fb->fix.smem_start  = fbi->map_dma;

		dprintk("map_video_memory: dma=%08x cpu=%p size=%08x\n",
			fbi->map_dma, fbi->map_cpu, fbi->fb->fix.smem_len);
	}

	return fbi->map_cpu ? 0 : -ENOMEM;
}

static inline void p6fb_unmap_video_memory(struct fb_info *info)
{
	struct p6fb_info *fbi = info->par;
	struct p6fb_hw *hw = fbi->hw;
	if (!fbi->map_cpu)
		return;

	if (fbi->hw->mach_info->map_base) {
		iounmap(fbi->map_cpu);
	}
	else {
		dma_free_writecombine(hw->dev, fbi->map_size, fbi->map_cpu, fbi->map_dma);
	}
}

static void p6fb_reset_timer(unsigned long data)
{
	struct p6fb_hw *hw = (struct p6fb_hw *)data;
	void __iomem *base = hw->base;
	unsigned long flags;
	u32 ctl;

	local_irq_save(flags);
	dev_err(hw->dev, "reset : timer\n");
	ctl = __raw_readl(base + P6_LCDC_CTRL);
	/* restart the lcd if we stop it for reseting it
	 XXX this is ugly...
	 */
	if (/*(ctl & P6_LCDC_CTRL_OUTEN) == 0*/1) {
		dev_err(hw->dev, "reset : lcd restart\n");
		/* enable interrupt */
		__raw_writel(P6_LCDC_ITEN_ALL, hw->base + P6_LCDC_ITEN);
		/* enable output */
		__raw_writel(ctl|P6_LCDC_CTRL_OUTEN, base + P6_LCDC_CTRL);
	}
	local_irq_restore(flags);
}

static void p6fb_reset(struct p6fb_hw *hw)
{
	void __iomem *base = hw->base;
	u32 ctl = __raw_readl(base + P6_LCDC_CTRL);
	if (ctl & P6_LCDC_CTRL_OUTEN) {
		__raw_writel(ctl & ~(P6_LCDC_CTRL_OUTEN), base + P6_LCDC_CTRL);
		/* avoid underflow IT storm */
		__raw_writel(0, base + P6_LCDC_ITEN);
		/* TODO tune the timer expiration */
		mod_timer(&hw->timer, jiffies + HZ);
		dev_err(hw->dev, "reset : lcd stop\n");
	}
}

static irqreturn_t p6fb_irq(int irq, void *dev_id)
{
	struct p6fb_hw *hw = dev_id;
	void __iomem *base = hw->base;
	int status = __raw_readl(base + P6_LCDC_STAT);
	if (status & P6_LCDC_STAT_UNDF) {
		if (printk_ratelimit())
			dev_err(hw->dev, "lcd underflow : 0x%08x\n", __raw_readl(base+P6_LCDC_CTRL));
		p6fb_reset(hw);
		__raw_writel(P6_LCDC_ITACK_UND, base + P6_LCDC_ITACK);
	}
	if (status & P6_LCDC_STAT_ERR) {
		if (printk_ratelimit())
			dev_err(hw->dev, "lcd error ctrl : 0x%08x\n", __raw_readl(base+P6_LCDC_CTRL));
		p6fb_reset(hw);
		__raw_writel(P6_LCDC_ITACK_ERR, base + P6_LCDC_ITACK);
	}
	if (status & P6_LCDC_STAT_LCD) {
		/* disable osd at next vbl */
		if (machine_is_rnb4())
			__raw_writel(0, base+P6_LCDC_OSD_CTL);
		hw->vbl_cnt++;
		wake_up_interruptible(&hw->vbl_wait);
		__raw_writel(P6_LCDC_ITACK_LCD, base + P6_LCDC_ITACK);
	}

	return IRQ_HANDLED;
}

static char driver_name[] = "p6fb";

struct fb_info *p6fb_plane_init(int num, struct p6fb_hw *hw, struct device *dev)
{
	struct fb_info *info;
	struct p6fb_info *fbi;
	info = framebuffer_alloc(sizeof(struct p6fb_info), dev);
	if (!info) {
		return NULL;
	}

	fbi = info->par;
	fbi->fb = info;
	fbi->hw = hw;
	fbi->plane = num;
	hw->info[num] = info;

	strcpy(info->fix.id, driver_name);

	info->fix.type            = FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux        = 0;
	info->fix.xpanstep        = 0; /* XXX can be supported with DMAOFFSET */
	info->fix.ypanstep        = 1;
	info->fix.ywrapstep       = 0;
	info->fix.accel           = FB_ACCEL_NONE;
	info->fix.visual          = FB_VISUAL_TRUECOLOR;

	info->var.nonstd          = 0;
	info->var.activate        = FB_ACTIVATE_NOW|FB_ACTIVATE_FORCE; /* XXX */
	info->var.height          = -1;
	info->var.width           = -1;
	info->var.accel_flags     = 0;
	info->var.vmode           = FB_VMODE_NONINTERLACED;

	info->fbops	              = &p6fb_ops;
	info->flags	              = FBINFO_FLAG_DEFAULT;
	info->pseudo_palette      = &fbi->pseudo_pal;

	info->var.xres            = mach_info->xres.defval;
	info->var.xres_virtual    = mach_info->xres.defval;
	info->var.yres            = mach_info->yres.defval;
	info->var.yres_virtual    = mach_info->yres.defval;
	info->var.bits_per_pixel  = mach_info->bpp.defval;

	/* timing init from platform data */
	info->var.pixclock        = mach_info->pixclock.defval;

	info->var.upper_margin    = mach_info->upper_margin;
	info->var.lower_margin    = mach_info->lower_margin;
	info->var.vsync_len       = mach_info->vsync_len;

	info->var.left_margin     = mach_info->left_margin;
	info->var.right_margin    = mach_info->right_margin;
	info->var.hsync_len       = mach_info->hsync_len;

	info->fix.smem_len        = mach_info->xres.max *
					mach_info->yres.max *
					mach_info->bpp.max / 8 *
					P6FB_NBR_VIDEO_BUFFERS;
	info->fix.line_length     = (info->var.xres_virtual*info->var.bits_per_pixel)/8;
	if (mach_info->smem_len[num])
		info->fix.smem_len = mach_info->smem_len[num];

	return info;
}

static void p6fb_wq_cleanup(struct p6fb_hw *hw)
{
	hw->vbl_cnt++;
	wake_up_interruptible_all(&hw->vbl_wait);
}

static int __init p6fb_probe(struct platform_device *pdev)
{
	struct p6fb_hw *hw;
	struct resource *res;
	int err;
	int i;

	mach_info = pdev->dev.platform_data;
	if (mach_info == NULL) {
		dev_err(&pdev->dev, "no platform data for lcd, cannot attach\n");
		return -EINVAL;
	}

	hw = kzalloc(sizeof(struct p6fb_hw), GFP_KERNEL);
	if (!hw) {
		return -ENOMEM;
	}
	hw->dev = &pdev->dev;
	hw->mach_info		    = pdev->dev.platform_data;
	platform_set_drvdata(pdev, hw);
	mutex_init(&hw->lock);

	/* hardware init */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		err = -ENOENT;
		goto noregs;
	}

	hw->mem = request_mem_region(res->start,
			res->end-res->start+1,
			pdev->name);

	if (hw->mem == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		err = -ENOENT;
		goto nores;
	}

	hw->base = ioremap(res->start, res->end - res->start + 1);
	if (hw->base == NULL) {
		dev_err(&pdev->dev, "failed ioremap()\n");
		err = -EINVAL;
		goto nomap;
	}

	hw->clk = clk_get(NULL, "lcdc");
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "failed get_clk()\n");
		err = -EINVAL;
		goto noclk;
	}
	clk_enable(hw->clk);

	setup_timer(&hw->timer, p6fb_reset_timer, (unsigned long)hw);
	init_waitqueue_head(&hw->vbl_wait);
	err = request_irq(platform_get_irq(pdev, 0), p6fb_irq,
			IRQF_DISABLED, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "cannot get irq %d - err %d\n", 0, err);
		err = -EBUSY;
		goto noirq;
	}
	/* XXX what to do if the controller is enabled, suppose the
	 * bootloader did the right thing ATM
	 */
	/* set vsync event */
	__raw_writel((__raw_readl(hw->base + P6_LCDC_CTRL) & ~P6_LCDC_CTRL_IT_FRONT) |
			P6_LCDC_CTRL_IT_VSYNC, hw->base + P6_LCDC_CTRL);
	/* enable interrupt */
	__raw_writel(P6_LCDC_ITEN_ALL, hw->base + P6_LCDC_ITEN);
	/* create device files */
	err = device_create_file(&pdev->dev, &dev_attr_debug);
	if (err < 0)
		goto nodevfile;

	for (i = 0; i < P6FB_NBR_VIDEO_PLANE; i++) {
		struct fb_info	   *info;
		info = p6fb_plane_init(i, hw, &pdev->dev);
		if (!info) {
			err = -ENOMEM;
			goto nofb;
		}
		/* Initialize video memory */
		err = p6fb_map_video_memory(info);
		if (err) {
			printk(KERN_ERR "Failed to allocate video RAM: %d\n", err);
			err = -ENOMEM;
			goto nofb;
		}
		dprintk("got video memory\n");

		/* reset register */
		//ret = p6fb_init_registers(fbi);


		/* only activate backgroud plane */
		if (i == 0) {
			if (hw->mach_info->map_base == NULL) {
				err = p6fb_check_var(&info->var, info);
				err = p6fb_set_par(info);
			}
		}
		else {
			/* set default value */
			struct p6fb_plane_info plane_info;
			memset(&plane_info, 0, sizeof(plane_info));
			p6fb_setup_plane(info->par, &plane_info);
		}


		err = register_framebuffer(info);
		if (err < 0) {
			printk(KERN_ERR "Failed to register framebuffer device: %d\n", err);
			goto nofb;
		}
		printk(KERN_INFO "fb%d: %s frame buffer device\n",
				info->node, info->fix.id);
		/* only enable plane 0 for rnb4 */
		if (machine_is_rnb4())
			break;
	}
	if (machine_is_rnb4()) {
		int i;

		//allocate the "osd mask"
		rnb4_mask_cpu  = dma_alloc_coherent(hw->dev, 400*240, &rnb4_mask_dma, GFP_KERNEL);
		if (!rnb4_mask_cpu)
			goto nofb;
		// set the allocation to zero
		memset(rnb4_mask_cpu, 0 , 400*240);
		//rebuild the triangle
		for(i=0; i<240; i++)
			memset(rnb4_mask_cpu+i*400, 0xFF, rnb4_mask_compress[i]);
	}

	return 0;

nofb:
	/* stop lcd */
	__raw_writel(0, hw->base + P6_LCDC_CTRL);
	/* avoid underflow IT storm */
	__raw_writel(0, hw->base + P6_LCDC_ITEN);

	for (i = 0; i < P6FB_NBR_VIDEO_PLANE; i++) {
		struct fb_info	   *info = hw->info[i];
		if (info == NULL)
			continue;
		unregister_framebuffer(info);
		p6fb_unmap_video_memory(info);
		/* free info struct, we can't use fbi struct anymore after that */
		framebuffer_release(info);
	}

	device_remove_file(&pdev->dev, &dev_attr_debug);
nodevfile:
	free_irq(platform_get_irq(pdev, 0), hw);
noirq:
	del_timer_sync(&hw->timer);
	p6fb_wq_cleanup(hw);
	clk_disable(hw->clk);
	clk_put(hw->clk);
noclk:
	iounmap(hw->base);
nomap:
	release_resource(hw->mem);
nores:
noregs:
	mutex_destroy(&hw->lock);
	kfree(hw);
	return err;
}

/*
 *  Cleanup
 */
static int p6fb_remove(struct platform_device *pdev)
{
	struct p6fb_hw	 *hw = platform_get_drvdata(pdev);
	void __iomem *base = hw->base;
	int i;

	/* stop lcd */
	__raw_writel(0, base + P6_LCDC_CTRL);
	/* avoid underflow IT storm */
	__raw_writel(0, base + P6_LCDC_ITEN);

	if (machine_is_rnb4()) {
		dma_free_coherent(hw->dev, 400*240, rnb4_mask_cpu, rnb4_mask_dma);
	}
	for (i = 0; i < P6FB_NBR_VIDEO_PLANE; i++) {
		struct fb_info	   *info = hw->info[i];
		if (info == NULL)
			continue;
		unregister_framebuffer(info);
		p6fb_unmap_video_memory(info);
		/* free info struct, we can't use fbi struct anymore after that */
		framebuffer_release(info);
	}

	device_remove_file(&pdev->dev, &dev_attr_debug);

	free_irq(platform_get_irq(pdev, 0), hw);
	del_timer_sync(&hw->timer);
	p6fb_wq_cleanup(hw);
	clk_disable(hw->clk);
	clk_put(hw->clk);
	iounmap(hw->base);
	release_resource(hw->mem);
	mutex_destroy(&hw->lock);
	kfree(hw);
	return 0;
}

#define p6fb_suspend NULL
#define p6fb_resume  NULL

static struct platform_driver p6fb_driver = {
	.probe		= p6fb_probe,
	.remove		= p6fb_remove,
	.suspend	= p6fb_suspend,
	.resume		= p6fb_resume,
	.driver		= {
		.name	= "p6-lcd",
		.owner	= THIS_MODULE,
	},
};

int __devinit p6fb_init(void)
{
	return platform_driver_register(&p6fb_driver);
}

static void __exit p6fb_cleanup(void)
{
	platform_driver_unregister(&p6fb_driver);
}

module_init(p6fb_init);
module_exit(p6fb_cleanup);

MODULE_AUTHOR("Parrot SA");
MODULE_DESCRIPTION("Framebuffer driver for the parrot6");
MODULE_LICENSE("GPL");
