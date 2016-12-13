/*
 * V4L2 Driver for SuperH Mobile CEU interface
 *
 * Copyright (C) 2008 Magnus Damm
 *
 * Based on V4L2 Driver for PXA camera host - "pxa_camera.c",
 *
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 * Copyright (C) 2009, Parrot SA (by <matthieu.castet@parrot.com>)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/clk.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/soc_camera.h>
#include <media/videobuf-dma-contig.h>
#include <media/v4l2-ioctl.h>

#include <mach/regs-camif-p6.h>
struct p6_camif_info {
	unsigned long flags; /* SOCAM_... */
	void (*enable_camera)(void);
	void (*disable_camera)(void);
};

static DEFINE_MUTEX(camera_lock);

/* per video frame buffer */
struct p6_camif_buffer {
	struct videobuf_buffer vb; /* v4l buffer must be first */
	const struct soc_camera_data_format *fmt;
};

struct p6_camif_dev {
	struct device *dev;
	struct soc_camera_host ici;
	struct soc_camera_device *icd;

	unsigned int irq;
	void __iomem *base;
	unsigned long video_limit;

	/* lock used to protect videobuf */
	spinlock_t lock;
	struct list_head capture;
	struct videobuf_buffer *active[3];
	unsigned int active_idx;
#ifdef DEBUG_BUFFER
	struct videobuf_queue *vq_stat;
#endif

	wait_queue_head_t       blockline;
	unsigned int			block_count;
	struct v4l2_buffer		blockinfo;

	struct clk *clk;
	struct p6_camif_info *pdata;
};

static int lines_per_irq = 7;
module_param(lines_per_irq, int, 0644);
MODULE_PARM_DESC(lines_per_irq, "generate an it each 2^lines_per_irq lines (range 0-7 default 7)");
static int silent_drop_frame;
module_param(silent_drop_frame, int, 0644);
MODULE_PARM_DESC(silent_drop_frame, "if frame is dropped, don't stop the capture, but continue using the same buffer");


static void camif_write(struct p6_camif_dev *priv,
		      unsigned long reg_offs, unsigned long data)
{
	iowrite32(data, priv->base + reg_offs);
}

static unsigned long camif_read(struct p6_camif_dev *priv,
			      unsigned long reg_offs)
{
	return ioread32(priv->base + reg_offs);
}

static void abort_current_buffer(struct soc_camera_device *icd, int wake)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;
	int i;
	struct videobuf_buffer *vb;

	camif_write(pcdev, _P6_CAMIF_CTRL, 
			camif_read(pcdev, _P6_CAMIF_CTRL) & ~P6_CAMIF_CTRL_P6_CAMIF_EN);
	// abort pending buffer....
	for (i = 0; i < 3; i++) {
		vb = pcdev->active[i];
		if (vb->state != VIDEOBUF_ACTIVE)
			continue;
		vb->state = VIDEOBUF_ERROR;
		if (wake) {
			//do_gettimeofday(&vb->ts);
			//vb->field_count++;
			wake_up_all(&vb->done);
		}
	}

	dev_dbg(&icd->dev, "clearing idx\n");
	pcdev->active_idx = 0;
	pcdev->block_count = 0;
	memset(pcdev->active, 0, sizeof(pcdev->active));
}

/*
 *  Videobuf operations
 */
static int p6_camif_videobuf_setup(struct videobuf_queue *vq,
					unsigned int *count,
					unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;
	/* XXX we overalocate memory */
	int bytes_per_pixel = (icd->current_fmt->depth + 7) >> 3;
	/* add extra space in the buffer to detect when the IP miss
	   the end of frame
	 */
	int extra_lines = (1 << lines_per_irq) * 2;

	*size = PAGE_ALIGN(icd->width * (icd->height + extra_lines) * bytes_per_pixel);

	if (0 == *count)
		*count = 2;

	if (pcdev->video_limit) {
		while (*size * *count > pcdev->video_limit)
			(*count)--;
	}

	dev_dbg(&icd->dev, "count=%d, size=%d\n", *count, *size);

	return 0;
}

static void free_buffer(struct videobuf_queue *vq,
			struct p6_camif_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		&buf->vb, buf->vb.baddr, buf->vb.bsize);

	if (in_interrupt())
		BUG();

#ifdef DEBUG_BUFFER
	struct videobuf_buffer *vb = &buf->vb;
	if (vb->state == VIDEOBUF_ACTIVE || vb->state == VIDEOBUF_QUEUED) {
		struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
		struct p6_camif_dev *pcdev = ici->priv;
		printk("vb is strange state %p %x %x\n", vb, vb->state, camif_read(pcdev, _P6_CAMIF_CTRL) & P6_CAMIF_CTRL_P6_CAMIF_EN);
	}
#endif
	videobuf_waiton(&buf->vb, 0, 0);
	videobuf_dma_contig_free(vq, &buf->vb);
	dev_dbg(&icd->dev, "%s freed\n", __func__);
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

static void p6_camif_capture(struct p6_camif_dev *pcdev, unsigned int idx)
{
	struct soc_camera_device *icd = pcdev->icd;
	u32 base = videobuf_to_dma_contig(pcdev->active[idx]);
	int cb_offset = 0, cr_offset = 0;

	switch (icd->current_fmt->fourcc) {
		case V4L2_PIX_FMT_YUV422P: /* YCbCr 3 plane */
			cb_offset = icd->height * icd->width;
			cr_offset = cb_offset + cb_offset/2;
			break;
		case V4L2_PIX_FMT_YUV420: /* YCbCr */
			cb_offset = icd->height * icd->width;
			cr_offset = cb_offset + cb_offset/4;
			break;
		case V4L2_PIX_FMT_YVU420: /* YCrCb */
			cr_offset = icd->height * icd->width;
			cb_offset = cr_offset + cr_offset/4;
			break;
		default:
			BUG();
	}
	camif_write(pcdev, _P6_CAMIF_YBASE1ADDR  + idx*4, base);
	camif_write(pcdev, _P6_CAMIF_CBBASE1ADDR + idx*4, base + cb_offset);
	camif_write(pcdev, _P6_CAMIF_CRBASE1ADDR + idx*4, base + cr_offset);
}

static int p6_camif_videobuf_prepare(struct videobuf_queue *vq,
					  struct videobuf_buffer *vb,
					  enum v4l2_field field)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct p6_camif_buffer *buf;
	int ret;

	buf = container_of(vb, struct p6_camif_buffer, vb);

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		vb, vb->baddr, vb->bsize);

	/* Added list head initialization on alloc */
	WARN_ON(!list_empty(&vb->queue));

#ifdef DEBUG_BUFFER
	/* This can be useful if you want to see if we actually fill
	 * the buffer with something */
	memset(videobuf_queue_to_vmalloc(vq,vb), 0xaa, vb->bsize);
#endif

	BUG_ON(NULL == icd->current_fmt);

	if (buf->fmt	!= icd->current_fmt ||
	    vb->width	!= icd->width ||
	    vb->height	!= icd->height ||
	    vb->field	!= field) {
		buf->fmt	= icd->current_fmt;
		vb->width	= icd->width;
		vb->height	= icd->height;
		vb->field	= field;
		vb->state	= VIDEOBUF_NEEDS_INIT;
	}

	vb->size = vb->width * vb->height * ((buf->fmt->depth + 7) >> 3);
	if (0 != vb->baddr && vb->bsize < vb->size) {
		ret = -EINVAL;
		goto out;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;
		vb->state = VIDEOBUF_PREPARED;
	}

	return 0;
fail:
	free_buffer(vq, buf);
out:
	return ret;
}

static void p6_camif_videobuf_queue(struct videobuf_queue *vq,
					 struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;
	unsigned long flags;

	dev_dbg(&icd->dev, "%s (vb=0x%p) 0x%08lx %zd\n", __func__,
		vb, vb->baddr, vb->bsize);

	vb->state = VIDEOBUF_ACTIVE;
	spin_lock_irqsave(&pcdev->lock, flags);
#ifdef DEBUG_BUFFER
	if (videobuf_queue_to_vmalloc(pcdev->vq_stat,vb) == NULL) {
		printk("invalid vd %p %x %p\n", vb, videobuf_to_dma_contig(vb), videobuf_queue_to_vmalloc(vq, vb));
	}
#endif

	dev_dbg(&icd->dev, "%d %p\n", pcdev->active_idx, pcdev->active[2]);
	if (!pcdev->active[2]) {
		dev_dbg(&icd->dev, "queueing buffer %d in %s (%p %x)\n", pcdev->active_idx, __func__, vb, videobuf_to_dma_contig(vb));
		BUG_ON(camif_read(pcdev, _P6_CAMIF_CTRL) & P6_CAMIF_CTRL_P6_CAMIF_EN);
		pcdev->active[pcdev->active_idx] = vb;
		p6_camif_capture(pcdev, pcdev->active_idx);
		pcdev->active_idx++;
		if (pcdev->active_idx > 2) {
			pcdev->active_idx = 0;
			/* start dma */
			dev_dbg(&icd->dev, "starting dma in %s\n", __func__);
			camif_write(pcdev, _P6_CAMIF_CTRL, 
					camif_read(pcdev, _P6_CAMIF_CTRL) | P6_CAMIF_CTRL_P6_CAMIF_EN);
		}
	}
	else {
		list_add_tail(&vb->queue, &pcdev->capture);
	}

	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static void p6_camif_videobuf_release(struct videobuf_queue *vq,
					   struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;
	unsigned long flags;

	spin_lock_irqsave(&pcdev->lock, flags);

	if (pcdev->active[0] == vb || pcdev->active[1] == vb || pcdev->active[2] == vb) {
		abort_current_buffer(icd, 0);
	}

	if ((vb->state == VIDEOBUF_ACTIVE || vb->state == VIDEOBUF_QUEUED) &&
			!list_empty(&vb->queue)) {
		vb->state = VIDEOBUF_ERROR;
		list_del_init(&vb->queue);
	}
	spin_unlock_irqrestore(&pcdev->lock, flags);
	free_buffer(vq, container_of(vb, struct p6_camif_buffer, vb));
}

static struct videobuf_queue_ops p6_camif_videobuf_ops = {
	.buf_setup      = p6_camif_videobuf_setup,
	.buf_prepare    = p6_camif_videobuf_prepare,
	.buf_queue      = p6_camif_videobuf_queue,
	.buf_release    = p6_camif_videobuf_release,
};

static irqreturn_t p6_camif_irq(int irq, void *data)
{
	struct p6_camif_dev *pcdev = data;
	struct soc_camera_device *icd = pcdev->icd;
	struct videobuf_buffer *vb = pcdev->active[pcdev->active_idx];
	struct videobuf_buffer *prevvb = vb;
	unsigned long flags;
	u32 curr_addr = camif_read(pcdev, _P6_CAMIF_YCURRENTADDR);

	if (camif_read(pcdev, _P6_CAMIF_STATUS) & P6_CAMIF_ITEN_OVERWRITE) {
		printk("it overwrite...\n");
		camif_write(pcdev, _P6_CAMIF_ITACK, P6_CAMIF_ITEN_OVERWRITE);
	}

	if (camif_read(pcdev, _P6_CAMIF_STATUS) & P6_CAMIF_ITEN_ERROR) {
		printk("it error...\n");
		camif_write(pcdev, _P6_CAMIF_ITACK, P6_CAMIF_ITEN_ERROR);
	}
	if (!(camif_read(pcdev, _P6_CAMIF_STATUS) & P6_CAMIF_ITEN_LINE))
		return IRQ_HANDLED;


	spin_lock_irqsave(&pcdev->lock, flags);
	BUG_ON(vb == NULL);
	/* no pending buffer : this shouldn't happen */
	if (vb == NULL)
		goto exit;

	dev_dbg(&icd->dev, "irq %x %x %x\n", videobuf_to_dma_contig(vb), curr_addr, pcdev->icd->height*pcdev->icd->width);
	do_gettimeofday(&pcdev->blockinfo.timestamp);
	
	pcdev->blockinfo.index    = vb->i;
	pcdev->blockinfo.memory   = vb->memory;
	pcdev->blockinfo.m.offset = vb->boff;
	pcdev->blockinfo.length   = vb->bsize;

	pcdev->block_count=1;
	wake_up(&pcdev->blockline);

	/* camif is writing our active buffer, nothing to do */
	if (videobuf_to_dma_contig(vb) <= curr_addr &&
		curr_addr < videobuf_to_dma_contig(vb) + pcdev->icd->height*pcdev->icd->width) {
		/* XXX this is in Y unit
		   may be we should do *bpps/8
		 */
		pcdev->blockinfo.length   = curr_addr - videobuf_to_dma_contig(vb);
		goto exit;
	}
#ifdef DEBUG_BUFFER
	{
		unsigned char *dum = videobuf_queue_to_vmalloc(pcdev->vq_stat,vb);
		if (dum) {
			//int bytes_per_pixel = (icd->current_fmt->depth + 7) >> 3;
			int len = vb->bsize;
			while (len-- > (pcdev->icd->height*pcdev->icd->width * icd->current_fmt->depth) >> 3) {
				if (dum[len] != 0xaa) {
					printk("overwrite at %x/%x...\n", len, vb->bsize);
					break;
				}
			}
		}
		else {
			printk("invalid buffer %p %x\n", vb, videobuf_to_dma_contig(vb));
		}
	}
#endif

	if (!list_empty(&pcdev->capture)) {
		dev_dbg(&icd->dev, "new buffer : %d\n", pcdev->active_idx);
		pcdev->active[pcdev->active_idx] = list_first_entry(&pcdev->capture,
					   struct videobuf_buffer, queue);
		list_del_init(&pcdev->active[pcdev->active_idx]->queue);
		p6_camif_capture(pcdev, pcdev->active_idx);

		vb->state = VIDEOBUF_DONE;
		do_gettimeofday(&vb->ts);
		vb->field_count++;
		wake_up(&vb->done);
	}
	/* XXX strange things can happen with blockline, if silent drop is enabled */
	else if (!silent_drop_frame) {
		dev_dbg(&icd->dev, "no new buffer : %d\n", pcdev->active_idx);
		abort_current_buffer(icd, 1);
		goto exit;
	}
	else
		printk("drop frame\n");

	pcdev->active_idx++;
	if (pcdev->active_idx > 2)
		pcdev->active_idx = 0;

	/* check if camif is writing to the next buffer */
	vb = pcdev->active[pcdev->active_idx];
	if(!(videobuf_to_dma_contig(vb) <= curr_addr &&
		curr_addr < videobuf_to_dma_contig(vb) + pcdev->icd->height*pcdev->icd->width) && 
		curr_addr != videobuf_to_dma_contig(prevvb) + pcdev->icd->height*pcdev->icd->width)
	{
		printk("addr %x next %x prev %x\n", curr_addr, videobuf_to_dma_contig(vb), videobuf_to_dma_contig(prevvb));
		BUG_ON(1);
	}

exit:
	camif_write(pcdev, _P6_CAMIF_ITACK, P6_CAMIF_ITEN_LINE);
	spin_unlock_irqrestore(&pcdev->lock, flags);

	return IRQ_HANDLED;
}

static int p6_camif_ioctl(struct soc_camera_device* icd, int cmd, void *arg)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;
	int ret;

	if (cmd != _IOWR('V', BASE_VIDIOC_PRIVATE, struct v4l2_buffer))
		return -EINVAL;

	ret = wait_event_interruptible_timeout(pcdev->blockline, pcdev->block_count, HZ);
	if (ret) {
		pcdev->block_count = 0;
		memcpy(arg, &pcdev->blockinfo, sizeof(pcdev->blockinfo));
		ret = 0;
	}
	else {
		ret = -EIO;
	}
	return ret;
}

static int p6_camif_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;
	const char* clk_name[] = { "camif0", "camif1"};
	int ret = -EBUSY;

	mutex_lock(&camera_lock);

	if (pcdev->icd)
		goto err;

	dev_info(&icd->dev,
		 "P6 camif driver attached to camera %d\n",
		 icd->devnum);

	//if (pcdev->pdata->enable_camera)
	//	pcdev->pdata->enable_camera();

	ret = icd->ops->init(icd);
	if (ret)
		goto err;

	BUG_ON(pcdev->ici.nr >= ARRAY_SIZE(clk_name));
	pcdev->clk = clk_get(NULL, clk_name[pcdev->ici.nr]);
	if (IS_ERR(pcdev->clk)) {
		dev_err(&icd->dev, "failed get_clk()\n");
		icd->ops->release(icd);
		ret = -EINVAL;
		goto err;
	}

	clk_enable(pcdev->clk);

#if 0
	camif_write(pcdev, CAPSR, 1 << 16); /* reset */
	while (camif_read(pcdev, CSTSR) & 1)
		msleep(1);
#else
	camif_write(pcdev, _P6_CAMIF_CTRL, 0);
	camif_write(pcdev, _P6_CAMIF_ITEN, 0);
#endif

	pcdev->icd = icd;
err:
	mutex_unlock(&camera_lock);

	return ret;
}

static void p6_camif_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;

	BUG_ON(icd != pcdev->icd);

	/* disable capture, disable interrupts */
	camif_write(pcdev, _P6_CAMIF_CTRL, 0);
	camif_write(pcdev, _P6_CAMIF_ITEN, 0);
	clk_disable(pcdev->clk);
	clk_put(pcdev->clk);
	icd->ops->release(icd);
	//if (pcdev->pdata->disable_camera)
	//	pcdev->pdata->disable_camera();

	dev_info(&icd->dev,
		 "p6 camif driver detached from camera %d\n",
		 icd->devnum);

	pcdev->icd = NULL;
}

static int p6_camif_set_bus_param(struct soc_camera_device *icd,
				       __u32 pixfmt)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;
	int ret;
	unsigned long camera_flags, common_flags, value;

	camera_flags = icd->ops->query_bus_param(icd);
#if 0
	common_flags = soc_camera_bus_param_compatible(camera_flags,
						       pcdev->pdata->flags);
	if (!common_flags)
		return -EINVAL;
#else
	common_flags = camera_flags;
#endif

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0)
		return ret;

	switch (common_flags & SOCAM_DATAWIDTH_MASK) {
	case SOCAM_DATAWIDTH_8:
		break;
	default:
		return -EINVAL;
	}

	value = 0;
	value = P6_CAMIF_CTRL_ORDER;
	value |= (common_flags & SOCAM_VSYNC_ACTIVE_LOW) ? (P6_CAMIF_CTRL_VSYNCPOL) : 0;
	value |= (common_flags & SOCAM_HSYNC_ACTIVE_LOW) ? (P6_CAMIF_CTRL_HSYNCPOL) : 0;
	value |= (common_flags & SOCAM_PCLK_SAMPLE_RISING) ? (P6_CAMIF_CTRL_SYNC_NEG) : 0;
	value |= (common_flags & SOCAM_SYNC_BT656) ? (P6_CAMIF_CTRL_SYNCSRC_BT656) : 0;

	switch (pixfmt) {
	case V4L2_PIX_FMT_YUV422P: /* YCbCr 3 plane */
		break;
	case V4L2_PIX_FMT_YUV420: /* YCbCr */
		value |= P6_CAMIF_CTRL_FORMAT;
		break;
	case V4L2_PIX_FMT_YVU420: /* YCrCb */
		value |= P6_CAMIF_CTRL_FORMAT;
		break;
	default:
		return -EINVAL;
	}
	/* TODO */
	/* negociation with sensor of SYNCSRC & ORDER */

	BUG_ON(icd->height < (1 << lines_per_irq));
	value |= ((lines_per_irq & 0x7) << 7);
	camif_write(pcdev, _P6_CAMIF_CTRL, value);

	camif_write(pcdev, _P6_CAMIF_ITEN, P6_CAMIF_ITEN_LINE|P6_CAMIF_ITEN_OVERWRITE|P6_CAMIF_ITEN_ERROR);
	camif_write(pcdev, _P6_CAMIF_DMA, P6_CAMIF_DMA_INCR16);

	return 0;
}

static int p6_camif_try_bus_param(struct soc_camera_device *icd,
				       __u32 pixfmt)
{
#if 0
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;
	unsigned long camera_flags, common_flags;

	camera_flags = icd->ops->query_bus_param(icd);
	common_flags = soc_camera_bus_param_compatible(camera_flags,
						       pcdev->pdata->flags);
	if (!common_flags)
		return -EINVAL;
#endif

	return 0;
}

static int p6_camif_set_fmt_cap(struct soc_camera_device *icd,
				     __u32 pixfmt, struct v4l2_rect *rect)
{
	return icd->ops->set_fmt_cap(icd, pixfmt, rect);
}

static int p6_camif_try_fmt_cap(struct soc_camera_device *icd,
				     struct v4l2_format *f)
{
	/* FIXME: calculate using depth and bus width */

	if (f->fmt.pix.height < 4)
		f->fmt.pix.height = 4;
	if (f->fmt.pix.height > 1920)
		f->fmt.pix.height = 1920;
	if (f->fmt.pix.width < 2)
		f->fmt.pix.width = 2;
	if (f->fmt.pix.width > 2560)
		f->fmt.pix.width = 2560;
	f->fmt.pix.width &= ~0x01;
	f->fmt.pix.height &= ~0x03;

	/* limit to sensor capabilities */
	return icd->ops->try_fmt_cap(icd, f);
}

static int p6_camif_reqbufs(struct soc_camera_file *icf,
				 struct v4l2_requestbuffers *p)
{
	int i;

	/* This is for locking debugging only. I removed spinlocks and now I
	 * check whether .prepare is ever called on a linked buffer, or whether
	 * a dma IRQ can occur for an in-work or unlinked buffer. Until now
	 * it hadn't triggered */
	for (i = 0; i < p->count; i++) {
		struct p6_camif_buffer *buf;

		buf = container_of(icf->vb_vidq.bufs[i],
				   struct p6_camif_buffer, vb);
		INIT_LIST_HEAD(&buf->vb.queue);
	}

	return 0;
}

static unsigned int p6_camif_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_file *icf = file->private_data;
	struct p6_camif_buffer *buf;

	buf = list_entry(icf->vb_vidq.stream.next,
			 struct p6_camif_buffer, vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if (buf->vb.state == VIDEOBUF_DONE ||
	    buf->vb.state == VIDEOBUF_ERROR)
		return POLLIN|POLLRDNORM;

	return 0;
}

static int p6_camif_querycap(struct soc_camera_host *ici,
				  struct v4l2_capability *cap)
{
	strlcpy(cap->card, "p6_camif", sizeof(cap->card));
	cap->version = KERNEL_VERSION(0, 0, 5);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

static void p6_camif_init_videobuf(struct videobuf_queue *q,
					struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct p6_camif_dev *pcdev = ici->priv;

	videobuf_queue_dma_contig_init(q,
				       &p6_camif_videobuf_ops,
				       pcdev->dev, &pcdev->lock,
				       V4L2_BUF_TYPE_VIDEO_CAPTURE,
				       V4L2_FIELD_NONE,
				       sizeof(struct p6_camif_buffer),
				       icd);
#ifdef DEBUG_BUFFER
	pcdev->vq_stat = q;
#endif
}

static struct soc_camera_host_ops p6_camif_host_ops = {
	.owner		= THIS_MODULE,
	.add		= p6_camif_add_device,
	.remove		= p6_camif_remove_device,
	.set_fmt_cap	= p6_camif_set_fmt_cap,
	.try_fmt_cap	= p6_camif_try_fmt_cap,
	.reqbufs	= p6_camif_reqbufs,
	.poll		= p6_camif_poll,
	.querycap	= p6_camif_querycap,
	.try_bus_param	= p6_camif_try_bus_param,
	.set_bus_param	= p6_camif_set_bus_param,
	.init_videobuf	= p6_camif_init_videobuf,
	.ioctl_default = p6_camif_ioctl,
};

static int p6_camif_probe(struct platform_device *pdev)
{
	struct p6_camif_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	unsigned int irq;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || !irq) {
		dev_err(&pdev->dev, "Not enough CEU platform resources.\n");
		err = -ENODEV;
		goto exit;
	}

	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	platform_set_drvdata(pdev, pcdev);
	INIT_LIST_HEAD(&pcdev->capture);
	spin_lock_init(&pcdev->lock);
	init_waitqueue_head(&pcdev->blockline);

	pcdev->pdata = pdev->dev.platform_data;
#if 0
	if (!pcdev->pdata) {
		err = -EINVAL;
		dev_err(&pdev->dev, "CEU platform data not set.\n");
		goto exit_kfree;
	}
#endif

	base = ioremap_nocache(res->start, res->end - res->start + 1);
	if (!base) {
		err = -ENXIO;
		dev_err(&pdev->dev, "Unable to ioremap CEU registers.\n");
		goto exit_kfree;
	}

	pcdev->irq = irq;
	pcdev->base = base;
	pcdev->video_limit = 0; /* only enabled if second resource exists */
	pcdev->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		err = dma_declare_coherent_memory(&pdev->dev, res->start,
						  res->start,
						  (res->end - res->start) + 1,
						  DMA_MEMORY_MAP |
						  DMA_MEMORY_EXCLUSIVE);
		if (!err) {
			dev_err(&pdev->dev, "Unable to declare CEU memory.\n");
			err = -ENXIO;
			goto exit_iounmap;
		}

		pcdev->video_limit = (res->end - res->start) + 1;
	}

	/* request irq */
	err = request_irq(pcdev->irq, p6_camif_irq, IRQF_DISABLED,
			  pdev->dev.bus_id, pcdev);
	if (err) {
		dev_err(&pdev->dev, "Unable to register CEU interrupt.\n");
		goto exit_release_mem;
	}

	pcdev->ici.priv = pcdev;
	pcdev->ici.dev.parent = &pdev->dev;
	pcdev->ici.nr = pdev->id;
	pcdev->ici.drv_name = pdev->dev.bus_id,
	pcdev->ici.ops = &p6_camif_host_ops,

	err = soc_camera_host_register(&pcdev->ici);
	if (err)
		goto exit_free_irq;

	return 0;

exit_free_irq:
	free_irq(pcdev->irq, pcdev);
exit_release_mem:
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1))
		dma_release_declared_memory(&pdev->dev);
exit_iounmap:
	iounmap(base);
exit_kfree:
	kfree(pcdev);
exit:
	return err;
}

static int p6_camif_remove(struct platform_device *pdev)
{
	struct p6_camif_dev *pcdev = platform_get_drvdata(pdev);

	soc_camera_host_unregister(&pcdev->ici);
	free_irq(pcdev->irq, pcdev);
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1))
		dma_release_declared_memory(&pdev->dev);
	iounmap(pcdev->base);
	kfree(pcdev);
	return 0;
}

static struct platform_driver p6_camif_driver = {
	.driver 	= {
		.name	= "p6_camif",
	},
	.probe		= p6_camif_probe,
	.remove		= p6_camif_remove,
};

static int __init p6_camif_init(void)
{
	return platform_driver_register(&p6_camif_driver);
}

static void __exit p6_camif_exit(void)
{
	platform_driver_unregister(&p6_camif_driver);
}

module_init(p6_camif_init);
module_exit(p6_camif_exit);

MODULE_LICENSE("GPL");
