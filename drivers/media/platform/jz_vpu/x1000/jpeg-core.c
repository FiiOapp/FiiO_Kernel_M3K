
/* linux/drivers/media/platform/jz_ipu/x1000/jpeg-core.c
 *
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Author: Gao Wei <wei.gao@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/syscalls.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "jpeg-core.h"
#include "jpeg-hw.h"
#include "jzm_jpeg_enc.h"

static struct x1000_jpeg_fmt formats_enc[] = {
	{
		.name		= "JPEG JFIF",
		.fourcc		= V4L2_PIX_FMT_JPEG,
		.colplanes	= 1,
		.types		= MEM2MEM_CAPTURE,
	},
	{
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= 16,
		.colplanes	= 1,
		.types		= MEM2MEM_OUTPUT,
	},
};
#define NUM_FORMATS_ENC ARRAY_SIZE(formats_enc)

static inline struct x1000_jpeg_ctx *ctrl_to_ctx(struct v4l2_ctrl *c)
{
	return container_of(c->handler, struct x1000_jpeg_ctx, ctrl_handler);
}

static inline struct x1000_jpeg_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct x1000_jpeg_ctx, fh);
}

/*
 * ============================================================================
 * Device file operations
 * ============================================================================
 */

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq);
static struct x1000_jpeg_fmt *x1000_jpeg_find_format(unsigned int mode,
						 __u32 pixelformat);
static int x1000_jpeg_controls_create(struct x1000_jpeg_ctx *ctx);

static int x1000_jpeg_open(struct file *file)
{
	struct x1000_jpeg *jpeg = video_drvdata(file);
	struct video_device *vfd = video_devdata(file);
	struct x1000_jpeg_ctx *ctx;
	struct x1000_jpeg_fmt *out_fmt;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&jpeg->lock)) {
		ret = -ERESTARTSYS;
		goto free;
	}

        ctx->desc_vidmem = dma_alloc_coherent(jpeg->dev,
			DESC_SIZE_MAX,
                        &ctx->desc_phys,
			GFP_KERNEL);
        if (!ctx->desc_vidmem)
                return -ENOMEM;

	v4l2_fh_init(&ctx->fh, vfd);
	/* Use separate control handler per file handle */
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->jpeg = jpeg;
	if (vfd == jpeg->vfd_encoder) {
		ctx->mode = X1000_JPEG_ENCODE;
		out_fmt = x1000_jpeg_find_format(ctx->mode, V4L2_PIX_FMT_YUYV);
	}

	ret = x1000_jpeg_controls_create(ctx);
	if (ret < 0)
		goto error;

	ctx->m2m_ctx = v4l2_m2m_ctx_init(jpeg->m2m_dev, ctx, queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		goto error;
	}

	ctx->out_q.fmt = out_fmt;
	ctx->cap_q.fmt = x1000_jpeg_find_format(ctx->mode, V4L2_PIX_FMT_YUYV);

	clk_prepare_enable(jpeg->clk);
	mutex_unlock(&jpeg->lock);
	return 0;

error:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&jpeg->lock);
free:
	kfree(ctx);
	return ret;
}

static int x1000_jpeg_release(struct file *file)
{
	struct x1000_jpeg *jpeg = video_drvdata(file);
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(file->private_data);

	mutex_lock(&jpeg->lock);
	clk_disable_unprepare(jpeg->clk);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	mutex_unlock(&jpeg->lock);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	dma_free_coherent(jpeg->dev, DESC_SIZE_MAX,
			ctx->desc_vidmem, ctx->desc_phys);
	kfree(ctx);

	return 0;
}

static unsigned int x1000_jpeg_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct x1000_jpeg *jpeg = video_drvdata(file);
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(file->private_data);
	unsigned int res;

	mutex_lock(&jpeg->lock);
	res = v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
	mutex_unlock(&jpeg->lock);
	return res;
}

static int x1000_jpeg_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct x1000_jpeg *jpeg = video_drvdata(file);
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(file->private_data);
	int ret;

	if (mutex_lock_interruptible(&jpeg->lock))
		return -ERESTARTSYS;
	ret = v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
	mutex_unlock(&jpeg->lock);
	return ret;
}


static const struct v4l2_file_operations x1000_jpeg_fops = {
	.owner		= THIS_MODULE,
	.open		= x1000_jpeg_open,
	.release	= x1000_jpeg_release,
	.poll		= x1000_jpeg_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= x1000_jpeg_mmap,
};

/*
 * ============================================================================
 * video ioctl operations
 * ============================================================================
 */

static int x1000_jpeg_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	if (ctx->mode == X1000_JPEG_ENCODE) {
		strlcpy(cap->driver, X1000_JPEG_M2M_NAME " encoder",
			sizeof(cap->driver));
		strlcpy(cap->card, X1000_JPEG_M2M_NAME " encoder",
			sizeof(cap->card));
	}
	cap->bus_info[0] = 0;
	/*
	 * This is only a mem-to-mem video device. The capture and output
	 * device capability flags are left only for backward compatibility
	 * and are scheduled for removal.
	 */
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M |
			    V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT;
	return 0;
}

static int enum_fmt(struct x1000_jpeg_fmt *formats, int n,
		    struct v4l2_fmtdesc *f, u32 type)
{
	int i, num = 0;

	for (i = 0; i < n; ++i) {
		if (formats[i].types & type) {
			/* index-th format of type type found ? */
			if (num == f->index)
				break;
			/* Correct type but haven't reached our index yet,
			 * just increment per-type index */
			++num;
		}
	}

	/* Format not found */
	if (i >= n)
		return -EINVAL;

	strlcpy(f->description, formats[i].name, sizeof(f->description));
	f->pixelformat = formats[i].fourcc;

	return 0;
}

static int x1000_jpeg_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	if (ctx->mode == X1000_JPEG_ENCODE)
		return enum_fmt(formats_enc, NUM_FORMATS_ENC, f,
				MEM2MEM_CAPTURE);
        return -1;
}

static int x1000_jpeg_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	if (ctx->mode == X1000_JPEG_ENCODE)
		return enum_fmt(formats_enc, NUM_FORMATS_ENC, f,
				MEM2MEM_OUTPUT);
        return -1;
}

static struct x1000_jpeg_q_data *get_q_data(struct x1000_jpeg_ctx *ctx,
					  enum v4l2_buf_type type)
{
	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return &ctx->out_q;
	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return &ctx->cap_q;

	return NULL;
}

static int x1000_jpeg_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct x1000_jpeg_q_data *q_data = NULL;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct x1000_jpeg_ctx *ct = fh_to_ctx(priv);

	vq = v4l2_m2m_get_vq(ct->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ct, f->type);
	BUG_ON(q_data == NULL);

	pix->width = q_data->w;
	pix->height = q_data->h;
	pix->field = V4L2_FIELD_NONE;
	pix->pixelformat = q_data->fmt->fourcc;
	pix->bytesperline = 0;
	if (q_data->fmt->fourcc != V4L2_PIX_FMT_JPEG) {
		u32 bpl = q_data->w;
		if (q_data->fmt->colplanes == 1)
			bpl = (bpl * q_data->fmt->depth) >> 3;
		pix->bytesperline = bpl;
	}
	pix->sizeimage = q_data->size;

	return 0;
}

static struct x1000_jpeg_fmt *x1000_jpeg_find_format(unsigned int mode,
						 u32 pixelformat)
{
	unsigned int k;
	struct x1000_jpeg_fmt *formats;
	int n;

	if (mode == X1000_JPEG_ENCODE) {
		formats = formats_enc;
		n = NUM_FORMATS_ENC;
	}

	for (k = 0; k < n; k++) {
		struct x1000_jpeg_fmt *fmt = &formats[k];
		if (fmt->fourcc == pixelformat)
			return fmt;
	}

	return NULL;
}

static void jpeg_bound_align_image(u32 *w, unsigned int wmin, unsigned int wmax,
				   unsigned int walign,
				   u32 *h, unsigned int hmin, unsigned int hmax,
				   unsigned int halign)
{
	int width, height, w_step, h_step;

	width = *w;
	height = *h;

	w_step = 1 << walign;
	h_step = 1 << halign;
	v4l_bound_align_image(w, wmin, wmax, walign, h, hmin, hmax, halign, 0);

	if (*w < width && (*w + w_step) < wmax)
		*w += w_step;
	if (*h < height && (*h + h_step) < hmax)
		*h += h_step;
}

static int vidioc_try_fmt(struct v4l2_format *f, struct x1000_jpeg_fmt *fmt,
			  struct x1000_jpeg_ctx *ctx, int q_type)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
		return -EINVAL;

	/* V4L2 specification suggests the driver corrects the format struct
	 * if any of the dimensions is unsupported */
	if (q_type == MEM2MEM_OUTPUT)
		jpeg_bound_align_image(&pix->width, X1000_JPEG_MIN_WIDTH,
				       X1000_JPEG_MAX_WIDTH, 0,
				       &pix->height, X1000_JPEG_MIN_HEIGHT,
				       X1000_JPEG_MAX_HEIGHT, 0);
	else
		jpeg_bound_align_image(&pix->width, X1000_JPEG_MIN_WIDTH,
				       X1000_JPEG_MAX_WIDTH, fmt->h_align,
				       &pix->height, X1000_JPEG_MIN_HEIGHT,
				       X1000_JPEG_MAX_HEIGHT, fmt->v_align);

	if (fmt->fourcc == V4L2_PIX_FMT_JPEG) {
		if (pix->sizeimage <= 0)
		        pix->sizeimage = (pix->width * pix->height * 16) >> 3;
		pix->bytesperline = 0;
	} else {
		u32 bpl = pix->bytesperline;

		if (fmt->colplanes > 1 && bpl < pix->width)
			bpl = pix->width; /* planar */

		if (fmt->colplanes == 1 && /* packed */
		    (bpl << 3) * fmt->depth < pix->width)
			bpl = (pix->width * fmt->depth) >> 3;

		pix->bytesperline = bpl;
		pix->sizeimage = (pix->width * pix->height * fmt->depth) >> 3;
	}

	return 0;
}

static int x1000_jpeg_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);
	struct x1000_jpeg_fmt *fmt;

	fmt = x1000_jpeg_find_format(ctx->mode, f->fmt.pix.pixelformat);
	if (!fmt || !(fmt->types & MEM2MEM_CAPTURE)) {
		v4l2_err(&ctx->jpeg->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	return vidioc_try_fmt(f, fmt, ctx, MEM2MEM_CAPTURE);
}

static int x1000_jpeg_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);
	struct x1000_jpeg_fmt *fmt;

	fmt = x1000_jpeg_find_format(ctx->mode, f->fmt.pix.pixelformat);
	if (!fmt || !(fmt->types & MEM2MEM_OUTPUT)) {
		v4l2_err(&ctx->jpeg->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	return vidioc_try_fmt(f, fmt, ctx, MEM2MEM_OUTPUT);
}

static int x1000_jpeg_s_fmt(struct x1000_jpeg_ctx *ct, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct x1000_jpeg_q_data *q_data = NULL;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	vq = v4l2_m2m_get_vq(ct->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ct, f->type);
	BUG_ON(q_data == NULL);

	if (vb2_is_busy(vq)) {
		v4l2_err(&ct->jpeg->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	q_data->fmt = x1000_jpeg_find_format(ct->mode, pix->pixelformat);
	q_data->w = pix->width;
	q_data->h = pix->height;
	if (q_data->fmt->fourcc != V4L2_PIX_FMT_JPEG)
		q_data->size = q_data->w * q_data->h * q_data->fmt->depth >> 3;
	else
		q_data->size = pix->sizeimage;

	return 0;
}

static int x1000_jpeg_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = x1000_jpeg_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	return x1000_jpeg_s_fmt(fh_to_ctx(priv), f);
}

static int x1000_jpeg_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = x1000_jpeg_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	return x1000_jpeg_s_fmt(fh_to_ctx(priv), f);
}

static int x1000_jpeg_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *reqbufs)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

static int x1000_jpeg_querybuf(struct file *file, void *priv,
			   struct v4l2_buffer *buf)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);
}

static int x1000_jpeg_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int x1000_jpeg_dqbuf(struct file *file, void *priv,
			  struct v4l2_buffer *buf)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int x1000_jpeg_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int x1000_jpeg_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static int x1000_jpeg_g_selection(struct file *file, void *priv,
			 struct v4l2_selection *s)
{
	struct x1000_jpeg_ctx *ctx = fh_to_ctx(priv);

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* For JPEG blob active == default == bounds */
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.width = ctx->out_q.w;
		s->r.height = ctx->out_q.h;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r.width = ctx->cap_q.w;
		s->r.height = ctx->cap_q.h;
		break;
	default:
		return -EINVAL;
	}
	s->r.left = 0;
	s->r.top = 0;
	return 0;
}

/*
 * V4L2 controls
 */

static int x1000_jpeg_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct x1000_jpeg_ctx *ctx = ctrl_to_ctx(ctrl);
	struct x1000_jpeg *jpeg = ctx->jpeg;
	unsigned long flags;

	switch (ctrl->id) {
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		spin_lock_irqsave(&jpeg->slock, flags);
		ctrl->val = ctx->compr_quality;
		spin_unlock_irqrestore(&jpeg->slock, flags);
		break;
	}

	return 0;
}

static int x1000_jpeg_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct x1000_jpeg_ctx *ctx = ctrl_to_ctx(ctrl);
	unsigned long flags;

	spin_lock_irqsave(&ctx->jpeg->slock, flags);

	switch (ctrl->id) {
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		ctx->compr_quality = ctrl->val;
		break;
        default:
		break;
	}

	spin_unlock_irqrestore(&ctx->jpeg->slock, flags);
	return 0;
}

static const struct v4l2_ctrl_ops x1000_jpeg_ctrl_ops = {
	.g_volatile_ctrl	= x1000_jpeg_g_volatile_ctrl,
	.s_ctrl			= x1000_jpeg_s_ctrl,
};

static int x1000_jpeg_controls_create(struct x1000_jpeg_ctx *ctx)
{
	unsigned int mask = ~0x27; /* 444, 422, 420, GRAY */
	struct v4l2_ctrl *ctrl;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 3);

	if (ctx->mode == X1000_JPEG_ENCODE) {
		v4l2_ctrl_new_std(&ctx->ctrl_handler, &x1000_jpeg_ctrl_ops,
				  V4L2_CID_JPEG_COMPRESSION_QUALITY,
				  0, 3, 1, 3);

		/* v4l2_ctrl_new_std(&ctx->ctrl_handler, &x1000_jpeg_ctrl_ops, */
		/* 		  V4L2_CID_JPEG_RESTART_INTERVAL, */
		/* 		  0, 3, 0xffff, 0); */
		mask = ~0x06; /* 422, 420 */
	}

        //XXX: didn't test.
	ctrl = v4l2_ctrl_new_std_menu(&ctx->ctrl_handler, &x1000_jpeg_ctrl_ops,
				      V4L2_CID_JPEG_CHROMA_SUBSAMPLING,
				      V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY, mask,
				      V4L2_JPEG_CHROMA_SUBSAMPLING_422);

	if (ctx->ctrl_handler.error)
		return ctx->ctrl_handler.error;

	return 0;
}

static const struct v4l2_ioctl_ops x1000_jpeg_ioctl_ops = {
	.vidioc_querycap		= x1000_jpeg_querycap,

	.vidioc_enum_fmt_vid_cap	= x1000_jpeg_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out	= x1000_jpeg_enum_fmt_vid_out,

	.vidioc_g_fmt_vid_cap		= x1000_jpeg_g_fmt,
	.vidioc_g_fmt_vid_out		= x1000_jpeg_g_fmt,

	.vidioc_try_fmt_vid_cap		= x1000_jpeg_try_fmt_vid_cap,
	.vidioc_try_fmt_vid_out		= x1000_jpeg_try_fmt_vid_out,

	.vidioc_s_fmt_vid_cap		= x1000_jpeg_s_fmt_vid_cap,
	.vidioc_s_fmt_vid_out		= x1000_jpeg_s_fmt_vid_out,

	.vidioc_reqbufs			= x1000_jpeg_reqbufs,
	.vidioc_querybuf		= x1000_jpeg_querybuf,

	.vidioc_qbuf			= x1000_jpeg_qbuf,
	.vidioc_dqbuf			= x1000_jpeg_dqbuf,

	.vidioc_streamon		= x1000_jpeg_streamon,
	.vidioc_streamoff		= x1000_jpeg_streamoff,

	.vidioc_g_selection		= x1000_jpeg_g_selection,
};

/*
 * ============================================================================
 * mem2mem callbacks
 * ============================================================================
 */

static void JPEGE_SliceInit(JPEGE_SliceInfo *s)
{
	unsigned int i;
	volatile unsigned int *chn = (volatile unsigned int *)s->des_va;

	GEN_VDMA_ACFG(chn, TCSM_FLUSH, 0, 0x0);

	/* Open clock configuration */
	GEN_VDMA_ACFG(chn, REG_JPGC_GLBI, 0, OPEN_CLOCK);

	/**
	 * Huffman Encode Table configuration
	 */
        for(i = 0; i < HUFFENC_LEN; i++)
		GEN_VDMA_ACFG(chn, REG_JPGC_HUFE+i*4, 0, huffenc[s->huffenc_sel][i]);

	/**
	 * Quantization Table configuration
	 */
	for(i = 0; i < QMEM_LEN; i++)
		GEN_VDMA_ACFG(chn, REG_JPGC_QMEM+i*4, 0, qmem[s->ql_sel][i]);

	/**
	* REGs configuration
	*/
	GEN_VDMA_ACFG(chn, REG_JPGC_STAT, 0,STAT_CLEAN);
	GEN_VDMA_ACFG(chn,REG_JPGC_BSA, 0, (unsigned int)s->bsa);
	GEN_VDMA_ACFG(chn, REG_JPGC_P0A, 0, VRAM_RAWY_BA);

	GEN_VDMA_ACFG(chn,REG_JPGC_NMCU, 0,s->nmcu);
	GEN_VDMA_ACFG(chn,REG_JPGC_NRSM, 0,s->nrsm);

	GEN_VDMA_ACFG(chn,REG_JPGC_P0C, 0,YUV420P0C);
	GEN_VDMA_ACFG(chn,REG_JPGC_P1C, 0,YUV420P1C);
	GEN_VDMA_ACFG(chn,REG_JPGC_P2C, 0,YUV420P2C);

	GEN_VDMA_ACFG(chn, REG_JPGC_GLBI, 0, (YUV420PVH          |
					      JPGC_NCOL          |
					      JPGC_SPEC/* MODE */|
					      JPGC_EFE           |
					      JPGC_EN));
	GEN_VDMA_ACFG(chn, REG_JPGC_TRIG, 0,				\
		      JPGC_BS_TRIG | JPGC_PP_TRIG | JPGC_CORE_OPEN);
	/**
	 * EFE configuration
	 */
	GEN_VDMA_ACFG(chn, REG_EFE_GEOM, 0, (EFE_JPGC_LST_MBY(s->mb_height-1) |
					     EFE_JPGC_LST_MBX(s->mb_width-1)));

	GEN_VDMA_ACFG(chn, REG_EFE_RAWY_SBA, 0, s->raw[0]);
	GEN_VDMA_ACFG(chn, REG_EFE_RAWU_SBA, 0, s->raw[1]);
	GEN_VDMA_ACFG(chn, REG_EFE_RAWV_SBA, 0, s->raw[2]);
	GEN_VDMA_ACFG(chn, REG_EFE_RAW_STRD, 0, (EFE_RAW_STRDY(s->stride[0]) |
						 EFE_RAW_STRDC(s->stride[1])));

	GEN_VDMA_ACFG(chn, REG_EFE_RAW_DBA, 0, VRAM_RAWY_BA);
	GEN_VDMA_ACFG(chn, REG_EFE_CTRL, VDMA_ACFG_TERM, (YUV_YUY2 |
							  EFE_PLANE_NV12 |
							  EFE_ID_JPEG    |
							  EFE_EN         |
							  (s->raw_format)|
							  EFE_RUN));
}

static void x1000_jpeg_device_run(void *priv)
{
	struct x1000_jpeg_ctx *ctx = priv;
	struct x1000_jpeg *jpeg = ctx->jpeg;
	JPEGE_SliceInfo vpu_s;

        jpeg_clear_stat(jpeg->regs);
        if (jpeg_reset(jpeg->regs))
		dev_warn(jpeg->dev, "[%d:%d] wait stop ack timeout\n",
			 current->tgid, current->pid);

        jpeg_set_hiaxi(jpeg->regs);
        jpeg_clear_int(jpeg->regs);

        {
                int width = ctx->out_q.w;
                int height = ctx->out_q.h;

                struct vb2_buffer *src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
                struct vb2_buffer *dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);

                vpu_s.des_va      = (unsigned int)ctx->desc_vidmem;
                vpu_s.des_pa      = ctx->desc_phys;

                vpu_s.raw[0]      = vb2_dma_contig_plane_dma_addr(src_buf, 0);
                vpu_s.bsa         = (uint8_t*)vb2_dma_contig_plane_dma_addr(dst_buf, 0);

                vpu_s.mb_width    = width/16;
                vpu_s.mb_height   = height/16;
                vpu_s.stride[0]   = width * 2;
                vpu_s.stride[1]   = width;
                vpu_s.nmcu        = width * height / 256 - 1;

                vpu_s.nrsm        = 0;
                vpu_s.rsm         = 0;
                vpu_s.ncol        = 2; /* yuv(3) - 1 */
                vpu_s.raw_format  = JPGE_NV21_MODE; /* EFE working, RAW data is NV21 */

                vpu_s.huffenc_sel = 0;
                vpu_s.ql_sel      = ctx->compr_quality;
        }

	JPEGE_SliceInit(&vpu_s);

        jpeg_start(jpeg->regs, ctx->desc_phys);
}

static int x1000_jpeg_job_ready(void *priv)
{
	return 1;
}

static void x1000_jpeg_job_abort(void *priv)
{
}

static void x1000_jpeg_lock(void *prv)
{
	struct x1000_jpeg_ctx *ctx = prv;
	struct x1000_jpeg *jpeg = ctx->jpeg;
	mutex_lock(&jpeg->lock);
}

static void x1000_jpeg_unlock(void *prv)
{
	struct x1000_jpeg_ctx *ctx = prv;
	struct x1000_jpeg *jpeg = ctx->jpeg;
	mutex_unlock(&jpeg->lock);
}

static struct v4l2_m2m_ops x1000_jpeg_m2m_ops = {
	.device_run	= x1000_jpeg_device_run,
	.job_ready	= x1000_jpeg_job_ready,
	.job_abort	= x1000_jpeg_job_abort,
	.lock		= x1000_jpeg_lock,
	.unlock		= x1000_jpeg_unlock,
};


/*
 * ============================================================================
 * Queue operations
 * ============================================================================
 */

static int x1000_jpeg_queue_setup(struct vb2_queue *vq,
			   const struct v4l2_format *fmt,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], void *alloc_ctxs[])
{
	struct x1000_jpeg_ctx *ctx = vb2_get_drv_priv(vq);
	struct x1000_jpeg_q_data *q_data = NULL;
	unsigned int size, count = *nbuffers;

	q_data = get_q_data(ctx, vq->type);
	BUG_ON(q_data == NULL);

	size = q_data->size;

	*nbuffers = count;
	*nplanes = 1;
	sizes[0] = size;
	alloc_ctxs[0] = ctx->jpeg->alloc_ctx;

	return 0;
}

static int x1000_jpeg_buf_prepare(struct vb2_buffer *vb)
{
	struct x1000_jpeg_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct x1000_jpeg_q_data *q_data = NULL;

	q_data = get_q_data(ctx, vb->vb2_queue->type);
	BUG_ON(q_data == NULL);

	if (vb2_plane_size(vb, 0) < q_data->size) {
		pr_err("%s data will not fit into plane (%lu < %lu)\n",
				__func__, vb2_plane_size(vb, 0),
				(long)q_data->size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, q_data->size);

	return 0;
}

static void x1000_jpeg_buf_queue(struct vb2_buffer *vb)
{
	struct x1000_jpeg_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (ctx->m2m_ctx)
		v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);
}

static void x1000_jpeg_wait_prepare(struct vb2_queue *vq)
{
	struct x1000_jpeg_ctx *ctx = vb2_get_drv_priv(vq);

	mutex_unlock(&ctx->jpeg->lock);
}

static void x1000_jpeg_wait_finish(struct vb2_queue *vq)
{
	struct x1000_jpeg_ctx *ctx = vb2_get_drv_priv(vq);

	mutex_lock(&ctx->jpeg->lock);
}

static int x1000_jpeg_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct x1000_jpeg_ctx *ctx = vb2_get_drv_priv(q);
	int ret;

	ret = pm_runtime_get_sync(ctx->jpeg->dev);

	return ret > 0 ? 0 : ret;
}

static int x1000_jpeg_stop_streaming(struct vb2_queue *q)
{
	struct x1000_jpeg_ctx *ctx = vb2_get_drv_priv(q);

	pm_runtime_put(ctx->jpeg->dev);

	return 0;
}

static struct vb2_ops x1000_jpeg_qops = {
	.queue_setup		= x1000_jpeg_queue_setup,
	.buf_prepare		= x1000_jpeg_buf_prepare,
	.buf_queue		= x1000_jpeg_buf_queue,
	.wait_prepare		= x1000_jpeg_wait_prepare,
	.wait_finish		= x1000_jpeg_wait_finish,
	.start_streaming	= x1000_jpeg_start_streaming,
	.stop_streaming		= x1000_jpeg_stop_streaming,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct x1000_jpeg_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &x1000_jpeg_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &x1000_jpeg_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	return vb2_queue_init(dst_vq);
}
/*
 * ============================================================================
 * ISR
 * ============================================================================
 */

static irqreturn_t x1000_jpeg_irq(int irq, void *dev_id)
{
	unsigned int sch_stat, aux_stat;

	struct x1000_jpeg *jpeg = dev_id;
	struct x1000_jpeg_ctx *curr_ctx;
	struct vb2_buffer *src_buf, *dst_buf;
	unsigned long payload_size = 0;
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;

	spin_lock(&jpeg->slock);

	curr_ctx = v4l2_m2m_get_curr_priv(jpeg->m2m_dev);
	if(unlikely(curr_ctx == NULL)) {
		printk("JPEG:ctx == NULL, Dev stopped,interrupt come in.\n");
		spin_unlock(&jpeg->slock);
		return IRQ_HANDLED;
	}

	src_buf = v4l2_m2m_src_buf_remove(curr_ctx->m2m_ctx);
	if(unlikely(src_buf == NULL)) {
		printk("JPEG:src_buf == NULL, Dev stopped,interrupt come in.\n");
		spin_unlock(&jpeg->slock);
		return IRQ_HANDLED;
	}
	dst_buf = v4l2_m2m_dst_buf_remove(curr_ctx->m2m_ctx);
	if(unlikely(dst_buf == NULL)) {
		printk("JPEG:dst_buf == NULL, Dev stopped,interrupt come in.\n");
		spin_unlock(&jpeg->slock);
		return IRQ_HANDLED;
	}

	sch_stat = jpeg_get_sch_stat(jpeg->regs);

	if(sch_stat) {
		if(sch_stat & SCH_STAT_ENDF) {
			if(sch_stat & SCH_STAT_JPGEND) {
                                payload_size = jpeg_compressed_size(jpeg->regs);
				dev_dbg(jpeg->dev, "JPG successfully done!\n");
				CLEAR_REG_BIT(jpeg->regs,REG_JPGC_STAT,JPGC_STAT_ENDF);
			} else {
				dev_dbg(jpeg->dev, "SCH successfully done!\n");
				CLEAR_REG_BIT(jpeg->regs,REG_SDE_STAT,SDE_STAT_BSEND);
				CLEAR_REG_BIT(jpeg->regs,REG_DBLK_STAT,DBLK_STAT_DOEND);
			}
		} else {
			CHECK_SCH_STAT(SCH_STAT_SLDERR, "SHLD error!\n");
			CHECK_SCH_STAT(SCH_STAT_TLBERR, "TLB error! Addr is 0x%08x\n",
					 jpeg_get_sch_stat(jpeg->regs));
			CHECK_SCH_STAT(SCH_STAT_BSERR, "BS error!\n");
			CHECK_SCH_STAT(SCH_STAT_ACFGERR, "ACFG error!\n");
			CHECK_SCH_STAT(SCH_STAT_TIMEOUT, "TIMEOUT error!\n");
			CLEAR_REG_BIT(jpeg->regs,REG_SCH_GLBC,
				      (SCH_INTE_ACFGERR | SCH_INTE_TLBERR |
				       SCH_INTE_BSERR | SCH_INTE_ENDF));
		        state = VB2_BUF_STATE_ERROR;
		}
	} else {
                aux_stat = jpeg_get_aux_stat(jpeg->regs);
		if(aux_stat & AUX_STAT_MIRQP) {
			dev_dbg(jpeg->dev, "AUX successfully done!\n");
			CLEAR_REG_BIT(jpeg->regs,REG_AUX_STAT,AUX_STAT_MIRQP);
		} else {
			dev_dbg(jpeg->dev, "illegal interrupt happened!\n");
		}
	}

	dst_buf->v4l2_buf.timecode = src_buf->v4l2_buf.timecode;
	dst_buf->v4l2_buf.timestamp = src_buf->v4l2_buf.timestamp;
	dst_buf->v4l2_buf.reserved = payload_size;
        // XXX working bad!
	/* dst_buf->v4l2_buf.length = payload_size; */

	v4l2_m2m_buf_done(src_buf, state);
	if (curr_ctx->mode == X1000_JPEG_ENCODE)
		vb2_set_plane_payload(dst_buf, 0, payload_size);
	v4l2_m2m_buf_done(dst_buf, state);
	v4l2_m2m_job_finish(jpeg->m2m_dev, curr_ctx->m2m_ctx);

	spin_unlock(&jpeg->slock);

	return IRQ_HANDLED;
}
/*
 * ============================================================================
 * Driver basic infrastructure
 * ============================================================================
 */

static int x1000_jpeg_probe(struct platform_device *pdev)
{
	struct x1000_jpeg *jpeg;
	struct resource *res;
	int ret;

	/* JPEG IP abstraction struct */
	jpeg = devm_kzalloc(&pdev->dev, sizeof(struct x1000_jpeg), GFP_KERNEL);
	if (!jpeg)
		return -ENOMEM;

	mutex_init(&jpeg->lock);
	spin_lock_init(&jpeg->slock);
	jpeg->dev = &pdev->dev;

	/* memory-mapped registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	jpeg->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jpeg->regs))
		return PTR_ERR(jpeg->regs);

	/* interrupt service routine registration */
	jpeg->irq = ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, jpeg->irq, x1000_jpeg_irq, 0,
			dev_name(&pdev->dev), jpeg);
	if (ret) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", jpeg->irq);
		return ret;
	}

	/* clocks */
	jpeg->clk = clk_get(&pdev->dev, "vpu");
	if (IS_ERR(jpeg->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = PTR_ERR(jpeg->clk);
		return ret;
	}
	dev_dbg(&pdev->dev, "clock source %p\n", jpeg->clk);

	/* v4l2 device */
	ret = v4l2_device_register(&pdev->dev, &jpeg->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register v4l2 device\n");
		goto clk_get_rollback;
	}

	/* mem2mem device */
	jpeg->m2m_dev = v4l2_m2m_init(&x1000_jpeg_m2m_ops);
	if (IS_ERR(jpeg->m2m_dev)) {
		v4l2_err(&jpeg->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(jpeg->m2m_dev);
		goto device_register_rollback;
	}

	jpeg->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(jpeg->alloc_ctx)) {
		v4l2_err(&jpeg->v4l2_dev, "Failed to init memory allocator\n");
		ret = PTR_ERR(jpeg->alloc_ctx);
		goto m2m_init_rollback;
	}

	/* JPEG encoder /dev/videoX node */
	jpeg->vfd_encoder = video_device_alloc();
	if (!jpeg->vfd_encoder) {
		v4l2_err(&jpeg->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto vb2_allocator_rollback;
	}
	strlcpy(jpeg->vfd_encoder->name, X1000_JPEG_M2M_NAME,
		sizeof(jpeg->vfd_encoder->name));
	jpeg->vfd_encoder->fops		= &x1000_jpeg_fops;
	jpeg->vfd_encoder->ioctl_ops	= &x1000_jpeg_ioctl_ops;
	jpeg->vfd_encoder->minor	= -1;
	jpeg->vfd_encoder->release	= video_device_release;
	jpeg->vfd_encoder->lock		= &jpeg->lock;
	jpeg->vfd_encoder->v4l2_dev	= &jpeg->v4l2_dev;
	jpeg->vfd_encoder->vfl_dir	= VFL_DIR_M2M;

	ret = video_register_device(jpeg->vfd_encoder, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&jpeg->v4l2_dev, "Failed to register video device\n");
		goto enc_vdev_alloc_rollback;
	}

	video_set_drvdata(jpeg->vfd_encoder, jpeg);
	v4l2_info(&jpeg->v4l2_dev,
		  "encoder device registered as /dev/video%d\n",
		  jpeg->vfd_encoder->num);

	/* final statements & power management */
	platform_set_drvdata(pdev, jpeg);

	pm_runtime_enable(&pdev->dev);

	v4l2_info(&jpeg->v4l2_dev, "Ingenic X1000 JPEG codec\n");

	return 0;

enc_vdev_alloc_rollback:
	video_device_release(jpeg->vfd_encoder);

vb2_allocator_rollback:
	vb2_dma_contig_cleanup_ctx(jpeg->alloc_ctx);

m2m_init_rollback:
	v4l2_m2m_release(jpeg->m2m_dev);

device_register_rollback:
	v4l2_device_unregister(&jpeg->v4l2_dev);

clk_get_rollback:
	clk_disable_unprepare(jpeg->clk);
	clk_put(jpeg->clk);

	return ret;
}

static int x1000_jpeg_remove(struct platform_device *pdev)
{
	struct x1000_jpeg *jpeg = platform_get_drvdata(pdev);

	pm_runtime_disable(jpeg->dev);

	video_unregister_device(jpeg->vfd_encoder);
	video_device_release(jpeg->vfd_encoder);
	vb2_dma_contig_cleanup_ctx(jpeg->alloc_ctx);
	v4l2_m2m_release(jpeg->m2m_dev);
	v4l2_device_unregister(&jpeg->v4l2_dev);

	clk_disable_unprepare(jpeg->clk);
	clk_put(jpeg->clk);

	return 0;
}

static int x1000_jpeg_runtime_suspend(struct device *dev)
{
        // TODO
	return 0;
}

static int x1000_jpeg_runtime_resume(struct device *dev)
{
        // TODO
	return 0;
}

static const struct dev_pm_ops x1000_jpeg_pm_ops = {
	.runtime_suspend = x1000_jpeg_runtime_suspend,
	.runtime_resume	 = x1000_jpeg_runtime_resume,
};

static struct platform_driver x1000_jpeg_driver = {
	.probe = x1000_jpeg_probe,
	.remove = x1000_jpeg_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = X1000_JPEG_M2M_NAME,
		.pm = &x1000_jpeg_pm_ops,
	},
};

module_platform_driver(x1000_jpeg_driver);

MODULE_DESCRIPTION("Ingenic JPEG codec driver");
MODULE_LICENSE("GPL");
