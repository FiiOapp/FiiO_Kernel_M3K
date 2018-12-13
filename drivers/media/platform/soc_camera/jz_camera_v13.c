/*
 * V4L2 Driver for Ingenic jz camera (CIM) host
 *
 * Copyright (C) 2014, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * This program is support Continuous physical address mapping,
 * not support sg mode now.
 */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <asm/dma.h>
#include <media/videobuf2-dma-contig.h>
/*kzalloc*/
#include <mach/jz_camera.h>

#include <linux/regulator/consumer.h>

/* #define CIM_DEBUG_FPS */
/* #define PRINT_CIM_REG */
struct regulator *regul;
static int frame_size_check_flag = 1;

static ssize_t cim_dump_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct jz_camera_dev *pcdev = dev_get_drvdata(dev);
	if(pcdev == NULL) {
		printk("===>>%s,%d pcdev is NULL!\n",__func__,__LINE__);
	        return -EINVAL;
	}
#define STRING  "\t=\t0x%08x\n"
	printk("REG_CIM_CFG" STRING, readl(pcdev->base + CIM_CFG));
	printk("REG_CIM_CTRL" STRING, readl(pcdev->base + CIM_CTRL));
	printk("REG_CIM_CTRL2" STRING, readl(pcdev->base + CIM_CTRL2));
	printk("REG_CIM_STATE" STRING, readl(pcdev->base + CIM_STATE));

	printk("REG_CIM_IMR" STRING, readl(pcdev->base + CIM_IMR));
	printk("REG_CIM_IID" STRING, readl(pcdev->base + CIM_IID));
	printk("REG_CIM_DA" STRING, readl(pcdev->base + CIM_DA));
	printk("REG_CIM_FA" STRING, readl(pcdev->base + CIM_FA));

	printk("REG_CIM_FID" STRING, readl(pcdev->base + CIM_FID));
	printk("REG_CIM_CMD" STRING, readl(pcdev->base + CIM_CMD));
	printk("REG_CIM_WSIZE" STRING, readl(pcdev->base + CIM_SIZE));
	printk("REG_CIM_WOFFSET" STRING, readl(pcdev->base + CIM_OFFSET));

	printk("REG_CIM_FS" STRING, readl(pcdev->base + CIM_FS));
        return 0;
}

static int jz_camera_querycap(struct soc_camera_host *ici, struct v4l2_capability *cap)
{
	strlcpy(cap->card, "jz-Camera", sizeof(cap->card));
	cap->version = VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static unsigned int jz_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}
static int jz_camera_alloc_desc(struct jz_camera_dev *pcdev, unsigned int count)
{
	struct jz_camera_dma_desc *dma_desc_paddr;
	struct jz_camera_dma_desc *dma_desc;

	pcdev->buf_cnt = count;
	pcdev->desc_vaddr = dma_alloc_coherent(pcdev->soc_host.v4l2_dev.dev,
			sizeof(*pcdev->dma_desc) * pcdev->buf_cnt,
			(dma_addr_t *)&pcdev->dma_desc, GFP_KERNEL);

	dma_desc_paddr = (struct jz_camera_dma_desc *) pcdev->dma_desc;
	dma_desc = (struct jz_camera_dma_desc *) pcdev->desc_vaddr;

	if (!pcdev->dma_desc)
		return -ENOMEM;

	return 0;
}

static void jz_dma_free_desc(struct jz_camera_dev *pcdev)
{
	if(pcdev && pcdev->desc_vaddr) {
		dma_free_coherent(pcdev->soc_host.v4l2_dev.dev,
				sizeof(*pcdev->dma_desc) * pcdev->buf_cnt,
				pcdev->desc_vaddr, (dma_addr_t )pcdev->dma_desc);

		pcdev->desc_vaddr = NULL;
	}
}
static int jz_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[]){
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct jz_camera_dev *pcdev = ici->priv;
	int size;
	size = icd->sizeimage;

	if (!*nbuffers || *nbuffers > MAX_BUFFER_NUM)
		*nbuffers = MAX_BUFFER_NUM;

	if (size * *nbuffers > MAX_VIDEO_MEM)
		*nbuffers = MAX_VIDEO_MEM / size;

	*nplanes = 1;
	sizes[0] = size;
	alloc_ctxs[0] = pcdev->alloc_ctx;

	pcdev->start_streaming_called = 0;
	pcdev->sequence = 0;
	pcdev->active = NULL;


	if(jz_camera_alloc_desc(pcdev, *nbuffers))
		return -ENOMEM;
	dev_dbg(icd->parent, "%s, count=%d, size=%d\n", __func__,
		*nbuffers, size);

	return 0;
}
static int jz_init_dma(struct vb2_buffer *vb2) {
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb2->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct jz_camera_dev *pcdev = ici->priv;
	struct jz_camera_dma_desc *dma_desc;
	u32 index = vb2->v4l2_buf.index;

	dma_desc = (struct jz_camera_dma_desc *) pcdev->desc_vaddr;

	dma_desc[index].id = index;
	dma_desc[index].buf = vb2_dma_contig_plane_dma_addr(vb2, 0);

	/* jz_camera_v13 support color format YUV422, RAW8.
	 *
	 * icd->current_fmt->host_fmt->fourcc == V4L2_PIX_FMT_YUY || V4L2_PIX_FMT_SBGGR8
	 * */
	dma_desc[index].cmd = icd->sizeimage >> 2 | CIM_CMD_EOFINT;
#ifndef CONFIG_ENABLE_SOFT_OVERFLOW
	dma_desc[index].cmd |= CIM_CMD_OFRCV;
#endif


	if(index == 0) {
		pcdev->dma_desc_head = (struct jz_camera_dma_desc *)(pcdev->desc_vaddr);
	}

	if(index == (pcdev->buf_cnt - 1)) {
		dma_desc[index].next = 0;
		dma_desc[index].cmd |= CIM_CMD_STOP;

		pcdev->dma_desc_tail = (struct jz_camera_dma_desc *)(&dma_desc[index]);
	} else {
		dma_desc[index].next = (dma_addr_t) (&pcdev->dma_desc[index + 1]);
	}

	dev_dbg(icd->parent, "cim dma desc[index] address is: 0x%x\n", dma_desc[index].buf);

	return 0;
}

static int check_platform_param(struct jz_camera_dev *pcdev,
			       unsigned char buswidth, unsigned long *flags)
{
	/*
	 * Platform specified synchronization and pixel clock polarities are
	 * only a recommendation and are only used during probing. The PXA270
	 * quick capture interface supports both.
	 */
	*flags = V4L2_MBUS_MASTER |
		V4L2_MBUS_HSYNC_ACTIVE_HIGH |
		V4L2_MBUS_HSYNC_ACTIVE_LOW |
		V4L2_MBUS_VSYNC_ACTIVE_HIGH |
		V4L2_MBUS_VSYNC_ACTIVE_LOW |
		V4L2_MBUS_DATA_ACTIVE_HIGH |
		V4L2_MBUS_PCLK_SAMPLE_RISING |
		V4L2_MBUS_PCLK_SAMPLE_FALLING;

	return 0;

}
static int jz_buffer_init(struct vb2_buffer *vb2)
{
	struct jz_buffer *buf = container_of(vb2, struct jz_buffer, vb2);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}
static void jz_buffer_queue(struct vb2_buffer *vb2)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb2->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct jz_camera_dev *pcdev = ici->priv;
	struct jz_buffer *buf = container_of(vb2, struct jz_buffer, vb2);
	unsigned long flags;
	struct jz_camera_dma_desc *dma_desc_vaddr;
	struct jz_camera_dma_desc *dma_desc_paddr;
	unsigned int regval = 0;
	int index = vb2->v4l2_buf.index;
	spin_lock_irqsave(&pcdev->lock, flags);

	list_add_tail(&buf->list, &pcdev->video_buffer_list);
	if (pcdev->active == NULL) {
		pcdev->active = buf;
	}

	if(!pcdev->start_streaming_called) {
		goto out;
	}
	/* judged index */
	if((index > pcdev->buf_cnt) || (index < 0)) {
		dev_err(icd->parent,"Warning: %s, %d, index > pcdev->buf_cnt || index < 0, please check index !!!\n",
				 __func__, index);
		goto out;
	}

	dma_desc_vaddr = (struct jz_camera_dma_desc *) pcdev->desc_vaddr;

	if(pcdev->dma_desc_head != pcdev->dma_desc_tail) {
		dma_desc_vaddr[index].cmd |= CIM_CMD_STOP;

		pcdev->dma_desc_tail->next = (dma_addr_t) (&pcdev->dma_desc[index]);
		pcdev->dma_desc_tail->cmd &= (~CIM_CMD_STOP);

		pcdev->dma_desc_tail = (struct jz_camera_dma_desc *)(&dma_desc_vaddr[index]);
	} else {
		if(pcdev->dma_stopped) {
			dma_desc_vaddr[index].cmd |= CIM_CMD_STOP;

			pcdev->dma_desc_head = (struct jz_camera_dma_desc *)(&dma_desc_vaddr[index]);
			pcdev->dma_desc_tail = (struct jz_camera_dma_desc *)(&dma_desc_vaddr[index]);

			dma_desc_paddr = (struct jz_camera_dma_desc *) pcdev->dma_desc;

			/* Configure register CIMDA */
			regval = (unsigned int) (&dma_desc_paddr[index]);
			writel(regval, pcdev->base + CIM_DA);

			pcdev->dma_stopped = 0;

		} else {
			dma_desc_vaddr[index].cmd |= CIM_CMD_STOP;

			pcdev->dma_desc_tail->next = (dma_addr_t) (&pcdev->dma_desc[index]);
			pcdev->dma_desc_tail->cmd &= (~CIM_CMD_STOP);

			pcdev->dma_desc_tail = (struct jz_camera_dma_desc *)(&dma_desc_vaddr[index]);
		}
	}

out:
	spin_unlock_irqrestore(&pcdev->lock, flags);
	return;

}

static	int jz_buffer_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	int ret = 0;

	dev_vdbg(icd->parent, "%s (vb=0x%p) 0x%p %lu\n", __func__,
		vb, vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

	vb2_set_plane_payload(vb, 0, icd->sizeimage);
	if (vb2_plane_vaddr(vb, 0) &&
	    vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)) {
		ret = -EINVAL;
		return ret;
	}

	return 0;

}
static void jz_dma_start(struct jz_camera_dev *pcdev)
{
	unsigned int regval = 0;
	unsigned int temp = 0;
	writel(0, pcdev->base + CIM_STATE);

	/* please enable dma first, enable cim ctrl later.
	 * if enable cim ctrl first, RXFIFO can easily overflow.
	 */
	regval = (unsigned int)(pcdev->dma_desc);
	writel(regval, pcdev->base + CIM_DA);

	/*enable dma*/
	temp = readl(pcdev->base + CIM_CTRL);
	temp |= CIM_CTRL_DMA_EN;
	writel(temp, pcdev->base + CIM_CTRL);

	/* clear rx fifo */
	temp = readl(pcdev->base + CIM_CTRL);
	temp |= CIM_CTRL_RXF_RST;
	writel(temp, pcdev->base + CIM_CTRL);

	temp = readl(pcdev->base + CIM_CTRL);
	temp &= ~CIM_CTRL_RXF_RST;
	writel(temp, pcdev->base + CIM_CTRL);

	/* enable cim */
	temp = readl(pcdev->base + CIM_CTRL);
	temp |= CIM_CTRL_ENA;
	writel(temp, pcdev->base + CIM_CTRL);

}
static void jz_dma_stop(struct jz_camera_dev *pcdev)
{
	unsigned long temp = 0;

	/* unmask all interrupts. */
	writel(0xffffffff, pcdev->base + CIM_IMR);

	/* clear status register */
	writel(0, pcdev->base + CIM_STATE);

	/* clear rx fifo */
	temp = readl(pcdev->base + CIM_CTRL);
	temp |= CIM_CTRL_RXF_RST;
	writel(temp, pcdev->base + CIM_CTRL);

	writel(0, pcdev->base + CIM_STATE);

	/* disable dma & cim */
	temp = readl(pcdev->base + CIM_CTRL);
	temp &= ~(CIM_CTRL_ENA | CIM_CTRL_DMA_EN);
	writel(temp, pcdev->base + CIM_CTRL);
}
static int jz_start_streaming(struct vb2_queue *q, unsigned int count)
{
	int ret;
	struct soc_camera_device *icd = soc_camera_from_vb2q(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct jz_camera_dev *pcdev = ici->priv;
	struct jz_buffer *buf, *node;
	unsigned long flags;

	list_for_each_entry_safe(buf, node, &pcdev->video_buffer_list, list) {
		ret = jz_init_dma(&buf->vb2);
		if(ret) {
			dev_err(icd->parent,"%s:DMA initialization for Y/RGB failed\n", __func__);
			return ret;
		}
	}

	spin_lock_irqsave(&pcdev->lock, flags);

	jz_dma_start(pcdev);

#ifdef PRINT_CIM_REG
	cim_dump_reg(pcdev->dev);
#endif
	pcdev->dma_stopped = 0;
	pcdev->start_streaming_called = 1;

	spin_unlock_irqrestore(&pcdev->lock, flags);


	return 0;
}

static int jz_stop_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct jz_camera_dev *pcdev = ici->priv;
	struct jz_buffer *buf, *node;
	unsigned long flags;

	spin_lock_irqsave(&pcdev->lock, flags);
	jz_dma_stop(pcdev);

	/* Release all active buffers */
	list_for_each_entry_safe(buf, node, &pcdev->video_buffer_list, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb2, VB2_BUF_STATE_ERROR);
	}

	pcdev->start_streaming_called = 0;
	pcdev->dma_stopped = 1;
	pcdev->active = NULL;

	jz_dma_free_desc(pcdev);
	spin_unlock_irqrestore(&pcdev->lock, flags);
	return 0;

}
static struct vb2_ops jz_videobuf2_ops = {
	.buf_init		= jz_buffer_init,
	.queue_setup		= jz_queue_setup,
	.buf_prepare		= jz_buffer_prepare,
	.buf_queue		= jz_buffer_queue,
	.start_streaming	= jz_start_streaming,
	.stop_streaming		= jz_stop_streaming,
	.wait_prepare		= soc_camera_unlock,
	.wait_finish		= soc_camera_lock,
};
static int jz_camera_init_videobuf2(struct vb2_queue *q, struct soc_camera_device *icd) {

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = icd;
	q->buf_struct_size = sizeof(struct jz_buffer);
	q->ops = &jz_videobuf2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	return vb2_queue_init(q);
}

static int jz_camera_try_fmt(struct soc_camera_device *icd, struct v4l2_format *f) {
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	__u32 pixfmt = pix->pixelformat;
	int ret;
	/* TODO: limit to jz hardware capabilities */

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_err(icd->parent,"Format %x not found\n", pix->pixelformat);
		return -EINVAL;
	}

	v4l_bound_align_image(&pix->width, 48, 2048, 1,
			&pix->height, 32, 2048, 0,
			pixfmt == V4L2_PIX_FMT_YUV422P ? 4 : 0);


	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;


	/* limit to sensor capabilities */
	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width	= mf.width;
	pix->height	= mf.height;
	pix->field	= mf.field;
	pix->colorspace	= mf.colorspace;

	return 0;
}

static int jz_camera_set_fmt(struct soc_camera_device *icd, struct v4l2_format *f) {
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret, buswidth;
	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_err(icd->parent, "Format %x not found\n", pix->pixelformat);
		return -EINVAL;
	}

	buswidth = xlate->host_fmt->bits_per_sample;
	if (buswidth > 8) {
		dev_warn(icd->parent, "bits-per-sample %d for format %x unsupported\n",
				buswidth, pix->pixelformat);
		return -EINVAL;
	}

	mf.width        = pix->width;
	mf.height       = pix->height;
	mf.field        = pix->field;
	mf.colorspace   = pix->colorspace;
	mf.code         = xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	if (mf.code != xlate->code)
		return -EINVAL;

	pix->width              = mf.width;
	pix->height             = mf.height;
	pix->field              = mf.field;
	pix->colorspace         = mf.colorspace;

	icd->current_fmt        = xlate;

	return ret;
}

static int jz_camera_set_crop(struct soc_camera_device *icd, const struct v4l2_crop *a) {
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	return v4l2_subdev_call(sd, video, s_crop, a);
}

static int jz_camera_set_bus_param(struct soc_camera_device *icd) {
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct jz_camera_dev *pcdev = ici->priv;
	struct v4l2_mbus_config cfg = {.type = V4L2_MBUS_PARALLEL,};
	u32 pixfmt = icd->current_fmt->host_fmt->fourcc;
	unsigned long common_flags;
	unsigned long bus_flags = 0;
	unsigned long cfg_reg = 0;
	unsigned long ctrl_reg = 0;
	unsigned long ctrl2_reg = 0;
	unsigned long fs_reg = 0;
	unsigned long temp = 0;
	int ret;

	ret = check_platform_param(pcdev, icd->current_fmt->host_fmt->bits_per_sample,
			&bus_flags);

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret) {
		common_flags = soc_mbus_config_compatible(&cfg,
				bus_flags);
		if (!common_flags) {
			dev_warn(icd->parent,
					"Flags incompatible: camera 0x%x, host 0x%lx\n",
					cfg.flags, bus_flags);
			return -EINVAL;
		}
	} else if (ret != -ENOIOCTLCMD) {
		return ret;
	} else {
		common_flags = bus_flags;
	}

	/* Make choises, based on platform preferences */

	if ((common_flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH) &&
	    (common_flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)) {
		if (pcdev->pdata->flags & JZ_CAMERA_VSYNC_HIGH)
			common_flags &= ~V4L2_MBUS_VSYNC_ACTIVE_HIGH;
		else
			common_flags &= ~V4L2_MBUS_VSYNC_ACTIVE_LOW;
	}

	if ((common_flags & V4L2_MBUS_PCLK_SAMPLE_RISING) &&
	    (common_flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)) {
		if (pcdev->pdata->flags & JZ_CAMERA_PCLK_RISING)
			common_flags &= ~V4L2_MBUS_PCLK_SAMPLE_FALLING;
		else
			common_flags &= ~V4L2_MBUS_PCLK_SAMPLE_RISING;
	}

	if ((common_flags & V4L2_MBUS_DATA_ACTIVE_HIGH) &&
	    (common_flags & V4L2_MBUS_DATA_ACTIVE_LOW)) {
		if (pcdev->pdata->flags & JZ_CAMERA_DATA_HIGH)
			common_flags &= ~V4L2_MBUS_DATA_ACTIVE_LOW;
		else
			common_flags &= ~V4L2_MBUS_DATA_ACTIVE_HIGH;
	}

	cfg.flags = common_flags;
	ret = v4l2_subdev_call(sd, video, s_mbus_config, &cfg);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		dev_dbg(icd->parent, "camera s_mbus_config(0x%lx) returned %d\n",
			common_flags, ret);
		return ret;
	}

	/*PCLK Polarity Set*/
	cfg_reg = (common_flags & V4L2_MBUS_PCLK_SAMPLE_RISING) ?
		cfg_reg | CIM_CFG_PCP_HIGH : cfg_reg & (~CIM_CFG_PCP_HIGH);

	/*VSYNC Polarity Set*/
	cfg_reg = (common_flags & V4L2_MBUS_VSYNC_ACTIVE_LOW) ?
		cfg_reg | CIM_CFG_VSP_HIGH : cfg_reg & (~CIM_CFG_VSP_HIGH);

	/*HSYNC Polarity Set*/
	cfg_reg = (common_flags & V4L2_MBUS_HSYNC_ACTIVE_LOW) ?
		cfg_reg | CIM_CFG_HSP_HIGH : cfg_reg & (~CIM_CFG_HSP_HIGH);

	cfg_reg |= CIM_CFG_DMA_BURST_INCR64 | CIM_CFG_DSM_GCM | CIM_CFG_PACK_Y0UY1V;

	ctrl_reg |= CIM_CTRL_DMA_SYNC | CIM_CTRL_FRC_1;

	ctrl2_reg |= CIM_CTRL2_APM | CIM_CTRL2_OPE |
		(1 << CIM_CTRL2_OPG_BIT) | CIM_CTRL2_ARIF;

        if (frame_size_check_flag)
                ctrl2_reg |= CIM_CTRL2_FSC;

	fs_reg = (icd->user_width -1) << CIM_FS_FHS_BIT | (icd->user_height -1)
		<< CIM_FS_FVS_BIT | 1 << CIM_FS_BPP_BIT;

	if(pixfmt == V4L2_PIX_FMT_SBGGR8) {
		fs_reg = (icd->user_width -1) << CIM_FS_FHS_BIT | (icd->user_height -1)
			<< CIM_FS_FVS_BIT | 0 << CIM_FS_BPP_BIT;
	}

	/*BS0 BS1 BS2 BS3 must be 00,01,02,03 when pack is b100*/

	if(cfg_reg & CIM_CFG_PACK_Y0UY1V)
		cfg_reg |= CIM_CFG_BS1_2_OBYT1 | CIM_CFG_BS2_2_OBYT2 | CIM_CFG_BS3_2_OBYT3;

	writel(cfg_reg, pcdev->base + CIM_CFG);
	writel(ctrl_reg, pcdev->base + CIM_CTRL);
	writel(ctrl2_reg, pcdev->base + CIM_CTRL2);
	writel(fs_reg, pcdev->base + CIM_FS);

	/* enable end of frame interrupt */
	temp = readl(pcdev->base + CIM_IMR);
	temp &= (~CIM_IMR_EOFM);
	writel(temp, pcdev->base + CIM_IMR);

	/* enable stop of frame interrupt */
	temp = readl(pcdev->base + CIM_IMR);
	temp &= (~CIM_IMR_STPM);
	writel(temp, pcdev->base + CIM_IMR);
#ifdef CONFIG_ENABLE_SOFT_OVERFLOW
	/* enable rx overflow interrupt */
	temp = readl(pcdev->base + CIM_IMR);
	temp &= (~CIM_IMR_RFIFO_OFM);
	writel(temp, pcdev->base + CIM_IMR);
#endif
	temp = readl(pcdev->base + CIM_IMR);
	temp &= (~CIM_IMR_STPM_1);
	writel(temp, pcdev->base + CIM_IMR);

	return 0;
}
static void jz_camera_activate(struct jz_camera_dev *pcdev) {
	int ret = -1;


	if(pcdev->clk) {
		ret = clk_enable(pcdev->clk);
	}
	if(pcdev->mclk) {
		ret = clk_set_rate(pcdev->mclk, pcdev->mclk_freq);
		ret = clk_enable(pcdev->mclk);
	}
	if(regul)
		regulator_enable(regul);
	if(ret) {
		dev_err(NULL, "enable clock failed!\n");
	}

	msleep(10);
}
static void jz_camera_deactivate(struct jz_camera_dev *pcdev) {

	if(pcdev->clk) {
		clk_disable(pcdev->clk);
	}
	if(pcdev->mclk) {
		clk_disable(pcdev->mclk);
	}
	if(regul)
		regulator_disable(regul);
}

static int jz_camera_add_device(struct soc_camera_device *icd) {
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct jz_camera_dev *pcdev = ici->priv;

	int icd_index = icd->devnum;

	if (pcdev->icd[icd_index])
		return -EBUSY;

	dev_dbg(icd->parent, "jz Camera driver attached to camera %d\n",
			icd->devnum);

	pcdev->icd[icd_index] = icd;

	jz_camera_activate(pcdev);

	return 0;
}
static void jz_camera_remove_device(struct soc_camera_device *icd) {
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct jz_camera_dev *pcdev = ici->priv;

	int icd_index = icd->devnum;

	BUG_ON(icd != pcdev->icd[icd_index]);

	jz_camera_deactivate(pcdev);
	dev_dbg(icd->parent, "jz Camera driver detached from camera %d\n",
			icd->devnum);

	pcdev->icd[icd_index] = NULL;
}

static irqreturn_t jz_camera_irq_handler(int irq, void *data) {
	struct jz_camera_dev *pcdev = (struct jz_camera_dev *)data;
	unsigned long status = 0, temp = 0;
	unsigned long flags = 0;
	int index = 0, regval = 0;
	struct jz_camera_dma_desc *dma_desc_paddr;
	for (index = 0; index < ARRAY_SIZE(pcdev->icd); index++) {
		if (pcdev->icd[index]) {
			break;
		}
	}

	if(index == MAX_SOC_CAM_NUM)
		return IRQ_HANDLED;

	/* judged pcdev->dma_desc_head->id */
	if((pcdev->dma_desc_head->id > pcdev->buf_cnt) || (pcdev->dma_desc_head->id < 0)) {
		dev_dbg(NULL, "Warning: %s, %d, pcdev->dma_desc_head->id >pcdev->buf_cnt ||pcdev->dma_desc_head->id < 0, please check pcdev->dma_desc_head->id !!!\n",__func__, pcdev->dma_desc_head->id);
		return IRQ_NONE;
	}

	spin_lock_irqsave(&pcdev->lock, flags);

	/* read interrupt status register */
	status = readl(pcdev->base + CIM_STATE);
	if (!status) {
		dev_err(NULL, "status is NULL! \n");
		spin_unlock_irqrestore(&pcdev->lock, flags);

		return IRQ_NONE;
	}

	if(!(status & CIM_STATE_RXOF_STOP_EOF)) {
		/* other irq */
		dev_dbg(NULL, "irq_handle status is 0x%lx, not judged in irq_handle\n", status);

		spin_unlock_irqrestore(&pcdev->lock, flags);
		return IRQ_HANDLED;
	}
#ifdef CONFIG_ENABLE_SOFT_OVERFLOW
	if (status & CIM_STATE_RXF_OF) {
		dev_dbg(pcdev->icd->parent, "RX FIFO OverFlow interrupt!\n");

		/* clear rx overflow interrupt */
		temp = readl(pcdev->base + CIM_STATE);
		temp &= ~CIM_STATE_RXF_OF;
		writel(temp, pcdev->base + CIM_STATE);

		/* disable cim */
		temp = readl(pcdev->base + CIM_CTRL);
		temp &= ~CIM_CTRL_ENA;
		writel(temp, pcdev->base + CIM_CTRL);

		/* clear rx fifo */
		temp = readl(pcdev->base + CIM_CTRL);
		temp |= CIM_CTRL_RXF_RST;
		writel(temp, pcdev->base + CIM_CTRL);

		temp = readl(pcdev->base + CIM_CTRL);
		temp &= ~CIM_CTRL_RXF_RST;
		writel(temp, pcdev->base + CIM_CTRL);

		/* clear status register */
		writel(0, pcdev->base + CIM_STATE);

		/* enable cim */
		temp = readl(pcdev->base + CIM_CTRL);
		temp |= CIM_CTRL_ENA;
		writel(temp, pcdev->base + CIM_CTRL);

		spin_unlock_irqrestore(&pcdev->lock, flags);
		return IRQ_HANDLED;
	}
#endif
	if(status & CIM_STATE_DMA_STOP) {
		/* clear dma interrupt status */
		temp = readl(pcdev->base + CIM_STATE);
		temp &= (~CIM_STATE_DMA_STOP);
		writel(temp, pcdev->base + CIM_STATE);
		pcdev->dma_stopped = 1;
	}

	if(status & CIM_STATE_DMA_EOF) {
		/* clear dma interrupt status */
		temp = readl(pcdev->base + CIM_STATE);
		temp &= (~CIM_STATE_DMA_EOF);
		writel(temp, pcdev->base + CIM_STATE);

		if(pcdev->active) {
			struct vb2_buffer *vb2 = &pcdev->active->vb2;
			struct jz_buffer *buf = container_of(vb2, struct jz_buffer, vb2);

			list_del_init(&buf->list);
			v4l2_get_timestamp(&vb2->v4l2_buf.timestamp);
			vb2->v4l2_buf.sequence = pcdev->sequence++;
			vb2_buffer_done(vb2, VB2_BUF_STATE_DONE);
#ifdef CIM_DEBUG_FPS
			if((vb2->v4l2_buf.sequence % 60) == 0)  {
				pcdev->debug_ms_start = ktime_to_ms(ktime_get_real());
			} else if((vb2->v4l2_buf.sequence % 60) == 59) {
				long long debug_ms_end = ktime_to_ms(ktime_get_real());
				long long ms = debug_ms_end - pcdev->debug_ms_start;

				long long fps = 60 * 1000;
				do_div(fps, ms);

				printk("===fps: %lld, start: %lld, end: %lld, sequence: %d\n",\
						fps, pcdev->debug_ms_start, debug_ms_end, pcdev->sequence);
			}
#endif


		}

		if (list_empty(&pcdev->video_buffer_list)) {
			pcdev->active = NULL;
			spin_unlock_irqrestore(&pcdev->lock, flags);
			return IRQ_HANDLED;
		}

		if(pcdev->dma_desc_head != pcdev->dma_desc_tail) {
			pcdev->dma_desc_head =
				(struct jz_camera_dma_desc *)UNCAC_ADDR(phys_to_virt(pcdev->dma_desc_head->next));
		}
		if(pcdev->dma_stopped && !list_empty(&pcdev->video_buffer_list)) {
			/* dma stop condition:
			 * 1. dma desc reach the end, and there is no more desc to be transferd.
			 *    dma need to stop.
			 * 2. if the capture list not empty, we should restart dma here.
			 *
			 * */

			dma_desc_paddr = (struct jz_camera_dma_desc *) pcdev->dma_desc;
			regval = (unsigned int) (&dma_desc_paddr[pcdev->dma_desc_head->id]);
			writel(regval, pcdev->base + CIM_DA);

			pcdev->dma_stopped = 0;
		}
		/* start next dma frame. */
		pcdev->active = list_entry(pcdev->video_buffer_list.next, struct jz_buffer, list);

		spin_unlock_irqrestore(&pcdev->lock, flags);

		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&pcdev->lock, flags);

	return IRQ_HANDLED;
}

static ssize_t frame_size_check_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	snprintf(buf, 100, " 0: disable frame size check.\n 1: enable frame size check.\n");
	return 100;
}

static ssize_t frame_size_check_w(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        if (count != 2)
                return -EINVAL;

        if (*buf == '0') {
                frame_size_check_flag = 0;
        } else if (*buf == '1') {
                frame_size_check_flag = 1;
        } else {
                return -EINVAL;
        }

        return count;
}

/**********************cim_debug***************************/
static DEVICE_ATTR(dump_cim_reg, S_IRUGO|S_IWUSR, cim_dump_reg, NULL);
static DEVICE_ATTR(frame_size_check, S_IRUGO|S_IWUSR, frame_size_check_r, frame_size_check_w);

static struct attribute *cim_debug_attrs[] = {
	&dev_attr_dump_cim_reg.attr,
	&dev_attr_frame_size_check.attr,
	NULL,
};

const char cim_group_name[] = "debug";
static struct attribute_group cim_debug_attr_group = {
	.name	= cim_group_name,
	.attrs	= cim_debug_attrs,
};



static struct soc_camera_host_ops jz_soc_camera_host_ops = {
	.owner = THIS_MODULE,
	.add = jz_camera_add_device,
	.remove = jz_camera_remove_device,
	.set_fmt = jz_camera_set_fmt,
	.try_fmt = jz_camera_try_fmt,
	.init_videobuf2 = jz_camera_init_videobuf2,
	.poll = jz_camera_poll,
	.querycap = jz_camera_querycap,
	.set_crop = jz_camera_set_crop,
	.set_bus_param = jz_camera_set_bus_param,
};

static int __init jz_camera_probe(struct platform_device *pdev) {
	int err = 0, ret = 0;
	unsigned int irq;
	struct resource *res;
	void __iomem *base;
	struct jz_camera_dev *pcdev;

	/* malloc */
	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto err_kzalloc;
	}

	/* resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		dev_err(&pdev->dev, "Could not get resource!\n");
		err = -ENODEV;
		goto err_get_resource;
	}

	/* irq */
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "Could not get irq!\n");
		err = -ENODEV;
		goto err_get_irq;
	}
	/*get cim1 clk*/
	pcdev->clk = clk_get(&pdev->dev, "cim");
	if (IS_ERR(pcdev->clk)) {
		err = PTR_ERR(pcdev->clk);
		dev_err(&pdev->dev, "%s:can't get clk %s\n", __func__, "cim");
		goto err_clk_get_cim1;
	}

	/*get cgu_cimmclk1 clk*/
	pcdev->mclk = clk_get(&pdev->dev, "cgu_cim");
	if (IS_ERR(pcdev->mclk)) {
		err = PTR_ERR(pcdev->mclk);
		dev_err(&pdev->dev, "%s:can't get clk %s\n",
				__func__, "cgu_cimmclk");
		goto err_clk_get_cgu_cimmclk1;
	}

	regul = regulator_get(NULL, CAMERA_GSENSOR_VCC);
	if(IS_ERR(regul)) {
		dev_err(&pdev->dev, "get regulator fail !, if you need regulator, please check this place!\n");
		err = -ENODEV;
		regul = NULL;
//		goto exit_put_clk_cim;
	}else {
		regulator_enable(regul);
	}

	pcdev->dev = &pdev->dev;
	pcdev->pdata = pdev->dev.platform_data;
	if (pcdev->pdata)
		pcdev->mclk_freq = pcdev->pdata->mclk_10khz * 10000;

	if (!pcdev->mclk_freq) {
		dev_warn(&pdev->dev, "mclk_10khz == 0! Please, fix your platform data."
				"Using default 24MHz\n");
		pcdev->mclk_freq = 24000000;
	}
	if (clk_get_rate(pcdev->mclk) >= pcdev->mclk_freq) {
		clk_set_rate(pcdev->mclk, pcdev->mclk_freq);
	} else {
		clk_set_rate(pcdev->mclk, pcdev->mclk_freq);
	}


	/* Request the regions. */
	if (!request_mem_region(res->start, resource_size(res), DRIVER_NAME)) {
		err = -EBUSY;
		goto err_request_mem_region;
	}
	base = ioremap(res->start, resource_size(res));
	if (!base) {
		err = -ENOMEM;
		goto err_ioremap;
	}

	spin_lock_init(&pcdev->lock);
	INIT_LIST_HEAD(&pcdev->video_buffer_list);

	pcdev->res = res;
	pcdev->irq = irq;
	pcdev->base = base;
	pcdev->active = NULL;

	/* request irq */
	err = request_irq(pcdev->irq, jz_camera_irq_handler, IRQF_DISABLED,
			dev_name(&pdev->dev), pcdev);
	if(err) {
		dev_err(&pdev->dev, "request irq failed!\n");
		goto err_request_irq;
	}

	pcdev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(pcdev->alloc_ctx)) {
		ret = PTR_ERR(pcdev->alloc_ctx);
		goto err_alloc_ctx;
	}
	pcdev->soc_host.drv_name        = DRIVER_NAME;
	pcdev->soc_host.ops             = &jz_soc_camera_host_ops;
	pcdev->soc_host.priv            = pcdev;
	pcdev->soc_host.v4l2_dev.dev    = &pdev->dev;
	pcdev->soc_host.nr              = 0; /* use one cim0 or cim1 */

	err = soc_camera_host_register(&pcdev->soc_host);
	if (err)
		goto err_soc_camera_host_register;

	if (!IS_ERR(regul))
		    regulator_disable(regul);

	ret = sysfs_create_group(&pcdev->dev->kobj, &cim_debug_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "device create sysfs group failed\n");

		ret = -EINVAL;
		goto err_free_file;
	}

	dev_dbg(&pdev->dev, "jz Camera driver loaded!\n");

	return 0;

err_free_file:
	sysfs_remove_group(&pcdev->dev->kobj, &cim_debug_attr_group);
err_soc_camera_host_register:
	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);
err_alloc_ctx:
	free_irq(pcdev->irq, pcdev);
err_request_irq:
	iounmap(base);
err_ioremap:
	release_mem_region(res->start, resource_size(res));
err_request_mem_region:
	clk_put(pcdev->mclk);
err_clk_get_cgu_cimmclk1:
	clk_put(pcdev->clk);
err_clk_get_cim1:
err_get_irq:
err_get_resource:
	kfree(pcdev);
err_kzalloc:
	return err;

}

static int __exit jz_camera_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct jz_camera_dev *pcdev = container_of(soc_host,
					struct jz_camera_dev, soc_host);
	struct resource *res;

	free_irq(pcdev->irq, pcdev);

	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);

	clk_put(pcdev->clk);
	clk_put(pcdev->mclk);

	soc_camera_host_unregister(soc_host);

	sysfs_remove_group(&pcdev->dev->kobj, &cim_debug_attr_group);

	iounmap(pcdev->base);

	res = pcdev->res;
	release_mem_region(res->start, resource_size(res));

	kfree(pcdev);

	dev_dbg(&pdev->dev, "jz Camera driver unloaded\n");

	return 0;
}

static struct platform_driver jz_camera_driver = {
	.remove		= __exit_p(jz_camera_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init jz_camera_init(void) {
	/*
	 * platform_driver_probe() can save memory,
	 * but this Driver can bind to one device only.
	 */
	return platform_driver_probe(&jz_camera_driver, jz_camera_probe);
}

static void __exit jz_camera_exit(void) {
	return platform_driver_unregister(&jz_camera_driver);
}

late_initcall(jz_camera_init);
module_exit(jz_camera_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("cxtan <chenxi.tan@ingenic.cn>");
MODULE_DESCRIPTION("jz Soc Camera Host Driver");
MODULE_ALIAS("a jz-cim platform");
