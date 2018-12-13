/*
 *  sound/soc/ingenic/asoc-dma.c
 *  ALSA Soc Audio Layer -- ingenic dmic dma platform driver base on ../asoc-v12/asoc-dma-v12.c
 *
 *  Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *	qipengzhen <aric.pzqi@ingenic.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/soc-dai.h>
#include <sound/pcm_params.h>
#include <mach/jzdma.h>
#include <linux/delay.h>



#include "../asoc-v12/asoc-dma-v12.h"

static int asoc_dma_debug = 0;
module_param(asoc_dma_debug, int, 0644);

#define DMA_DEBUG_MSG(msg...)			\
	do {					\
		if (asoc_dma_debug)		\
			printk(KERN_DEBUG"ADMA: " msg);	\
	} while(0)
#define DMA_SUBSTREAM_MSG(substream, msg...)	\
	do {					\
		if (asoc_dma_debug) {		\
			printk(KERN_DEBUG"ADMA[%s][%s]:", \
					substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? \
					"replay" : "record",	\
					substream->pcm->id);	\
			printk(KERN_DEBUG msg);		\
		} \
	} while(0)



extern dma_addr_t jz_dmic_module_get_trans_addr(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai);

/*
 * fake using a continuous buffer
 */
static inline void *
snd_pcm_get_ptr(struct snd_pcm_substream *substream, unsigned int ofs)
{
	return substream->runtime->dma_area + ofs;
}

static int jz_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct jz_pcm_runtime_data *prtd = substream->runtime->private_data;

	DMA_SUBSTREAM_MSG(substream, "%s enter\n", __func__);

	{
		unsigned long long time_ns;
		time_ns = 1000LL * 1000 * 1000 * params_period_size(params);
		do_div(time_ns, params_rate(params));
		prtd->expires = ns_to_ktime(time_ns);
	}
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
}

static void debug_work(struct work_struct *debug_work)
{
	struct jz_pcm_runtime_data *prtd =
		container_of(debug_work, struct jz_pcm_runtime_data, debug_work);
#if defined(CONFIG_JZ_ASOC_DMA_AUTO_CLR_DRT_MEM)
	struct snd_pcm_substream *substream = prtd->substream;
#endif
	if (!IS_ERR_OR_NULL(prtd->file)) {
		prtd->old_fs = get_fs();
		set_fs(KERNEL_DS);
		vfs_write(prtd->file, prtd->copy_start,
				prtd->copy_length,
				&prtd->file_offset);
		prtd->file_offset = prtd->file->f_pos;
		set_fs(prtd->old_fs);
	}
#if defined(CONFIG_JZ_ASOC_DMA_AUTO_CLR_DRT_MEM)
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		memset(prtd->copy_start, 0, prtd->copy_length);
	}
#endif
}

static size_t
snd_pcm_get_pos_algin_period(struct snd_pcm_substream *substream, dma_addr_t addr)
{
	return (addr - substream->runtime->dma_addr -
			(addr - substream->runtime->dma_addr)%
			snd_pcm_lib_period_bytes(substream));
}

static size_t
snd_pcm_get_pos(struct snd_pcm_substream *substream, dma_addr_t addr)
{
	return (addr - substream->runtime->dma_addr);
}

static enum hrtimer_restart jz_asoc_hrtimer_callback(struct hrtimer *hr_timer) {
	struct jz_pcm_runtime_data *prtd = container_of(hr_timer,
			struct jz_pcm_runtime_data, hr_timer);
	struct snd_pcm_substream *substream = prtd->substream;

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	dma_addr_t pdma_addr = 0;
	size_t buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	size_t curr_pos = 0;


	if (atomic_read(&prtd->stopped))
		goto out;


	hrtimer_start(&prtd->hr_timer, prtd->expires , HRTIMER_MODE_REL);


	pdma_addr = jz_dmic_module_get_trans_addr(substream, cpu_dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		curr_pos = snd_pcm_get_pos_algin_period(substream, pdma_addr);
		if (curr_pos == prtd->pos)
			goto out;
#ifdef CONFIG_JZ_ASOC_DMA_AUTO_CLR_DRT_MEM
		if (prtd->pos < curr_pos) {
			memset(snd_pcm_get_ptr(substream, prtd->pos), 0 , (curr_pos - prtd->pos));
		}
		if (prtd->pos > curr_pos) {
			memset(snd_pcm_get_ptr(substream, prtd->pos), 0, (buffer_bytes - prtd->pos));
			memset(snd_pcm_get_ptr(substream, 0), 0, curr_pos);
		}
#endif
		prtd->pos = curr_pos;
	} else {
		curr_pos = snd_pcm_get_pos(substream, pdma_addr);
		if (curr_pos == prtd->pos)
			goto out;
		prtd->pos = curr_pos;
	}
	//printk(KERN_DEBUG"curr_pos = %d buffer_bytes = %d\n", curr_pos, buffer_bytes);

	snd_pcm_period_elapsed(substream);
out:

	return HRTIMER_NORESTART;
}


static int jz_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct jz_pcm_runtime_data *prtd = substream->runtime->private_data;

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

#ifdef CONFIG_JZ_ASOC_DMA_AUTO_CLR_DRT_MEM
	size_t buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
#endif
	int ret;

	DMA_SUBSTREAM_MSG(substream,"%s enter cmd %d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		atomic_set(&prtd->stopped, 0);
		hrtimer_start(&prtd->hr_timer, prtd->expires , HRTIMER_MODE_REL);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		DMA_SUBSTREAM_MSG(substream,"stop trigger\n");
		atomic_set(&prtd->stopped, 1);
		/* To make sure there is not data transfer on AHB bus,
		 * then we can stop the dma, Wait tur or ror happen
		 */
		if (cpu_dai->driver->ops->trigger) {
			ret = cpu_dai->driver->ops->trigger(substream, cmd, cpu_dai);
			if (ret < 0)
				return ret;
		}

#ifdef CONFIG_JZ_ASOC_DMA_AUTO_CLR_DRT_MEM
		printk(KERN_DEBUG"show the time memset1 %d\n", buffer_bytes);
		memset(snd_pcm_get_ptr(substream, 0), 0, buffer_bytes);
		printk(KERN_DEBUG"show the time memset2 %d\n", buffer_bytes);
#endif
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t jz_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct jz_pcm_runtime_data *prtd = substream->runtime->private_data;

	DMA_DEBUG_MSG("%s: %d %ld\n", __func__, prtd->pos,
			bytes_to_frames(substream->runtime, prtd->pos));
	return bytes_to_frames(substream->runtime, prtd->pos);
}


#define JZ_DMA_BUFFERSIZE (64 * PAGE_SIZE)
static const struct snd_pcm_hardware jz_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S24_LE |
		SNDRV_PCM_FMTBIT_S20_3LE |
		SNDRV_PCM_FMTBIT_S18_3LE |
		SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S8,
	.rates                  = SNDRV_PCM_RATE_8000_192000,
	.rate_min               = 8000,
	.rate_max               = 192000,
	.channels_min           = 1,
	.channels_max           = 2,
	.buffer_bytes_max       = JZ_DMA_BUFFERSIZE,
	.period_bytes_min       = PAGE_SIZE/4,     /* 1K */
	.period_bytes_max       = PAGE_SIZE * 16, /* 64K */
	.periods_min            = 4,
	.periods_max            = 256,
	.fifo_size              = 0,
};

static int jz_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct jz_pcm_runtime_data *prtd = NULL;
	int ret;
#ifdef CONFIG_ANDROID
	char *file_dir = "/data";
#else
	char *file_dir = "/tmp";
#endif

	DMA_DEBUG_MSG("%s enter\n", __func__);
	ret = snd_soc_set_runtime_hwparams(substream, &jz_pcm_hardware);
	if (ret)
		return ret;

	ret = snd_pcm_hw_constraint_integer(substream->runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;

	atomic_set(&prtd->stopped, 0);
	hrtimer_init(&prtd->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	prtd->hr_timer.function = jz_asoc_hrtimer_callback;

	prtd->substream = substream;
	substream->runtime->private_data = prtd;

	prtd->file_name = kzalloc(40*sizeof(char), GFP_KERNEL);
	if (prtd->file_name) {
		sprintf(prtd->file_name, "%s/%sd%is%i.pcm",
				file_dir,
				substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
				"replay" : "record",
				substream->pcm->device,
				substream->number);
		dev_dbg(rtd->dev, "open debug file %s \n", prtd->file_name);
		prtd->file = filp_open(prtd->file_name,
				O_RDWR | O_APPEND, S_IRUSR | S_IWUSR | O_CREAT);
		if (IS_ERR_OR_NULL(prtd->file)) {
			prtd->file = NULL;
			goto out_pcm_open;
		}
		dev_warn(rtd->dev,"open debug %s success (Poor performance)\n",
				prtd->file_name);
		prtd->file_offset =prtd->file->f_pos;
		INIT_WORK(&prtd->debug_work, debug_work);
out_pcm_open:
		kfree(prtd->file_name);
		prtd->file_name = NULL;
	}
	return 0;
}

static int jz_pcm_close(struct snd_pcm_substream *substream)
{
	struct jz_pcm_runtime_data *prtd = substream->runtime->private_data;

	DMA_DEBUG_MSG("%s enter\n", __func__);
	hrtimer_cancel(&prtd->hr_timer);

	substream->runtime->private_data = NULL;
	if (prtd->file_name)
		kfree(prtd->file_name);
	if (!IS_ERR_OR_NULL(prtd->file)) {
		flush_work(&prtd->debug_work);
		filp_close(prtd->file, NULL);
	}
	kfree(prtd);
	return 0;
}

static struct snd_pcm_ops jz_pcm_ops = {
	.open		= jz_pcm_open,
	.close		= jz_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= jz_pcm_hw_params,
	.hw_free	= snd_pcm_lib_free_pages,
	.trigger	= jz_pcm_trigger,
	.pointer	= jz_pcm_pointer,
};


static void jz_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	int i;

	DMA_DEBUG_MSG("%s enter\n", __func__);

	for (i = SNDRV_PCM_STREAM_PLAYBACK; i <= SNDRV_PCM_STREAM_CAPTURE; i++) {
		substream = pcm->streams[i].substream;
		if (!substream)
			continue;
	}
	snd_pcm_lib_preallocate_free_for_all(pcm);
	return;
}

static int jz_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_pcm_substream *substream;
	size_t buffer_size = JZ_DMA_BUFFERSIZE;
	size_t buffer_bytes_max = JZ_DMA_BUFFERSIZE;
	int ret = -EINVAL;
	int i;

	DMA_DEBUG_MSG("%s enter\n", __func__);

	for (i = 0; i < 2; i++) {
		substream = pcm->streams[i].substream;
		if (!substream)
			continue;
		ret = snd_pcm_lib_preallocate_pages(substream,
				SNDRV_DMA_TYPE_DEV,
				rtd->dev,
				buffer_size,
				buffer_bytes_max);
	}
	if (ret)
		goto out;

	return 0;
out:
	dev_err(rtd->dev, "Failed to alloc dma buffer %d\n", ret);
	jz_pcm_free(pcm);
	return ret;
}

static struct snd_soc_platform_driver jz_pcm_platform = {
	.ops            = &jz_pcm_ops,
	.pcm_new        = jz_pcm_new,
	.pcm_free       = jz_pcm_free,
};

static int jz_pcm_platform_probe(struct platform_device *pdev)
{
	int ret;

	ret = snd_soc_register_platform(&pdev->dev, &jz_pcm_platform);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register platfrom\n");
		platform_set_drvdata(pdev, NULL);
		return ret;
	}
	dev_info(&pdev->dev, "Audio dmic dma platfrom probe success\n");
	return 0;
}

static int jz_pcm_platform_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Audio dma platfrom removed\n");
	snd_soc_unregister_platform(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}


static struct platform_driver jz_pcm_platfrom_driver = {
	.probe  = jz_pcm_platform_probe,
	.remove = jz_pcm_platform_remove,
	.driver = {
		.name   = "jz-asoc-dmic-module-dma",
		.owner  = THIS_MODULE,
	},
};

static int jz_pcm_init(void)
{
	struct platform_device *pdev = NULL;
	pdev = platform_device_register_simple("jz-asoc-dmic-module-dma", -1, NULL, 0);
	if(IS_ERR(pdev)) {
		printk("platform device register jz-dma-dmic failed!\n");
		return -ENOMEM;
	}
	printk("platform device register jz-dma-dmic success!\n");

	return platform_driver_register(&jz_pcm_platfrom_driver);
}
module_init(jz_pcm_init);

static void jz_pcm_exit(void)
{
	platform_driver_unregister(&jz_pcm_platfrom_driver);
}
module_exit(jz_pcm_exit);

MODULE_DESCRIPTION("JZ ASOC Platform driver for dmic use only");
MODULE_AUTHOR("qipengzhen<aric.pzqi@ingenic.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz-asoc-dma");
