/*
 *  sound/soc/ingenic/asoc-dmic.c
 *  ALSA Soc Audio Layer -- ingenic dmic driver , work with wakeup module.
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
#include <linux/delay.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <linux/io.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
 #include <linux/circ_buf.h>


#include "asoc-dmic-v13.h"
#include "asoc-aic-v13.h"


#include <linux/voice_wakeup_module.h>

static int jz_dmic_debug = 0;
module_param(jz_dmic_debug, int, 0644);
#define DMIC_DEBUG_MSG(msg...)			\
	do {					\
		if (jz_dmic_debug)		\
			printk(KERN_DEBUG"dmic: " msg);	\
	} while(0)

#define DMIC_FIFO_DEPTH 64
#define JZ_DMIC_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)
#define JZ_DMIC_RATE (SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_48000)

static void dump_registers(struct device *dev)
{

	printk("not implemented!\n");
	return;
}

static int jz_dmic_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct jz_dmic *jz_dmic = dev_get_drvdata(dai->dev);
	struct dma_fifo *tfifo = jz_dmic->tcsm_fifo;
	struct dma_fifo *rfifo = jz_dmic->record_fifo;

	DMIC_DEBUG_MSG("enter %s, substream capture\n",__func__);

	tfifo->n_size = TCSM_DATA_BUFFER_SIZE;
	tfifo->xfer.buf = (char *)TCSM_DATA_BUFFER_ADDR;
	tfifo->xfer.head = (char *)KSEG1ADDR(wakeup_module_get_dma_address()) - tfifo->xfer.buf;
	tfifo->xfer.tail = tfifo->xfer.head;

	rfifo->n_size = substream->runtime->dma_bytes;
	rfifo->xfer.buf = substream->runtime->dma_area;
	rfifo->paddr = substream->runtime->dma_addr;
	rfifo->xfer.head = 0;
	rfifo->xfer.tail = 0;


	regulator_enable(jz_dmic->vcc_dmic);
	jz_dmic->en = 1;

	wakeup_module_open(NORMAL_RECORD);
	printk("start dmic wakeup module\n");
	return 0;
}

static int dmic_set_rate(int rate)
{

	wakeup_module_ioctl(DMIC_IOCTL_SET_SAMPLERATE, rate);
	return 0;
}

static int dmic_set_channel(int ch)
{
	wakeup_module_ioctl(DMIC_IOCTL_SET_CHANNEL, ch);
	return 0;
}

static int jz_dmic_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int channels = params_channels(params);
	int rate = params_rate(params);
	int fmt_width = snd_pcm_format_width(params_format(params));

	DMIC_DEBUG_MSG("enter %s, substream = %s\n",__func__,"capture");

	if (!((1 << params_format(params)) & JZ_DMIC_FORMATS)
			||channels < 1||channels > 4|| rate > 48000||rate < 8000
			||fmt_width != 16) {
		dev_err(dai->dev, "hw params not inval channel %d params %x rate %d fmt_width %d\n",
				channels, params_format(params),rate,fmt_width);
		return -EINVAL;
	}


	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dmic_set_rate(rate);
		dmic_set_channel(channels);
	} else {
		dev_err(dai->dev, "DMIC is a capture device\n");
	}
	return 0;
}

static void jz_dmic_start_substream(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct jz_dmic *jz_dmic = dev_get_drvdata(dai->dev);
	struct dma_fifo *rfifo = jz_dmic->record_fifo;

	DMIC_DEBUG_MSG("enter %s, substream start capture\n", __func__);

	if(rfifo->xfer.buf != (char *)substream->runtime->dma_area) {
		rfifo->n_size = substream->runtime->dma_bytes;
		rfifo->xfer.buf = substream->runtime->dma_area;
		rfifo->xfer.head = 0;
		rfifo->xfer.tail = 0;
		rfifo->paddr = substream->runtime->dma_addr;
	}
	/* start timer to transfer data from tcsm to ddr.*/

	mod_timer(&jz_dmic->record_timer, jiffies + msecs_to_jiffies(30));

	return;
}

static void jz_dmic_stop_substream(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct jz_dmic *jz_dmic = dev_get_drvdata(dai->dev);
	DMIC_DEBUG_MSG("enter %s, substream stop capture\n",__func__);


	/* stop transfer timer */
	del_timer_sync(&jz_dmic->record_timer);


	return;
}

static int jz_dmic_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	DMIC_DEBUG_MSG("enter %s, substream capture cmd = %d\n", __func__,cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		printk(KERN_DEBUG"dmic start\n");
		jz_dmic_start_substream(substream, dai);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		printk(KERN_DEBUG"dmic stop\n");
		jz_dmic_stop_substream(substream, dai);
		break;
	}
	return 0;
}

static void jz_dmic_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai) {
	struct jz_dmic *jz_dmic = dev_get_drvdata(dai->dev);

	DMIC_DEBUG_MSG("enter %s, substream = capture\n", __func__);

	jz_dmic_stop_substream(substream, dai);
	/* module close */

	wakeup_module_close(NORMAL_RECORD);
	regulator_disable(jz_dmic->vcc_dmic);
	jz_dmic->en = 0;

	return;
}

static int jz_dmic_probe(struct snd_soc_dai *dai)
{
	return 0;
}



dma_addr_t jz_dmic_module_get_trans_addr(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	struct jz_dmic *jz_dmic = dev_get_drvdata(dai->dev);
	struct dma_fifo *rfifo = jz_dmic->record_fifo;

	struct circ_buf * rxfer = &rfifo->xfer;

	return rfifo->paddr + rxfer->head;
}
EXPORT_SYMBOL(jz_dmic_module_get_trans_addr);

static void record_timer_handler(unsigned long data)
{
	struct jz_dmic *jz_dmic = (struct jz_dmic *)data;
	struct dma_fifo *rfifo = jz_dmic->record_fifo;
	struct dma_fifo *tfifo = jz_dmic->tcsm_fifo;

	struct circ_buf *rxfer = &rfifo->xfer;

	struct circ_buf *txfer = &tfifo->xfer;
	dma_addr_t trans_addr = wakeup_module_get_dma_address();

	int ntotal;

	int nread;
	int ncnt2end = 0;
	int ncopy = 0;
	//printk("trans_addr:%08x, rxfer->buf: 0x%08x", trans_addr, rxfer->buf);
	txfer->head = (char *)KSEG1ADDR(trans_addr) - txfer->buf;

	/* TODO: copy data from tcsm to ddr , simulate dma transfer. */

	/* get available data to be store */


	ntotal = CIRC_CNT(txfer->head, txfer->tail, tfifo->n_size);

	while(ntotal > 0) {

		nread = CIRC_CNT(txfer->head, txfer->tail, tfifo->n_size);

		if(nread > CIRC_CNT_TO_END(txfer->head, txfer->tail, tfifo->n_size)) {
			nread = CIRC_CNT_TO_END(txfer->head, txfer->tail, tfifo->n_size);
		}


		ncopy = nread;
		ncnt2end = rfifo->n_size - rxfer->head;
		while((ncopy > ncnt2end)) {

			memcpy(rxfer->buf + rxfer->head, txfer->buf + txfer->tail, ncnt2end);
			ncopy -= ncnt2end;

			rxfer->head += ncnt2end;
			rxfer->head %= rfifo->n_size;

			txfer->tail += ncnt2end;
			txfer->tail %= tfifo->n_size;

			/* new round */
			ncnt2end = rfifo->n_size - rxfer->head;
		}

		if(ncopy != 0) {
			memcpy(rxfer->buf + rxfer->head, txfer->buf + txfer->tail, ncopy);
			rxfer->head += ncopy;
			rxfer->head %= rfifo->n_size;

			txfer->tail += ncopy;
			txfer->tail %= tfifo->n_size;
		}

		ntotal -= nread;

	}

	//printk("record_timer:txfer->head:%08x, rxfer->head:0x%08x\n", txfer->head, rxfer->head);
	mod_timer(&jz_dmic->record_timer, jiffies + msecs_to_jiffies(20));

}

static struct snd_soc_dai_ops jz_dmic_dai_ops = {
	.startup	= jz_dmic_startup,
	.trigger 	= jz_dmic_trigger,
	.hw_params 	= jz_dmic_hw_params,
	.shutdown	= jz_dmic_shutdown,
};

#define jz_dmic_suspend	NULL
#define jz_dmic_resume	NULL

static ssize_t jz_dmic_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct jz_dmic *jz_dmic = dev_get_drvdata(dev);
	dump_registers(jz_dmic->dev);
	return 0;
}

static struct device_attribute jz_dmic_sysfs_attrs[] = {
	__ATTR(dmic_regs, S_IRUGO, jz_dmic_regs_show, NULL),
};

static struct snd_soc_dai_driver jz_dmic_dai = {
		.probe   = jz_dmic_probe,
		.suspend = jz_dmic_suspend,
		.resume  = jz_dmic_resume,
		.capture = {
			.channels_min = 1,
			.channels_max = 4,
			.rates = JZ_DMIC_RATE,
			.formats = JZ_DMIC_FORMATS,
		},
		.ops = &jz_dmic_dai_ops,
};

static const struct snd_soc_component_driver jz_dmic_component = {
	.name		= "jz-dmic-module",
};

static int jz_dmic_platfrom_probe(struct platform_device *pdev)
{
	struct jz_dmic *jz_dmic;
	int i = 0, ret;

	jz_dmic = devm_kzalloc(&pdev->dev, sizeof(struct jz_dmic), GFP_KERNEL);
	if (!jz_dmic)
		return -ENOMEM;

	jz_dmic->dev = &pdev->dev;

	jz_dmic->vcc_dmic = regulator_get(&pdev->dev,"vcc_dmic");

	platform_set_drvdata(pdev, (void *)jz_dmic);

	for (; i < ARRAY_SIZE(jz_dmic_sysfs_attrs); i++) {
		ret = device_create_file(&pdev->dev, &jz_dmic_sysfs_attrs[i]);
		if (ret)
			dev_warn(&pdev->dev,"attribute %s create failed %x",
					attr_name(jz_dmic_sysfs_attrs[i]), ret);
	}

	jz_dmic->record_fifo = kzalloc(sizeof(struct dma_fifo), GFP_KERNEL);
	if(!jz_dmic->record_fifo) {
		printk("failed to allocate dmic record fifo!\n");
		goto _err_alloc_rfifo;
	}

	jz_dmic->tcsm_fifo = kzalloc(sizeof(struct dma_fifo), GFP_KERNEL);
	if(!jz_dmic->tcsm_fifo) {
		printk("failed to allocate dmic tcsm fifo!\n");
		goto _err_alloc_tfifo;
	}


	init_timer(&jz_dmic->record_timer);
	jz_dmic->record_timer.function = record_timer_handler;
	jz_dmic->record_timer.data	= (unsigned long)jz_dmic;



	ret = snd_soc_register_component(&pdev->dev, &jz_dmic_component,
					 &jz_dmic_dai, 1);
	if (ret)
		goto err_register_cpu_dai;
	dev_dbg(&pdev->dev, "dmic platform probe success\n");
	return ret;

err_register_cpu_dai:
	platform_set_drvdata(pdev, NULL);
	kfree(jz_dmic->tcsm_fifo);
_err_alloc_tfifo:
	kfree(jz_dmic->record_fifo);
_err_alloc_rfifo:
	kfree(jz_dmic);

	return ret;
}

static int jz_dmic_platfom_remove(struct platform_device *pdev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(jz_dmic_sysfs_attrs); i++)
		device_remove_file(&pdev->dev, &jz_dmic_sysfs_attrs[i]);
	snd_soc_unregister_component(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}


#ifdef CONFIG_PM
static int jz_dmic_platfom_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct jz_dmic *jz_dmic = platform_get_drvdata(pdev);
	if(jz_dmic->en){
		regulator_disable(jz_dmic->vcc_dmic);
	}
	return 0;
}

static int jz_dmic_platfom_resume(struct platform_device *pdev)
{
	struct jz_dmic *jz_dmic = platform_get_drvdata(pdev);
	if(jz_dmic->en){
		regulator_enable(jz_dmic->vcc_dmic);
	}
	return 0;
}
#endif
static struct platform_driver jz_dmic_plat_driver = {
	.probe  = jz_dmic_platfrom_probe,
	.remove = jz_dmic_platfom_remove,
#ifdef CONFIG_PM
	.suspend = jz_dmic_platfom_suspend,
	.resume = jz_dmic_platfom_resume,
#endif
	.driver = {
		.name = "jz-asoc-dmic-module",
		.owner = THIS_MODULE,
	},
};

static int jz_dmic_init(void)
{
	struct platform_device *pdev = NULL;

	pdev = platform_device_register_simple("jz-asoc-dmic-module", -1, NULL, 0);
	if(IS_ERR(pdev)) {
		printk("platform device register jz-asoc-dmic-module failed!\n");
		return -ENOMEM;
	}

	printk("platform device register jz-asoc-dmic-module success!\n");


        return platform_driver_register(&jz_dmic_plat_driver);
}
module_init(jz_dmic_init);

static void jz_dmic_exit(void)
{
	platform_driver_unregister(&jz_dmic_plat_driver);
}
module_exit(jz_dmic_exit);

MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("JZ AIC dmic SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz-dmic");
