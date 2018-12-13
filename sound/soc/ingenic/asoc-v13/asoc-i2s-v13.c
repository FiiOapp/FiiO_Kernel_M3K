/*
 *  sound/soc/ingenic/asoc-i2s.c
 *  ALSA Soc Audio Layer -- ingenic i2s (part of aic controller) driver
 *
 *  Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *	cscheng <shicheng.cheng@ingenic.com>
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
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/slab.h>
#include "asoc-aic-v13.h"
#include <../../arch/mips/xburst/soc-x1000/common/clk/clk.h>
#include <../../kernel/sound/soc/ingenic/ocodec/pcm5242_dlv.h>
#include <sound/ak4376.h>
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>
static int jz_i2s_debug = 0;
module_param(jz_i2s_debug, int, 0644);
#define I2S_DEBUG_MSG(msg...)			\
	do {					\
		if (jz_i2s_debug)		\
			printk("I2S: " msg);	\
	} while(0)

struct jz_i2s {
	struct device *aic;
#define I2S_WRITE 0x1
#define I2S_READ  0x2
#define I2S_INCODEC (0x1 <<4)
#define I2S_EXCODEC (0x2 <<4)
#define I2S_SLAVE (0x1 << 8)
#define I2S_MASTER (0x2 << 8)
	int i2s_mode;
	struct jz_pcm_dma_params tx_dma_data;
	struct jz_pcm_dma_params rx_dma_data;
	struct clk	*i2s_enable;
};

unsigned char volumeL;
unsigned char volumeR;
int work_sign=0;
static int sample_sign;
#define I2S_RFIFO_DEPTH 32
#define I2S_TFIFO_DEPTH 64
#define JZ_I2S_FORMATS (SNDRV_PCM_FMTBIT_S8 |  SNDRV_PCM_FMTBIT_S16_LE |	\
		SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S20_3LE |	\
		SNDRV_PCM_FMTBIT_S24_LE )
#define JZ_I2S_RATE (SNDRV_PCM_RATE_8000_384000) //&(~SNDRV_PCM_RATE_64000))

static void dump_registers(struct device *aic)
{
	struct jz_aic *jz_aic = dev_get_drvdata(aic);

	pr_info("AIC_FR\t\t%p : 0x%08x\n", (jz_aic->vaddr_base+AICFR),jz_aic_read_reg(aic, AICFR));
	pr_info("AIC_CR\t\t%p : 0x%08x\n", (jz_aic->vaddr_base+AICCR),jz_aic_read_reg(aic, AICCR));
	pr_info("AIC_I2SCR\t%p : 0x%08x\n",(jz_aic->vaddr_base+I2SCR),jz_aic_read_reg(aic, I2SCR));
	pr_info("AIC_SR\t\t%p : 0x%08x\n", (jz_aic->vaddr_base+AICSR),jz_aic_read_reg(aic, AICSR));
	pr_info("AIC_I2SSR\t%p : 0x%08x\n",(jz_aic->vaddr_base+I2SSR),jz_aic_read_reg(aic, I2SSR));
	pr_info("AIC_I2SDIV\t%p : 0x%08x\n",(jz_aic->vaddr_base+I2SDIV),jz_aic_read_reg(aic, I2SDIV));
	pr_info("AIC_DR\t%p : 0x%08x\n",(jz_aic->vaddr_base+AICDR),jz_aic_read_reg(aic, AICDR));
	pr_info("AIC_I2SCDR\t 0x%08x\n",*(volatile unsigned int*)0xb0000060);
	pr_info("AICSR\t 0x%08x\n",*(volatile unsigned int*)0xb0020014);
	
	pr_info("AIC_I2SCDR_PRE\t%p : 0x%08x\n",(jz_aic->vaddr_base+I2SCDR),jz_aic_read_reg(aic, I2SCDR));
        pr_info("AIC_I2SCDR1_PRE\t%p : 0x%08x\n",(jz_aic->vaddr_base+I2SCDR1),jz_aic_read_reg(aic, I2SCDR1));
        pr_info("AIC_I2SDIV\t%p : 0x%08x\n",(jz_aic->vaddr_base+I2SDIV),jz_aic_read_reg(aic, I2SDIV));
        pr_info("AIC_I2SCDR\t 0x%08x\n",*(volatile unsigned int*)0xb0000070);
	return;
}

static int jz_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = jz_i2s->aic;

	I2S_DEBUG_MSG("enter %s dai fmt %x\n", __func__, fmt);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	default:
	case SND_SOC_DAIFMT_I2S:		/*i2s format*/
		__i2s_select_i2s_fmt(aic);
		break;
	case SND_SOC_DAIFMT_MSB:
		__i2s_select_msb_fmt(aic);	/*msb format*/
		break;
	}
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	default:
	case SND_SOC_DAIFMT_CBM_CFM:	/*sync and bit clk (codec master : i2s slave)*/
		__i2s_bclk_input(aic);
		__i2s_sync_input(aic);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		__i2s_stop_bitclk(aic);
		__i2s_bclk_output(aic);
		__i2s_sync_output(aic);
		__i2s_start_bitclk(aic);
		break;
	}

	return 0;
}

static int jz_set_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = jz_i2s->aic;

	I2S_DEBUG_MSG("enter %s clk_id %d req %d clk dir %d\n", __func__,
			clk_id, freq, dir);

	if (clk_id == JZ_I2S_INNER_CODEC) {
		__aic_select_internal_codec(aic);
		aic_set_rate(aic, freq);
	} else
		__aic_select_external_codec(aic);

	//aic_set_rate(aic, freq);

	if (dir  == SND_SOC_CLOCK_OUT)
		{	
		//__i2s_stop_bitclk(aic);
		//__i2s_bclk_output(aic);
		//__i2s_sync_output(aic);
		//__i2s_start_bitclk(aic);
		__i2s_select_sysclk_output(aic);
		}
	else
		__i2s_select_sysclk_input(aic);
	return 0;
}

static int jz_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = jz_i2s->aic;

	I2S_DEBUG_MSG("enter %s div_id %d div %d\n", __func__, div_id , div);

	/*BIT CLK fix 64FS*/
	/*SYS_CLK is 256, 384, 512, 768*/
	if (div != 256 && div != 384 &&
			div != 512 && div != 768)
		return -EINVAL;

	__i2s_set_dv(aic, (div/64));
	__i2s_set_idv(aic, (div/64));
	return 0;
}


static int jz_i2s_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = jz_i2s->aic;
	enum aic_mode work_mode = AIC_I2S_MODE;
			printk("%s\n",__func__);
	//gpio_direction_output(PO_EN, 1);
	//mdelay(1);
        //ak4376_reg_init();
	//AK4376_WriteOneByte(0x0B,128);
        //AK4376_WriteOneByte(0x0C,0);
	/*printk("try to power on ak4376\n");
         gpio_direction_output(PO_EN, 1);
         mdelay(1);
         ak4376_cache_reg_init();
	*/
	//gpio_direction_output(AMP_MUTE, 1);//AMP open
	 //gpio_direction_output(AMP_EN, 1);
	I2S_DEBUG_MSG("enter %s, substream = %s\n",
			__func__,
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	work_mode = aic_set_work_mode(aic, work_mode, true);
	if (work_mode != AIC_I2S_MODE) {
		dev_warn(jz_i2s->aic, "Aic now is working on %s mode, open i2s mode failed\n",
				aic_work_mode_str(work_mode));
		return -EPERM;
	}

	if (!jz_i2s->i2s_mode) {
		__aic_select_i2s(aic);
		__i2s_play_lastsample(aic);
		__i2s_set_transmit_trigger(aic, I2S_TFIFO_DEPTH/4);
		__i2s_set_receive_trigger(aic, (I2S_RFIFO_DEPTH/4 - 1));
		__aic_enable(aic);
	}

	if (substream->stream ==
			SNDRV_PCM_STREAM_PLAYBACK) {
		//__i2s_send_rfirst(aic);
		__i2s_send_lfirst(aic);
		__i2s_disable_transmit_dma(aic);
		__i2s_disable_replay(aic);
		__aic_clear_tur(aic);
		jz_i2s->i2s_mode |= I2S_WRITE;
	} else {
		__i2s_disable_receive_dma(aic);
		__i2s_disable_record(aic);
		__aic_clear_ror(aic);
		jz_i2s->i2s_mode |= I2S_READ;
	}
	printk("start set AIC register....\n");
	//AK4376_WriteOneByte(0x0B,volumeL);
        //AK4376_WriteOneByte(0x0C,volumeR);
	return 0;
}
/*
static int jz_i2s_set_rate(struct device *aic ,struct jz_aic* jz_aic, unsigned long sample_rate){
	struct clk* cgu_aic_clk = jz_aic->clk;
	__i2s_stop_bitclk(aic);
	//__i2s_disable_sysclk_output(aic);
    clk_set_rate(cgu_aic_clk, sample_rate);
	//writel(0xa,I2S_CPM_VALID);
	__i2s_start_bitclk(aic);
	//__i2s_enable_sysclk_output(aic);
	//spdif_select_ori_sample_freq(aic,sample_rate);
	//spdif_select_sample_freq(aic,sample_rate);

	return sample_rate;
}
*/
static void set_ak4376_high_performance() {
	//02H LPMODE 0:high performance default 0 1:Low Power Mode
	if(!ak4376_power_mode) 
		AK4376_WriteOneByte(0x02,0x01);
	else
		AK4376_WriteOneByte(0x02,0x11);
	//24H 64fs
	AK4376_WriteOneByte(0x24,0x00);
}
static void set_ak4376_high_256k_sample() {
	//02H LPMODE 0:high performance default 0 1:Low Power Mode
	if(!ak4376_power_mode) 
		AK4376_WriteOneByte(0x02,0x01);
	else
		AK4376_WriteOneByte(0x02,0x11);
	//24H 64fs
	AK4376_WriteOneByte(0x24,0x00);
}
static void set_ak4376_low_256k_sample() {
	//02H LPMODE
	AK4376_WriteOneByte(0x02,0x11);
	//24H >128fs
	AK4376_WriteOneByte(0x24,0x40);
}
static void ak4376_sample_set(int sample)
{
	AK4376_WriteOneByte(0x03,0xFC);
	switch(sample){
	case 1://8000
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x20);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0xEF);
        AK4376_WriteOneByte(0x14,0x1D);
	break;
	case 2://11025
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x21);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x9F);
        AK4376_WriteOneByte(0x14,0x13);
	break;
	case 3://12000
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x0E);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x9F);
        AK4376_WriteOneByte(0x14,0x13);
	break;
	case 4://16000
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x24);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x77);
        AK4376_WriteOneByte(0x14,0x0E);
	break;
	case 5://22050
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x25);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x4F);
        AK4376_WriteOneByte(0x14,0x09);
	break;
	case 6://24000
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x26);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x4F);
        AK4376_WriteOneByte(0x14,0x09);	
	break;
	case 7://32000
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x08);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x3B);
        AK4376_WriteOneByte(0x14,0x0E);
	break;
	case 8://44100
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x09);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x27);
        AK4376_WriteOneByte(0x14,0x09);
	break;
	case 9://48000
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x0A);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x27);
        AK4376_WriteOneByte(0x14,0x09);
	break;
	case 10://64000
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x0C);
        AK4376_WriteOneByte(0x10,0x01);
        AK4376_WriteOneByte(0x12,0x27);
        AK4376_WriteOneByte(0x14,0x04);
	break;
	case 11://88200
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x0D);
        AK4376_WriteOneByte(0x10,0x01);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 12://96000
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x0E);
        AK4376_WriteOneByte(0x10,0x01);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 13://176400
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x71);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 14: //192K
		set_ak4376_high_performance();
		AK4376_WriteOneByte(0x05,0x72);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 15:
		//high performance mode 64fs
		//256K
		set_ak4376_high_256k_sample();
		//05H D6 D5 must set 0 1
		AK4376_WriteOneByte(0x05,0x34);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 16:
		//high performance mode 64fs
		//352800
		set_ak4376_high_256k_sample();
		//05H D6 D5 must set 0 1
		AK4376_WriteOneByte(0x05,0x35);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 17: //384K
		//high performance mode 64fs
		set_ak4376_high_256k_sample();
		//05H D6 D5 must set 0 1
		AK4376_WriteOneByte(0x05,0x36);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	default:
	break;
	}
	AK4376_WriteOneByte(0x03,0x03);
}

static int jz_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int channels = params_channels(params);
	int fmt_width = snd_pcm_format_width(params_format(params));
	struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = jz_i2s->aic;
	enum dma_slave_buswidth buswidth;
	int trigger;
	//,volumL,volumR;
	unsigned long sample_rate = params_rate(params);
	struct clk *clk_srcs = get_clk_from_id(0);
//	clk_srcs[CLK_ID_MPLL].rate = 600000000;
    //printk("%s:sample_rate=%lu,channels=%d fmt_width=%d\n",__func__,sample_rate,channels, fmt_width);
	I2S_DEBUG_MSG("enter %s, substream = %s\n", __func__,
		      (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");
	if (!((1 << params_format(params)) & JZ_I2S_FORMATS) ||
			channels > 2) {
		dev_err(dai->dev, "hw params not inval channel %d params %x\n",
				channels, params_format(params));
		return -EINVAL;
	}
	//gpio_direction_output(PO_EN, 0);
	/*
	 printk("try to power on ak4376\n");
         gpio_direction_output(PO_EN, 1);
         mdelay(1);
         ak4376_cache_reg_init();
	*/
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		//clk_srcs[CLK_ID_MPLL].rate = 600000000;
		/* channel */
		__i2s_channel(aic, channels);

		/* format */
		if (fmt_width == 8)
			buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;
		else if (fmt_width == 16)
			buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
		else
			buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;

		jz_i2s->tx_dma_data.buswidth = buswidth;
		jz_i2s->tx_dma_data.max_burst = (I2S_TFIFO_DEPTH * buswidth)/2;
		trigger = I2S_TFIFO_DEPTH - (jz_i2s->tx_dma_data.max_burst/(int)buswidth);
		__i2s_set_oss(aic, fmt_width);
		__i2s_set_transmit_trigger(aic, (trigger/2));
		snd_soc_dai_set_dma_data(dai, substream, (void *)&jz_i2s->tx_dma_data);
	
	
	//__i2s_set_sample_rate(aic,200000, sample_rate);
	switch(sample_rate){
	/*case 6000:
		audio_write((7<<29)|(4<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (1));
	break;*/
	case 8000: // sys_clock = 3072000 =384fs
	sample_sign=1;
	audio_write((7<<29)|(16<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (5));
	break;
	case 11025://147/15625x2x2 ~ 147/31250x2~147/31248*2=7/2976 128fs
	sample_sign=2;
	
	
	//clk_srcs[CLK_ID_MPLL].rate = 599961600;
	audio_write((3<<29)|(7<<13)|(2500<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (3));
	break;
	case 12000:
	sample_sign=3;
	
	
	audio_write((7<<29)|(8<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (1));
	break;
	case 16000: // sys_clock = 6144000 =384fs
	sample_sign=4;
	
	audio_write((7<<29)|(32<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (5));
	break;
	case 22050://147/15625x2x2 ~ 147/31250x2~147/31248*2=7/2976 64fs
	sample_sign=5;
	
	audio_write((3<<29)|(7<<13)|(2500<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (1));
	break;
	case 24000:  // sys_clock = 307200 = 128fs
	sample_sign=6;
	
	audio_write((7<<29)|(16<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (1));
	break;
	case 32000: // sys_clock = 11288000 =196fs
	sample_sign=7;
	
	audio_write((7<<29)|(64<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (5));
	break;
	case 44100://147/15625x2 ~ 147/31250~147/31248=7/1488 64fs
	sample_sign=8;
	
	//clk_srcs[CLK_ID_MPLL].rate = 599961600;
	//clk_srcs[CLK_ID_MPLL].rate = 564480000;
	//audio_write(0,I2SCDR_PRE);
	//audio_write((7<<29)|(7<<13)|(1488<<0),I2SCDR_PRE);
	audio_write((3<<29)|(7<<13)|(1250<<0),I2SCDR_PRE);
	//audio_write((5<<29)|(294<<12)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (1));
	break;
	case 48000: // sys_clock = 6144000 =128fs
	sample_sign=9;
	
	audio_write((7<<29)|(32<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (1));//
	break;
	case 64000:  // sys_clock =24576000 =196fs   128/4   3125-1 /4
	sample_sign=10;
	
	audio_write((7<<29)|(128<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (5));//384fs
	/*clk_srcs[CLK_ID_MPLL].rate = 400000000;
        audio_write((7<<29)|(32<<13)|(3125<<0),I2SCDR_PRE);
        audio_write(0,I2SCDR1_PRE);
        __i2s_set_dv(aic, (0));*/
	break;
	case 88200: //147/15625 ~ 147/15624=21/2232 64fs
	sample_sign=11;
	
	//audio_write((7<<29)|(21<<13)|(2232<<0),I2SCDR_PRE);
	//clk_srcs[CLK_ID_MPLL].rate = 599961600;
	//clk_srcs[CLK_ID_MPLL].rate = 564480000;
	//audio_write(0,I2SCDR_PRE);
	audio_write((3<<29)|(14<<13)|(1250<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (1));
	break;
	case 96000:   // sys_clock = 6144000 =64fs
	sample_sign=12;
	
	audio_write((7<<29)|(32<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (0));
	break;
	/*case 128000:  // sys_clock =24576000 =196fs   128/4   3125-1 /4
        AK4376_WriteOneByte(0x05,0x70);
        AK4376_WriteOneByte(0x10,0x03);
        AK4376_WriteOneByte(0x12,0x27);
        AK4376_WriteOneByte(0x14,0x04);
	clk_srcs[CLK_ID_MPLL].rate = 400000000;
        audio_write((7<<29)|(64<<13)|(3125<<0),I2SCDR_PRE);
        audio_write(0,I2SCDR1_PRE);
        __i2s_set_dv(aic, (0));
	break;*/
	case 176400: //294/15625 ~ 294/15624=42/2232 64fs
	sample_sign=13;
	//	clk_srcs[CLK_ID_MPLL].rate = 599961600;
	//audio_write(0,I2SCDR_PRE);
	audio_write((3<<29)|(28<<13)|(1250<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (1));
	break;
	case 192000:  // sys_clock = 12288000 = 64fs
	//600000000*64/3125/64=12288000/64=192000
	sample_sign=14;
	audio_write((7<<29)|(64<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (0));//64fs
	break;
	case 256000:
	//64fs for DAC
	//64000*4=256000
	sample_sign=15;
	audio_write((3<<29)|(128<<13)|(7875<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (0));//64fs
	break;
	case 352800:
	//64fs for DAC
	//44100*8=352800
	sample_sign=16;
	audio_write((3<<29)|(28<<13)|(1250<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (0));
	break;
	case 384000://sys_clock = 24576000 = 64fs
	//64fs for DAC
	sample_sign=17;
	audio_write((7<<29)|(128<<13)|(3125<<0),I2SCDR_PRE);
	audio_write(0,I2SCDR1_PRE);
	__i2s_set_dv(aic, (0));//64fs
	break;
	default:
	break;
	}
	ak4376_sample_set(sample_sign);
    }
 	else {
		clk_srcs[CLK_ID_MPLL].rate = 600000000;
		__i2s_channel(aic, channels);
		
		audio_write((1<<29)|(31<<13)|(8189<<0),I2SCDR_PRE);
        	audio_write(1,I2SCDR1_PRE);
        	__i2s_set_dv(aic, (0));
                /* format */
                if (fmt_width == 8)
                        buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;
                else if (fmt_width == 16)
                        buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
                else
                        buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;

                jz_i2s->rx_dma_data.buswidth = buswidth;
                jz_i2s->rx_dma_data.max_burst = (I2S_RFIFO_DEPTH * buswidth)/2;
                trigger = jz_i2s->tx_dma_data.max_burst/(int)buswidth;
                __i2s_set_iss(aic, fmt_width);
                __i2s_set_receive_trigger(aic, (trigger/2 - 1));
                snd_soc_dai_set_dma_data(dai, substream, (void *)&jz_i2s->rx_dma_data);
        }
	/*printk("try to power on ak4376\n");
         gpio_direction_output(PO_EN, 1);
         mdelay(1);
         ak4376_cache_reg_init();
	*/
/*
	// sample rate
	unsigned long tmp_rate = 0;
    __i2s_stop_bitclk(aic);
    __i2s_disable_sysclk_output(aic);
	tmp_rate = aic_set_rate(aic,sample_rate);
    __i2s_start_bitclk(aic);
    __i2s_enable_sysclk_output(aic);
	if(tmp_rate < 0)
		printk("set i2s clk failed!!\n");
    */
	return 0;
}

static void jz_i2s_start_substream(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = jz_i2s->aic;
	int timeout = 150000;
	//printk("%s\n",__func__);
	//if(!work_sign){
	 //gpio_direction_output(PO_EN, 1);	  
	work_sign=1;

	//}
	//gpio_direction_output(AMP_MUTE, 1);//AMP open
	//gpio_direction_output(AMP_EN, 1);
	I2S_DEBUG_MSG("enter %s, substream = %s\n",
			__func__,
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		int i = 4;
		dev_dbg(dai->dev, "codec fifo level0 %x\n", jz_aic_read_reg(aic, AICSR));
		for (i= 0; i < 16 ; i++) {
			__aic_write_txfifo(aic, 0x0);
			__aic_write_txfifo(aic, 0x0);
		}
		__aic_clear_tur(aic);
		dev_dbg(dai->dev, "codec fifo level1 %x\n", jz_aic_read_reg(aic, AICSR));
		__i2s_enable_replay(aic);
		while((!__aic_test_tur(aic)) && (timeout--)){
			if(timeout == 0){
				printk("wait tansmit fifo under run error\n");
				return;
			}
		}
		__i2s_enable_transmit_dma(aic);
		__aic_clear_tur(aic);
		if (jz_i2s_debug) __aic_en_tur_int(aic);
	} else {
		__aic_flush_rxfifo(aic);
		mdelay(1);
		__i2s_enable_record(aic);
                __i2s_enable_receive_dma(aic);
		//clk_enable(jz_i2s->i2s_enable);
		if (jz_i2s_debug) __aic_en_ror_int(aic);
	}
	//ak4376_reg_init();
        AK4376_WriteOneByte(0x0B,volumeL);
        AK4376_WriteOneByte(0x0C,volumeR);
	//ak4376_sample_set(sample_sign);
	//ak4376_reg_init();
	 //printk("==> stop MUTE\n");
	// AK4376_WriteOneByte(0x0B,volumeL);
	 //AK4376_WriteOneByte(0x0C,volumeR);
	//AK4376_WriteOneByte(0x0B,volumeL);
        //AK4376_WriteOneByte(0x0C,volumeR);

	//register_write(0, 0x03,	0);
	return;
}

static void jz_i2s_stop_substream(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = jz_i2s->aic;
	int timeout = 150000;
	I2S_DEBUG_MSG("enter %s, substream = %s\n",
			__func__,
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (jz_i2s_debug) __aic_dis_tur_int(aic);
		if (__i2s_transmit_dma_is_enable(aic)) {
			__i2s_disable_transmit_dma(aic);
			__aic_clear_tur(aic);
			/*hrtime mode: stop will be happen in any where, make sure there is
			 *	no data transfer on ahb bus before stop dma
			 */
			while((!__aic_test_tur(aic)) && (timeout--)){
				if(timeout == 0){
					printk("wait tansmit fifo under run error\n");
					return;
				}
			}
		}
		__i2s_disable_replay(aic);
		__aic_clear_tur(aic);
	} else {
		if (jz_i2s_debug) __aic_dis_ror_int(aic);
		if (__i2s_receive_dma_is_enable(aic)) {
			__i2s_disable_receive_dma(aic);
			__aic_clear_ror(aic);
			while(!__aic_test_ror(aic) && timeout--){
				if(timeout == 0){
					printk("wait tansmit fifo under run error\n");
					return;
				}
			}
		}
		//clk_disable(jz_i2s->i2s_enable);
		__i2s_disable_record(aic);
		__aic_clear_ror(aic);
	}
	//printk("==> start MUTE");
	//gpio_direction_output(PO_EN, 0);
	work_sign=0;
	AK4376_WriteOneByte(0x0B,128);
	AK4376_WriteOneByte(0x0C,0);
	//volumeL=ak4376_i2c_read(ak4376,0x0B);
	//volumeR=ak4376_i2c_read(ak4376,0x0C);
	//AK4376_WriteOneByte(0x0B,128);
	//AK4376_WriteOneByte(0x0C,0);
	//register_write(0, 0x03,	(1 <<  4)|(1 <<  0));
	//gpio_direction_output(AMP_MUTE, 0);//AMP open
	 //gpio_direction_output(AMP_EN, 0);
	return;
}

static int jz_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	/* struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev); */
	struct jz_pcm_runtime_data *prtd = substream->runtime->private_data;
	I2S_DEBUG_MSG("enter %s, substream = %s cmd = %d\n",
		      __func__,
		      (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture",
		      cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
#ifndef CONFIG_JZ_ASOC_DMA_HRTIMER_MODE
		if (atomic_read(&prtd->stopped_pending))
			return -EPIPE;
#endif
	//AK4376_WriteOneByte(0x0B,volumeL);
        //AK4376_WriteOneByte(0x0C,volumeR);	
	jz_i2s_start_substream(substream, dai);
	//printk("ak4376 mute\n");
	//AK4376_WriteOneByte(0x0B,volumeL);
        //AK4376_WriteOneByte(0x0C,volumeR);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
#ifndef CONFIG_JZ_ASOC_DMA_HRTIMER_MODE
		if (atomic_read(&prtd->stopped_pending))
			return 0;
#endif
		jz_i2s_stop_substream(substream, dai);
		break;
	}
	/*dump_registers(aic);*/
	return 0;
}

static void jz_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai) {
	struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = jz_i2s->aic;
	enum aic_mode work_mode = AIC_I2S_MODE;
			printk("%s\n",__func__);
	//printk("==> start MUTE");
       //gpio_direction_output(PO_EN, 0);
	//gpio_direction_output(AMP_MUTE, 0);//AMP close
	 //gpio_direction_output(AMP_EN, 0);
	I2S_DEBUG_MSG("enter %s, substream = %s\n",
			__func__,
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");
	work_mode = aic_set_work_mode(jz_i2s->aic, work_mode, false);
	BUG_ON((work_mode != AIC_NO_MODE));

	jz_i2s_stop_substream(substream, dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		jz_i2s->i2s_mode &= ~I2S_WRITE;
	else
		jz_i2s->i2s_mode &= ~I2S_READ;

	if (!jz_i2s->i2s_mode)
		__aic_disable(aic);
	return;
}

static int jz_i2s_probe(struct snd_soc_dai *dai)
{
	struct jz_i2s *jz_i2s = dev_get_drvdata(dai->dev);
	struct device *aic = jz_i2s->aic;
	/* struct jz_aic *jz_aic = dev_get_drvdata(aic); */
	I2S_DEBUG_MSG("enter %s\n", __func__);
	//__i2s_bclk_output(aic);
	__i2s_select_sysclk_output(aic);
	__aic_select_internal_codec(aic);
	//__aic_select_external_codec(aic);
	__aic_select_i2s(aic);
	__aic_enable(aic);
	return 0;
}


static struct snd_soc_dai_ops jz_i2s_dai_ops = {
	.startup	= jz_i2s_startup,
	.trigger 	= jz_i2s_trigger,
	.hw_params 	= jz_i2s_hw_params,
	.shutdown	= jz_i2s_shutdown,
	.set_fmt	= jz_set_dai_fmt,
	.set_sysclk	= jz_set_sysclk,
	.set_clkdiv	= jz_set_clkdiv,
};

#define jz_i2s_suspend	NULL
#define jz_i2s_resume	NULL
static struct snd_soc_dai_driver jz_i2s_dai = {
		.probe   = jz_i2s_probe,
		.suspend = jz_i2s_suspend,
		.resume  = jz_i2s_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = JZ_I2S_RATE,
			.formats = JZ_I2S_FORMATS,
		},
		.capture = {
			.channels_min = 2,
			.channels_max = 2,
			.rates = JZ_I2S_RATE,
			.formats = JZ_I2S_FORMATS,
		},
		.ops = &jz_i2s_dai_ops,
};

static ssize_t jz_i2s_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct jz_i2s *jz_i2s = dev_get_drvdata(dev);
	dump_registers(jz_i2s->aic);
	return 0;
}

static struct device_attribute jz_i2s_sysfs_attrs[] = {
	__ATTR(i2s_regs, S_IRUGO, jz_i2s_regs_show, NULL),
};

static const struct snd_soc_component_driver jz_i2s_component = {
	.name		= "jz-i2s",
};

static int jz_i2s_platfrom_probe(struct platform_device *pdev)
{
	struct jz_aic_subdev_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct jz_i2s *jz_i2s;
	int i = 0, ret;
	/*struct device *aic = pdev->dev.parent;*/
	jz_i2s = devm_kzalloc(&pdev->dev, sizeof(struct jz_i2s), GFP_KERNEL);
	if (!jz_i2s)
		return -ENOMEM;

	jz_i2s->aic = pdev->dev.parent;
	jz_i2s->i2s_mode = 0;
	jz_i2s->tx_dma_data.dma_addr = pdata->dma_base + AICDR;
	jz_i2s->rx_dma_data.dma_addr = pdata->dma_base + AICDR;
	platform_set_drvdata(pdev, (void *)jz_i2s);

	for (; i < ARRAY_SIZE(jz_i2s_sysfs_attrs); i++) {
		ret = device_create_file(&pdev->dev, &jz_i2s_sysfs_attrs[i]);
		if (ret)
			dev_warn(&pdev->dev,"attribute %s create failed %x",
					attr_name(jz_i2s_sysfs_attrs[i]), ret);
	}

	jz_i2s->i2s_enable = clk_get(&pdev->dev, "i2s_enable");
	if (IS_ERR_OR_NULL(jz_i2s->i2s_enable)) {
		ret = PTR_ERR(jz_i2s->i2s_enable);
		jz_i2s->i2s_enable = NULL;
		dev_err(&pdev->dev, "Failed to get clock: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_component(&pdev->dev, &jz_i2s_component,
					 &jz_i2s_dai, 1);

	if (!ret)
		dev_info(&pdev->dev, "i2s platform probe success\n");
	return ret;
}

static int jz_i2s_platfom_remove(struct platform_device *pdev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(jz_i2s_sysfs_attrs); i++)
		device_remove_file(&pdev->dev, &jz_i2s_sysfs_attrs[i]);
	platform_set_drvdata(pdev, NULL);
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}


static struct platform_driver jz_i2s_plat_driver = {
	.probe  = jz_i2s_platfrom_probe,
	.remove = jz_i2s_platfom_remove,
	.driver = {
		.name = "jz-asoc-aic-i2s",
		.owner = THIS_MODULE,
	},
};

static int jz_i2s_init(void)
{
        return platform_driver_register(&jz_i2s_plat_driver);
}
module_init(jz_i2s_init);

static void jz_i2s_exit(void)
{
	platform_driver_unregister(&jz_i2s_plat_driver);
}
module_exit(jz_i2s_exit);

MODULE_AUTHOR("shicheng.cheng@ingenic.com");
MODULE_DESCRIPTION("JZ AIC I2S SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz-aic-i2s");
