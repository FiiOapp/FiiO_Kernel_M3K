/*
 * sound/soc/ingenic/icodec/icdc_d3.c
 * ALSA SoC Audio driver -- ingenic internal codec (icdc_d3) driver

 * Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *	cscheng <shicheng.cheng@ingenic.com>
 *
 * Note: icdc_d3 is an internal codec for jz SOC
 *	 used for x1000 and so on
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>
#include <sound/soc-dai.h>
#include <soc/irq.h>
#include "icdc_d3.h"
#include <sound/ak4376.h>
#include <linux/skytc/eq/eq.h>
#include <../../kernel/sound/soc/ingenic/ocodec/pcm5242_dlv.h>
// Vocal tract balance
bool ak4376_power_mode =0;
unsigned char ak4376_volume[]={
        0,1,2,3,4,5,6,7,
        8,9,10,11,12,
        13,14,15,16,17,
        18,19,20,21,22,
        23,24,25,25,25,
        25,25,25,25,25,
        25,25,25,25,25,
        25,25,25,25,25,
        25,25,25,25,25,
        25,25,25,25,25,
        25,25,25,25,25,
        25,25,
};

static int icdc_d3_debug = 0;
module_param(icdc_d3_debug, int, 0644);
#define DEBUG_MSG(msg...)						\
	do {										\
		if (icdc_d3_debug)						\
			printk("ICDC: " msg);				\
	} while(0)

static u8 icdc_d3_reg_defcache[SCODA_MAX_REG_NUM] = {
/* reg 0x0 ... 0x9 */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xd3,0xd3,
/* reg 0xa ... 0x13 */
	0x00,0x30,0x30,0xb0,0xb1,0xb0,0x00,0x00,0x0f,0x40,
/* reg 0x14 ... 0x1d */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0xff,
/* reg 0x1e ... 0x27 */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
/* reg 0x28 ... 0x31 */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
/* reg 0x32 ... 0x39 */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
/* extern reg */
	0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x34,0x07,0x44,0x1f,0x00,
};

static int icdc_d3_volatile(struct snd_soc_codec *codec, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM)
		return 1;

	switch (reg) {
	case SCODA_REG_SR:
	case SCODA_REG_SR2:
	case SCODA_REG_SIGR:
	case SCODA_REG_SIGR3:
	case SCODA_REG_SIGR5:
	case SCODA_REG_SIGR7:
	case SCODA_REG_MR:
	case SCODA_REG_IFR:
	case SCODA_REG_IFR2:
	case SCODA_REG_SR_ADC_AGCDGL:
	case SCODA_REG_SR_ADC_AGCDGR:
	case SCODA_REG_SR_ADC_AGCAGL:
	case SCODA_REG_SR_ADC_AGCAGR:
	case SCODA_REG_SR_TR1:
	case SCODA_REG_SR_TR2:
	case SCODA_REG_SR_TR_SRCDAC:
		return 1;
	default:
		return 0;
	}
}

static int icdc_d3_writable(struct snd_soc_codec *codec, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM)
		return 0;

	switch (reg) {
	case SCODA_REG_SR:
	case SCODA_REG_SR2:
	case SCODA_REG_SIGR:
	case SCODA_REG_SIGR3:
	case SCODA_REG_SIGR5:
	case SCODA_REG_SIGR7:
	case SCODA_REG_MR:
	case SCODA_REG_SR_ADC_AGCDGL:
	case SCODA_REG_SR_ADC_AGCDGR:
	case SCODA_REG_SR_ADC_AGCAGL:
	case SCODA_REG_SR_ADC_AGCAGR:
	case SCODA_REG_SR_TR1:
	case SCODA_REG_SR_TR2:
	case SCODA_REG_SR_TR_SRCDAC:
		return 0;
	default:
		return 1;
	}
}

static int icdc_d3_readable(struct snd_soc_codec *codec, unsigned int reg)
{
	if (reg > SCODA_MAX_REG_NUM)
		return 0;
	else
		return 1;
}

static void dump_registers_hazard(struct icdc_d3 *icdc_d3)
{
	int reg = 0;
	dev_info(icdc_d3->dev, "-------------------register:");
	for ( ; reg < SCODA_MAX_REG_NUM; reg++) {
		if (reg % 8 == 0)
			printk("\n");
		if (icdc_d3_readable(icdc_d3->codec, reg))
			printk(" 0x%02x:0x%02x,", reg, icdc_d3_hw_read(icdc_d3, reg));
		else
			printk(" 0x%02x:0x%02x,", reg, 0x0);
	}
	printk("\n");
	printk("mix_0=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_0));
	printk("mix_1=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_1));
	printk("mix_2=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_2));
	printk("mix_3=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_3));
	printk("mix_4=%02x\n", icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_4));

	printk("\n");
	dev_info(icdc_d3->dev, "----------------------------\n");
	return;
}

static int icdc_d3_write(struct snd_soc_codec *codec, unsigned int reg,
						 unsigned int value)
{
	struct icdc_d3 *icdc_d3 = snd_soc_codec_get_drvdata(codec);
        int ret = 0;
        int val ;
        //BUG_ON(reg > SCODA_MAX_REG_NUM);
        //printk("%s reg = %x value = %x val=%x \n",__func__,reg,value,val);
        if(reg == AK4376_DACL || reg == AK4376_DACR || reg == AK4376_BALANCE)
        {
/*
        if(value <=60)
                val = 0x3c - value;
	else if(value <)
        else if(value <192)
                val = 0xbc - value;
        else    val = 0xfc - value;
*/
	if(value>0xbc)
		val = 0xfc-value;
	else if(value>0x7c)
		val = 0xbc-value;
	else if(value>0x3c)
		val = 0x7c-value;
	else val = 0x3c -value;
        //val = value - 0x26;
        //printk("%s reg = %x value = %x val=%x \n",__func__,reg,value,ak4376_volume[val]);
        ret = snd_soc_cache_write(codec, reg, value);
                        if (ret != 0)
                                dev_err(codec->dev, "Cache write to %x failed: %d\n",
                                                reg, ret);
        if(reg == AK4376_DACL)
        {
//                if((ak4376_volume[val]-balance_valueR)>1)
//                volumeL=128+ak4376_volume[val]-balance_valueR;
 //               else
                volumeL=128+ak4376_volume[val];
		if(work_sign)
 	       AK4376_WriteOneByte(0x0B,volumeL);
 //               ak4376_i2c_write(ak4376,0x0B,volumeL);
   //             printk("set L volum reg=0x0B, value=%d\n",ak4376_i2c_read(ak4376,0x0B));
        //printk("set L volum reg=%d, value=%d\n",reg,AK4376_ReadOneByte(reg));
        }
        else if(reg == AK4376_DACR)
        {
//                if((ak4376_volume[val]-balance_valueL)>1)
//                volumeR=ak4376_volume[val]-balance_valueL;
//                else
                volumeR=ak4376_volume[val];
   		if(work_sign)
	     AK4376_WriteOneByte(0x0C,volumeR);
        //        ak4376_i2c_write(ak4376,0x0C,volumeR);
          //      printk("set R volum reg=0x0C, value=%d\n",ak4376_i2c_read(ak4376,0x0C));

        //printk("set R volum reg=%d, value=%d\n",reg,AK4376_ReadOneByte(reg));

        }
     
	else if(reg == AK4376_BALANCE)
        {
		val = value % 2;
                if(val != 0)
			{
			if(work_sign)
			{
			printk("try to set ak4376 low mode\n");
			AK4376_WriteOneByte(0x02,0x11);
			AK4376_WriteOneByte(0x24,0x40);
			}
           		ak4376_power_mode=1;
			printk("start ak4376 low mode\n");
			}
		else if(val==0)
			{
			if(work_sign)
                        {
			printk("try to set ak4376 high mode\n");
                        AK4376_WriteOneByte(0x02,0x01);
                        AK4376_WriteOneByte(0x24,0x00);
                        }
			ak4376_power_mode=0;   
			printk("start ak4376 high mode\n");
			}  
        }
     } 
        else{
        val = value;
        if (icdc_d3_writable(codec, reg)) {
                if (!icdc_d3_volatile(codec,reg)) {
                        if((reg == SCODA_REG_GCR_DACL)||(reg == SCODA_REG_GCR_DACR)){
                                if(val < 32)
                                        val = 31 - val;
                                else
                                        val = 95 - val;
                        }
                        ret = snd_soc_cache_write(codec, reg, val);
                        if (ret != 0)
                                dev_err(codec->dev, "Cache write to %x failed: %d\n",
                                                reg, ret);
                }
                return icdc_d3_hw_write(icdc_d3, reg, val);
        }
    }
        return 0;
}

static unsigned int icdc_d3_read(struct snd_soc_codec *codec, unsigned int reg)
{

	struct icdc_d3 *icdc_d3 = snd_soc_codec_get_drvdata(codec);
        int val = 0, ret = 0;
        //BUG_ON(reg > SCODA_MAX_REG_NUM);

        if (1){//(!icdc_d3_volatile(codec,reg)) {
                ret = snd_soc_cache_read(codec, reg, &val);
                if (ret >= 0){
                /*      if((reg == SCODA_REG_GCR_DACL)||(reg == SCODA_REG_GCR_DACR)){
                                if(val < 32)
                                        val = 31 - val;
                                else
                                        val = 95 - val;
                        }*/
                        return val;
                }else
                        dev_err(codec->dev, "Cache read from %x failed: %d\n",
                                        reg, ret);
        }

        //if (icdc_d3_readable(codec, reg))
 //       if(reg == AK4376_DACL)
            //    return ak4376_i2c_read(ak4376,0x0B);
   //     else if(reg == AK4376_DACR)
              //  return ak4376_i2c_read(ak4376,0x0C);
        //else if(reg == AK4376_BALANCE)        
        //      return ak4376_i2c_read(ak4376,0x02);
     //   else
                return icdc_d3_hw_read(icdc_d3, reg);

        return 0;
}

static int icdc_d3_set_bias_level(struct snd_soc_codec *codec,
								  enum snd_soc_bias_level level) {
	DEBUG_MSG("%s enter set level %d\n", __func__, level);
	codec->dapm.bias_level = level;
	return 0;
}

static int icdc_d3_hw_params(struct snd_pcm_substream *substream,
							 struct snd_pcm_hw_params *params,
							 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct icdc_d3 *icdc_d3 = snd_soc_codec_get_drvdata(codec);
	int playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);
	int bit_width_sel = 3;
	int speed_sel = 0;
	int aicr_reg = playback ? SCODA_REG_AICR_DAC : SCODA_REG_AICR_ADC;
	int fcr_reg = playback ? SCODA_REG_FCR_DAC : SCODA_REG_FCR_ADC;
	int sample_attr[] = {	8000, 11025, 12000, 16000, 22050,  24000, 32000,44100,
							48000, 88200,  96000, 176400, 192000,384000};
	int speed = params_rate(params);
	int bit_width = params_format(params);
	DEBUG_MSG("%s enter  set bus width %d , sample rate %d\n",
			  __func__, bit_width, speed);
	/* bit width */
	switch (bit_width) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bit_width_sel = 0;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		bit_width_sel = 1;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		bit_width_sel = 2;
		break;
	default:
	case SNDRV_PCM_FORMAT_S24_3LE:
		bit_width_sel = 3;
		break;
	}

	/*sample rate*/
	for (speed_sel = 0; speed > sample_attr[speed_sel]; speed_sel++)
		;
	snd_soc_update_bits(codec, aicr_reg, SCODA_AICR_DAC_ADWL_MASK,
						(bit_width_sel << SCODA_AICR_DAC_ADWL_SHIFT));
	snd_soc_update_bits(codec, fcr_reg, SCODA_FCR_FREQ_MASK,
						(speed_sel << SCODA_FCR_FREQ_SHIFT));

	if(playback)
	{
		icdc_d3->playback_bit_width_sel = bit_width_sel;
		icdc_d3->playback_speed_sel = speed_sel;
	}else{

		icdc_d3->capture_bit_width_sel = bit_width_sel;
		icdc_d3->capture_speed_sel = speed_sel;
	}
	return 0;
}


static int icdc_d3_trigger(struct snd_pcm_substream * stream, int cmd,
						   struct snd_soc_dai *dai)
{
#ifdef DEBUG
	struct snd_soc_codec *codec = dai->codec;
	struct icdc_d3 *icdc_d3 = snd_soc_codec_get_drvdata(codec);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dump_registers_hazard(icdc_d3);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dump_registers_hazard(icdc_d3);
		break;
	}
#endif
	return 0;
}

#define DLV4780_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
						 SNDRV_PCM_FMTBIT_S20_3LE |SNDRV_PCM_FMTBIT_S24_LE)

static int jz_icdc_startup(struct snd_pcm_substream *substream,
						   struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_card *card = codec->card;
	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	/*power on codec*/
	icdc_d3->VIC_count++;
	if(icdc_d3->VIC_count==1){
		if (snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 0<<SCODA_CR_VIC_SB_SHIFT))
			msleep(250);
		if (snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 0<<SCODA_CR_VIC_SB_SLEEP_SHIFT))
			msleep(400);
	}
	mutex_unlock(&card->dapm_mutex);
	/*printk("icodec try to power on ak4376\n");
         gpio_direction_output(PO_EN, 1);
         mdelay(1);
         ak4376_cache_reg_init();
	ak4376_i2c_write(ak4376,0x0B,volumeL);
	ak4376_i2c_write(ak4376,0x0C,volumeR);
	*/
	return 0;
}

static void jz_icdc_shutdown(struct snd_pcm_substream *substream,
							 struct snd_soc_dai *dai)
{

	struct snd_soc_codec *codec = dai->codec;
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_card *card = codec->card;
	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	/*power off codec*/
	icdc_d3->VIC_count--;
	if(icdc_d3->VIC_count==0){
		snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1<<SCODA_CR_VIC_SB_SLEEP_SHIFT);
		snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1<<SCODA_CR_VIC_SB_SHIFT);
	}
	mutex_unlock(&card->dapm_mutex);
	/*printk("try to power down ak4376\n");
         gpio_direction_output(PO_EN, 0);
	*/
	return;
}

int icdc_d3_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_codec *codec = dai->codec;
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);

	struct snd_soc_card *card = codec->card;
	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	if(stream == SNDRV_PCM_STREAM_PLAYBACK) {
		icdc_d3->dac_set_mute = mute;
		switch (mute)
		{
			case 0 :
				if(!icdc_d3->amixer_set_mute)
					snd_soc_update_bits(codec, SCODA_REG_CR_DAC,SCODA_CR_DAC_SMUTE_MASK,
							0<<SCODA_CR_DAC_SMUTE_SHIFT);
				break;
			case 1 :
				if(icdc_d3->linein_set_mute)
					snd_soc_update_bits(codec, SCODA_REG_CR_DAC, SCODA_CR_DAC_SMUTE_MASK,
							1<<SCODA_CR_DAC_SMUTE_SHIFT);
				break;
		}

	} else if(stream == SNDRV_PCM_STREAM_CAPTURE) {
		switch (mute)
		{
			case 0 :
				icdc_d3->capture_mute_count++;
				if(icdc_d3->capture_mute_count==1)
					snd_soc_update_bits(codec, SCODA_REG_CR_ADC, SCODA_CR_ADC_SMUTE_MASK,
							0 << SCODA_CR_ADC_SMUTE_SHIFT);
				break;
			case 1 :
				icdc_d3->capture_mute_count--;
				if(icdc_d3->capture_mute_count==0)
					snd_soc_update_bits(codec, SCODA_REG_CR_ADC, SCODA_CR_ADC_SMUTE_MASK,
							1 << SCODA_CR_ADC_SMUTE_SHIFT);
				break;
		}
	}
	mutex_unlock(&card->dapm_mutex);
	return 0;
}


static struct snd_soc_dai_ops icdc_d3_dai_ops = {
	.hw_params	= icdc_d3_hw_params,
	.mute_stream = icdc_d3_mute_stream,
	.trigger = icdc_d3_trigger,
	.shutdown	= jz_icdc_shutdown,
	.startup	= jz_icdc_startup,
};

static struct snd_soc_dai_driver  icdc_d3_codec_dai = {
	.name = "icdc-d3-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
#if defined(CONFIG_SOC_4780)
		.rates = SNDRV_PCM_RATE_8000_96000,
#else
		.rates = SNDRV_PCM_RATE_8000_384000,//SNDRV_PCM_RATE_8000_192000,
#endif
		.formats = DLV4780_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
#if defined(CONFIG_SOC_4780)
		.rates = SNDRV_PCM_RATE_8000_96000,
#else
		.rates = SNDRV_PCM_RATE_8000_384000,//SNDRV_PCM_RATE_8000_192000,
#endif
		.formats = DLV4780_FORMATS,
	},
	.ops = &icdc_d3_dai_ops,
};

/* unit: 0.01dB */
static const DECLARE_TLV_DB_SCALE(dac_tlv, -10300, 50, 0);
static const DECLARE_TLV_DB_SCALE(mix_tlv, -3100, 100, 0);
static const DECLARE_TLV_DB_SCALE(adc_tlv, 2400, 50, 0);
static const DECLARE_TLV_DB_SCALE(mic_tlv, 0, 100, 0);

static const unsigned int icdc_d3_adc_mic_sel_value[] = {0x0, 0x1,};
static const unsigned int icdc_d3_mixer_input_sel_value[] = {0x0, 0x5, 0xa, 0xf,};
static const unsigned int icdc_d3_mixer_input_sel_value_double[] = {0x0, 0x1, 0x2, 0x3,};
static const unsigned int icdc_d3_mixer_mode_sel_value[] = {0x0, 0x1,};
static const char *icdc_d3_adc_mic_sel[] = { "AMIC ON", "DMIC ON"};

static const unsigned int icdc_d3_mix_sel_value[] = {0x0, 0x50, 0xa0, 0xf0};
static const char *icdc_d3_mix_sel[] = {"normal inputs",
										"cross inputs",
										"mixed inputs",
										"0 inputs"};

static const unsigned int icdc_d3_mux0_sel_value[] = {0x0, 0x1};
static const char *icdc_d3_mux0_sel[] = { "Playback DAC only",
										  "Playback DAC + ADC"};

static const unsigned int icdc_d3_mux2_sel_value[] = {0x0, 0x1};
static const char *icdc_d3_mux2_sel[] = { "Record input only",
	                                   "Record input + DAC"};



static const unsigned int icdc_d3_dmute_sel_value[] = {0x0, 0x1};
static const char *icdc_d3_dmute_sel[] = { "Digital umute",
	                                   "Digital mute"};


static const struct soc_enum icdc_d3_enum[] = {
	SOC_VALUE_ENUM_SINGLE(SCODA_REG_CR_ADC, 6, 0x1,  ARRAY_SIZE(icdc_d3_adc_mic_sel),icdc_d3_adc_mic_sel, icdc_d3_adc_mic_sel_value), /*0*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_0, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mix_sel),icdc_d3_mix_sel, icdc_d3_mix_sel_value), /*1*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_1, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mix_sel),icdc_d3_mix_sel, icdc_d3_mix_sel_value), /*2*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_2, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mix_sel),icdc_d3_mix_sel, icdc_d3_mix_sel_value), /*3*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_3, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mix_sel),icdc_d3_mix_sel, icdc_d3_mix_sel_value), /*4*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_0, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mux0_sel),icdc_d3_mux0_sel, icdc_d3_mux0_sel_value), /*5*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_MIX_2, 6, 4, 0x3,  ARRAY_SIZE(icdc_d3_mux2_sel),icdc_d3_mux2_sel, icdc_d3_mux2_sel_value), /*6*/
	SOC_VALUE_ENUM_DOUBLE(SCODA_REG_CR_DAC, 7, 1, 1,  ARRAY_SIZE(icdc_d3_dmute_sel),icdc_d3_dmute_sel, icdc_d3_dmute_sel_value), /*7*/
};

static const struct snd_kcontrol_new icdc_d3_adc_controls =
	SOC_DAPM_VALUE_ENUM("Route",  icdc_d3_enum[0]);

static int icdc_d3_Mix0_controls_get (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;

	value = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_0);
	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static int icdc_d3_Mix0_controls_put (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	unsigned int value = ucontrol->value.enumerated.item[0];
	unsigned int data = 0;
	struct snd_soc_card *card = codec->card;

	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	data = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_0);
	data &= 0x01;
	icdc_d3_hw_write_extend (icdc_d3, SCODA_MIX_0, icdc_d3_mix_sel_value[value]|data);
	mutex_unlock(&card->dapm_mutex);

	return 0;
}

static int icdc_d3_Mix1_controls_get (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;

	value = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_1);
	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static int icdc_d3_Mix1_controls_put (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	unsigned int value = ucontrol->value.enumerated.item[0];
	struct snd_soc_card *card = codec->card;

	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	icdc_d3_hw_write_extend (icdc_d3, SCODA_MIX_1, icdc_d3_mix_sel_value[value]);
	mutex_unlock(&card->dapm_mutex);

	return 0;
}

static int icdc_d3_Mix2_controls_get (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;

	value = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_2);
	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static int icdc_d3_Mix2_controls_put (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	unsigned int value = ucontrol->value.enumerated.item[0];
	unsigned int data = 0;
	struct snd_soc_card *card = codec->card;

	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	data = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_2);
	data &= 0x01;
	icdc_d3_hw_write_extend (icdc_d3, SCODA_MIX_2, icdc_d3_mix_sel_value[value]|data);
	mutex_unlock(&card->dapm_mutex);

	return 0;
}

static int icdc_d3_Mix3_controls_get (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;

	value = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_3);
	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static int icdc_d3_Mix3_controls_put (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	int value = ucontrol->value.enumerated.item[0];
	struct snd_soc_card *card = codec->card;

	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	icdc_d3_hw_write_extend (icdc_d3, SCODA_MIX_3, icdc_d3_mix_sel_value[value]);
	mutex_unlock(&card->dapm_mutex);

	return 0;
}

void icdc_d3_cr_mix_set(struct snd_soc_codec *codec , unsigned int value)
{
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	int *mix_cr_countp = &icdc_d3->mix_cr_count;
	unsigned int data = 0;

	data = icdc_d3_read(codec, SCODA_REG_CR_MIX);

	if(value==1){
		*mix_cr_countp += 1;
		if(*mix_cr_countp == 1){
			data |= 0x80;
			icdc_d3_write (codec, SCODA_REG_CR_MIX ,data);
		}
	}else{
		*mix_cr_countp -= 1;
		if(*mix_cr_countp == 0){
			data &= ~0x80;
			icdc_d3_write (codec, SCODA_REG_CR_MIX ,data);
		}
	}
}

static int icdc_d3_Mux0_controls_get (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;

	value = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_0);
	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static int icdc_d3_Mux0_controls_put (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	int value = ucontrol->value.enumerated.item[0];
	unsigned int data = 0;
	struct snd_soc_card *card = codec->card;

	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	data = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_0);
	if ((data&0x01) != value)
		icdc_d3_cr_mix_set(codec, value);

	icdc_d3_hw_write_extend (icdc_d3, SCODA_MIX_0, icdc_d3_mux0_sel_value[value]|(data&0xF0));
	mutex_unlock(&card->dapm_mutex);

	return 0;
}

static int icdc_d3_Mux2_controls_get (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	unsigned int value = 0;

	value = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_2);
	ucontrol->value.enumerated.item[0] = value;

	return 0;
}

static int icdc_d3_Mux2_controls_put (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);
	int value = ucontrol->value.enumerated.item[0];
	unsigned int data = 0;
	struct snd_soc_card *card = codec->card;

	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	data = icdc_d3_hw_read_extend(icdc_d3, SCODA_MIX_2);
	if ((data&0x01) != value)
		icdc_d3_cr_mix_set(codec, value);

	icdc_d3_hw_write_extend (icdc_d3, SCODA_MIX_2, icdc_d3_mux2_sel_value[value]|(data&0xF0));
	mutex_unlock(&card->dapm_mutex);


	return 0;
}





static int icdc_d3_dmute_controls_put (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int value = ucontrol->value.enumerated.item[0];
	struct snd_soc_card *card = codec->card;
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);


	mutex_lock_nested(&card->dapm_mutex, SND_SOC_DAPM_CLASS_RUNTIME);
	if(value)
		icdc_d3->amixer_set_mute = 1;
	else{
		icdc_d3->amixer_set_mute = 0;
		if(icdc_d3->linein_set_mute&&icdc_d3->dac_set_mute){
			mutex_unlock(&card->dapm_mutex);
			return 0;
		}
	}
	snd_soc_update_bits(codec, SCODA_REG_CR_DAC, SCODA_CR_DAC_SMUTE_MASK,
						 icdc_d3_dmute_sel_value[value]<<SCODA_CR_DAC_SMUTE_SHIFT);
	mutex_unlock(&card->dapm_mutex);
	return 0;
}



static int icdc_d3_dmute_controls_get (struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int value = 0;
	value = icdc_d3_read(codec, SCODA_REG_CR_DAC);
	value = value >> 7;
	ucontrol->value.integer.value[0] = value;
	return 0;

}

static const struct snd_kcontrol_new icdc_d3_snd_controls[] = {
	/* Volume controls */
	SOC_DOUBLE_R_TLV("Master Playback Volume", SCODA_REG_GCR_DACL, SCODA_REG_GCR_DACR, 0, 31, 0, dac_tlv),
	SOC_DOUBLE_R_TLV("TITANIUM Playback Volume", SCODA_REG_GCR_DACL, SCODA_REG_GCR_DACR, 0, 31, 0, dac_tlv),
	SOC_DOUBLE_R_TLV("Playback Mixer Volume", SCODA_REG_GCR_MIXDACL, SCODA_REG_GCR_MIXDACR, 0, 31, 0, mix_tlv),
	SOC_DOUBLE_R_TLV("Digital Capture Volume", SCODA_REG_GCR_ADCL, SCODA_REG_GCR_ADCR, 0, 43, 0, adc_tlv),
	SOC_DOUBLE_R_TLV("Digital Capture Mixer Volume", SCODA_REG_GCR_MIXADCL, SCODA_REG_GCR_MIXADCR, 0, 31, 0, mix_tlv),
	SOC_SINGLE_TLV("Mic Volume", SCODA_REG_GCR_MIC1, 0, 4, 0, mic_tlv),
	/* ADC private controls */
	SOC_SINGLE("ADC High Pass Filter Switch",SCODA_REG_FCR_ADC, 6, 1, 0),
	/* mic private controls */
	SOC_SINGLE_EXT("Digital Playback mute", SCODA_REG_CR_DAC, 7, 1, 0, icdc_d3_dmute_controls_get, icdc_d3_dmute_controls_put),

	SOC_ENUM_EXT("Mix0", icdc_d3_enum[1], icdc_d3_Mix0_controls_get,icdc_d3_Mix0_controls_put),
	SOC_SINGLE_TLV("DACL Playback Volume",AK4376_DACL,0, 60, 1, dac_tlv),
        SOC_SINGLE_TLV("DACR Playback Volume",AK4376_DACR,0, 60, 1, dac_tlv),
        //SOC_SINGLE_TLV("EQ Mode", 0x01,0, 20, 0, NULL),
        SOC_SINGLE("DAC Low Mode Switch",AK4376_BALANCE,0, 1, 0),

	SOC_ENUM_EXT("Mix1", icdc_d3_enum[2], icdc_d3_Mix1_controls_get,icdc_d3_Mix1_controls_put),
	SOC_ENUM_EXT("Mix2", icdc_d3_enum[3], icdc_d3_Mix2_controls_get,icdc_d3_Mix2_controls_put),
	SOC_ENUM_EXT("Mix3", icdc_d3_enum[4], icdc_d3_Mix3_controls_get,icdc_d3_Mix3_controls_put),
	SOC_ENUM_EXT("Mux0", icdc_d3_enum[5], icdc_d3_Mux0_controls_get,icdc_d3_Mux0_controls_put),
	SOC_ENUM_EXT("Mux2", icdc_d3_enum[6], icdc_d3_Mux2_controls_get,icdc_d3_Mux2_controls_put),


};



static int icdc_d3_vdigital_bypass_controls_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct icdc_d3 *icdc_d3  = snd_soc_codec_get_drvdata(codec);

	switch (event) {
		case SND_SOC_DAPM_PRE_PMU:

			icdc_d3->VIC_count++;
			if(icdc_d3->VIC_count==1){
				snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 0<<SCODA_CR_VIC_SB_SHIFT);
				snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK,
						0<<SCODA_CR_VIC_SB_SLEEP_SHIFT);
			}

			icdc_d3->linein_set_mute = 0;
			if(!icdc_d3->amixer_set_mute)
				snd_soc_update_bits(codec, SCODA_REG_CR_DAC,SCODA_CR_DAC_SMUTE_MASK,
							0<<SCODA_CR_DAC_SMUTE_SHIFT);


			icdc_d3->capture_mute_count++;
			if(icdc_d3->capture_mute_count==1)
		                 snd_soc_update_bits(codec, SCODA_REG_CR_ADC,SCODA_CR_ADC_SMUTE_MASK,
						0<<SCODA_CR_ADC_SMUTE_SHIFT);


			snd_soc_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_ADWL_MASK,
					(0 << SCODA_AICR_DAC_ADWL_SHIFT));
			snd_soc_update_bits(codec, SCODA_REG_FCR_DAC, SCODA_FCR_FREQ_MASK,
					(3 << SCODA_FCR_FREQ_SHIFT));                   //sample rate: 16KHZ


			snd_soc_update_bits(codec, SCODA_REG_AICR_ADC, SCODA_AICR_DAC_ADWL_MASK,
					(0 << SCODA_AICR_DAC_ADWL_SHIFT));
			snd_soc_update_bits(codec, SCODA_REG_FCR_ADC, SCODA_FCR_FREQ_MASK,
					(3 << SCODA_FCR_FREQ_SHIFT));                   //sample rate: 16KHZ
			break;

		case SND_SOC_DAPM_POST_PMD:
			icdc_d3->linein_set_mute = 1;
			if(icdc_d3->dac_set_mute)
				snd_soc_update_bits(codec, SCODA_REG_CR_DAC,SCODA_CR_DAC_SMUTE_MASK,
						1<<SCODA_CR_DAC_SMUTE_SHIFT);

			icdc_d3->capture_mute_count--;
			if(icdc_d3->capture_mute_count==0)
		                 snd_soc_update_bits(codec, SCODA_REG_CR_ADC,SCODA_CR_ADC_SMUTE_MASK,
						1<<SCODA_CR_ADC_SMUTE_SHIFT);

			icdc_d3->VIC_count--;
			if(icdc_d3->VIC_count==0){
				snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1<<SCODA_CR_VIC_SB_SHIFT);
				snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK,
						1<<SCODA_CR_VIC_SB_SLEEP_SHIFT);
			}


			if(icdc_d3->playback_bit_width_sel!=-1){
				snd_soc_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_ADWL_MASK,
						(icdc_d3->playback_bit_width_sel << SCODA_AICR_DAC_ADWL_SHIFT));
				snd_soc_update_bits(codec, SCODA_REG_FCR_DAC, SCODA_FCR_FREQ_MASK,
						(icdc_d3->playback_speed_sel << SCODA_FCR_FREQ_SHIFT));
			}

			if(icdc_d3->capture_bit_width_sel!=-1){
				snd_soc_update_bits(codec, SCODA_REG_AICR_ADC, SCODA_AICR_DAC_ADWL_MASK,
						(icdc_d3->capture_bit_width_sel << SCODA_AICR_DAC_ADWL_SHIFT));
				snd_soc_update_bits(codec, SCODA_REG_FCR_ADC, SCODA_FCR_FREQ_MASK,
						(icdc_d3->capture_speed_sel << SCODA_FCR_FREQ_SHIFT));
			}



			break;
		default:
			break;
	}
	return 0;
}



static const struct snd_kcontrol_new icdc_d3_vdigital_bypass_controls =
	SOC_DAPM_SINGLE("Switch", 0x11, 7, 1, 0);

static const struct snd_soc_dapm_widget icdc_d3_dapm_widgets[] = {
/* ADC */
	SND_SOC_DAPM_ADC("ADC", "Capture" , SCODA_REG_AICR_ADC, 4, 1),
	SND_SOC_DAPM_MUX("ADC Mux", SCODA_REG_CR_ADC, 4, 1, &icdc_d3_adc_controls),                        /*0*/
	SND_SOC_DAPM_MICBIAS("MICBIAS", SCODA_REG_CR_MIC1, 5, 1),
	SND_SOC_DAPM_PGA("AMIC", SCODA_REG_CR_MIC1, 4, 1, NULL, 0),
	SND_SOC_DAPM_PGA("DMIC", SCODA_REG_CR_DMIC, 7, 0, NULL, 0),

	SND_SOC_DAPM_DAC("DAC", "Playback", SCODA_REG_AICR_DAC, 4, 1),
	SND_SOC_DAPM_PGA("DAC_MERCURY", SCODA_REG_CR_DAC, 4, 1, NULL, 0),

	SND_SOC_DAPM_SWITCH_E("VDIGITAL BYPASS", SND_SOC_NOPM, 0, 0, &icdc_d3_vdigital_bypass_controls,
			icdc_d3_vdigital_bypass_controls_event,
			SND_SOC_DAPM_POST_PMD|SND_SOC_DAPM_PRE_PMU),

/* PINS */
	SND_SOC_DAPM_INPUT("AIP"),
	SND_SOC_DAPM_INPUT("AIN"),
	SND_SOC_DAPM_INPUT("DMIC IN"),
	SND_SOC_DAPM_OUTPUT("DO_LO_PWM"),
	SND_SOC_DAPM_OUTPUT("DO_BO_PWM"),
};

static const struct snd_soc_dapm_route intercon[] = {
	{ "MICBIAS",  NULL,  "AIP" },
	{ "MICBIAS",  NULL,  "AIN" },
	{ "AMIC",  NULL,  "MICBIAS" },
	/*input*/
	{ "ADC Mux", "AMIC ON", "AMIC" },
	{ "ADC Mux", "DMIC ON", "DMIC IN" },


	{ "ADC", NULL, "ADC Mux" },

	{"VDIGITAL BYPASS", "Switch", "ADC"},
	{"DAC", NULL, "VDIGITAL BYPASS"},

	/*output*/
	{ "DAC_MERCURY"  , NULL, "DAC" },
	{ "DO_LO_PWM", NULL, "DAC_MERCURY" },
};

#ifdef CONFIG_PM
static int icdc_d3_suspend(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, SCODA_REG_AICR_ADC, SCODA_AICR_ADC_SB_MASK, 1<<SCODA_AICR_ADC_SB_SHIFT);
	snd_soc_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SB_MASK, 1<<SCODA_AICR_DAC_SB_SHIFT);
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1<<SCODA_CR_VIC_SB_SLEEP_SHIFT);
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1<<SCODA_CR_VIC_SB_SHIFT);
	icdc_d3_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_update_bits(codec, SCODA_REG_CR_CK, SCODA_CR_CK_SDCLK_MASK, 1<<SCODA_CR_CK_SDCLK_SHIFT);

	return 0;
}

static int icdc_d3_resume(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, SCODA_REG_CR_CK, SCODA_CR_CK_SDCLK_MASK, 0<<SCODA_CR_CK_SDCLK_SHIFT);
	icdc_d3_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	if (snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 0<<SCODA_CR_VIC_SB_SHIFT))
		msleep(250);
	if (snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 0<<SCODA_CR_VIC_SB_SLEEP_SHIFT)) {
		msleep(10);
	}
	snd_soc_update_bits(codec, SCODA_REG_AICR_ADC, SCODA_AICR_ADC_SB_MASK, 0<<SCODA_AICR_ADC_SB_SHIFT);
	snd_soc_update_bits(codec, SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SB_MASK, 0<<SCODA_AICR_DAC_SB_SHIFT);

	return 0;
}
#endif

static int icdc_d3_probe(struct snd_soc_codec *codec)
{
	struct icdc_d3 *icdc_d3 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "codec icdc-d3 probe enter\n");
	/* power off codec */
	icdc_d3_set_bias_level(codec ,SND_SOC_BIAS_STANDBY);
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_SLEEP_MASK, 1<<SCODA_CR_VIC_SB_SLEEP_SHIFT);
	snd_soc_update_bits(codec, SCODA_REG_CR_VIC, SCODA_CR_VIC_SB_MASK, 1<<SCODA_CR_VIC_SB_SHIFT);

#ifdef DEBUG
	/*dump for debug*/
	dump_registers_hazard(icdc_d3);
#endif
	/* codec select enable 24M clock*/
	snd_soc_update_bits(codec, SCODA_REG_CR_CK , SCODA_CR_CK_MCLK_DIV_MASK, 1 << SCODA_CR_CK_MCLK_DIV_SHIFT);
	snd_soc_update_bits(codec, SCODA_REG_CR_CK , SCODA_CR_CK_SDCLK_MASK, 0 << SCODA_CR_CK_SDCLK_SHIFT);
	snd_soc_update_bits(codec, SCODA_REG_CR_CK , SCODA_CR_CRYSTAL_MASK, 0 << SCODA_CR_CRYSTAL_SHIFT);

	/*codec select Dac/Adc i2s interface*/
	snd_soc_update_bits(codec,SCODA_REG_AICR_DAC, SCODA_AICR_DAC_SLAVE_MASK, 0<<SCODA_AICR_DAC_SLAVE_SHIFT);
	snd_soc_update_bits(codec,SCODA_REG_AICR_DAC, SCODA_AICR_DAC_AUDIO_MASK, SCODA_AICR_DAC_AUDIOIF_I2S<<SCODA_AICR_DAC_AUDIOIF_SHIFT);

	/*codec generated IRQ is a high level */
	snd_soc_update_bits(codec, SCODA_REG_ICR, SCODA_ICR_INT_FORM_MASK, SCODA_ICR_INT_FORM_HIGH<<SCODA_ICR_INT_FORM_SHIFT);

	/*codec irq mask*/
	snd_soc_write(codec, SCODA_REG_IMR, SCODA_IMR_COMMON_MASK);
	snd_soc_write(codec, SCODA_REG_IMR2, SCODA_IMR2_COMMON_MASK);

	/*codec clear all irq*/
	snd_soc_write(codec, SCODA_REG_IFR, SCODA_IMR_COMMON_MASK);
	snd_soc_write(codec, SCODA_REG_IFR2, SCODA_IMR2_COMMON_MASK);

	icdc_d3_hw_write_extend (icdc_d3, SCODA_MIX_2, (0<<6)| (1<<4));
	icdc_d3->mix_cr_count = 0;
	icdc_d3->codec = codec;
	icdc_d3->playback_speed_sel = -1;
	icdc_d3->playback_bit_width_sel = -1;
	icdc_d3->capture_speed_sel = -1;
	icdc_d3->capture_bit_width_sel = -1;


	icdc_d3->capture_mute_count = 0;
	icdc_d3->VIC_count = 0;

	icdc_d3->amixer_set_mute = 0;
	icdc_d3->dac_set_mute = 1;
	icdc_d3->linein_set_mute = 1;


	return 0;
}

static int icdc_d3_remove(struct snd_soc_codec *codec)
{
	dev_info(codec->dev, "codec icdc_d3 remove enter\n");
	icdc_d3_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_icdc_d3_codec = {
	.probe = 	icdc_d3_probe,
	.remove = 	icdc_d3_remove,
#ifdef CONFIG_PM
	.suspend =	icdc_d3_suspend,
	.resume =	icdc_d3_resume,
#endif
	.read = 	icdc_d3_read,
	.write = 	icdc_d3_write,
	.volatile_register = icdc_d3_volatile,
	.readable_register = icdc_d3_readable,
	.writable_register = icdc_d3_writable,
	.reg_cache_default = icdc_d3_reg_defcache,
	.reg_word_size = sizeof(u8),
	.reg_cache_step = 1,
	.reg_cache_size = SCODA_MAX_REG_NUM,
	.set_bias_level = icdc_d3_set_bias_level,

	.controls = 	icdc_d3_snd_controls,
	.num_controls = ARRAY_SIZE(icdc_d3_snd_controls),
	.dapm_widgets = icdc_d3_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(icdc_d3_dapm_widgets),
	.dapm_routes = intercon,
	.num_dapm_routes = ARRAY_SIZE(intercon),
	.ignore_pmdown_time = 1,
};

/*Just for debug*/
static ssize_t icdc_d3_regs_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct icdc_d3 *icdc_d3 = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = icdc_d3->codec;
	if (!codec) {
		dev_info(dev, "icdc_d3 is not probe, can not use %s function\n", __func__);
		return 0;
	}
	mutex_lock(&codec->mutex);
	dump_registers_hazard(icdc_d3);
	mutex_unlock(&codec->mutex);
	return 0;
}

static ssize_t icdc_d3_regs_store(struct device *dev, struct device_attribute *attr,
								  const char *buf, size_t count)
{
	struct icdc_d3 *icdc_d3 = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = icdc_d3->codec;
	const char *start = buf;
	unsigned int reg, val;
	int ret_count = 0;

	if (!codec) {
		dev_info(dev, "icdc_d3 is not probe, can not use %s function\n", __func__);
		return count;
	}

	while(!isxdigit(*start)) {
		start++;
		if (++ret_count >= count)
			return count;
	}
	reg = simple_strtoul(start, (char **)&start, 16);
	while(!isxdigit(*start)) {
		start++;
		if (++ret_count >= count)
			return count;
	}
	val = simple_strtoul(start, (char **)&start, 16);
	mutex_lock(&codec->mutex);
	icdc_d3_write(codec, reg, val);
	mutex_unlock(&codec->mutex);
	return count;
}

static DEVICE_ATTR(hw_regs, S_IWUSR | S_IRUSR,
				   icdc_d3_regs_show, icdc_d3_regs_store);

static struct attribute *icdc_d3_attributes[] = {
	&dev_attr_hw_regs.attr,
	NULL
};

static const struct attribute_group icdc_d3_attr_group = {
	.attrs = icdc_d3_attributes,
};

static int icdc_d3_platform_probe(struct platform_device *pdev)
{
	struct icdc_d3 *icdc_d3 = NULL;
	struct resource *res = NULL;
	int ret = 0;

	icdc_d3 = (struct icdc_d3*)devm_kzalloc(&pdev->dev,
											sizeof(struct icdc_d3), GFP_KERNEL);
	if (!icdc_d3)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Faild to get ioresource mem\n");
		return -ENOENT;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start,
								 resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "Failed to request mmio memory region\n");
		return -EBUSY;
	}
	icdc_d3->mapped_resstart = res->start;
	icdc_d3->mapped_ressize = resource_size(res);
	icdc_d3->mapped_base = devm_ioremap_nocache(&pdev->dev,
												icdc_d3->mapped_resstart,
												icdc_d3->mapped_ressize);
	if (!icdc_d3->mapped_base) {
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res)
		return -ENOENT;
	icdc_d3->dev = &pdev->dev;
	icdc_d3->dac_user_mute = 1;
	icdc_d3->aohp_in_pwsq = 0;
	spin_lock_init(&icdc_d3->io_lock);
	platform_set_drvdata(pdev, (void *)icdc_d3);

	ret = snd_soc_register_codec(&pdev->dev,
								 &soc_codec_dev_icdc_d3_codec, &icdc_d3_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Faild to register codec\n");
		platform_set_drvdata(pdev, NULL);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &icdc_d3_attr_group);
	if (ret)
		dev_warn(&pdev->dev,"attribute create failed : %x",ret);

	dev_info(&pdev->dev, "codec icdc-d3 platfrom probe success\n");

	return 0;
}

static int icdc_d3_platform_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "codec icdc-d3 platform remove\n");
	snd_soc_unregister_codec(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver icdc_d3_codec_driver = {
	.driver = {
		.name = "icdc-d3",
		.owner = THIS_MODULE,
	},
	.probe = icdc_d3_platform_probe,
	.remove = icdc_d3_platform_remove,
};

static int icdc_d3_modinit(void)
{
	return platform_driver_register(&icdc_d3_codec_driver);
}
module_init(icdc_d3_modinit);

static void icdc_d3_exit(void)
{
	platform_driver_unregister(&icdc_d3_codec_driver);
}
module_exit(icdc_d3_exit);

MODULE_DESCRIPTION("iCdc d3 Codec Driver");
MODULE_AUTHOR("sccheng<shicheng.cheng@ingenic.com>");
MODULE_LICENSE("GPL");
