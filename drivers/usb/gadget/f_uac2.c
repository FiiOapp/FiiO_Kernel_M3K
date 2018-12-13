/*
 * f_uac2.c -- USB Audio Class 2.0 Function
 *
 * Copyright (C) 2011
 *    Yadwinder Singh (yadi.brar01@gmail.com)
 *    Jaswinder Singh (jaswinder.singh@linaro.org)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/usb/audio.h>
#include <linux/usb/audio-v2.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

/* Playback(USB-IN) Default Stereo - Fl/Fr */
static int p_chmask = 0x3;
module_param(p_chmask, uint, S_IRUGO);
MODULE_PARM_DESC(p_chmask, "Playback Channel Mask");

/* Playback Default 48 KHz */
static int p_srate = 48000;
module_param(p_srate, uint, S_IRUGO);
MODULE_PARM_DESC(p_srate, "Playback Sampling Rate");

/* Playback Default 16bits/sample */
static int p_ssize = 2;
module_param(p_ssize, uint, S_IRUGO);
MODULE_PARM_DESC(p_ssize, "Playback Sample Size(bytes)");

/* Capture(USB-OUT) Default Stereo - Fl/Fr */
static int c_chmask = 0x3;
module_param(c_chmask, uint, S_IRUGO);
MODULE_PARM_DESC(c_chmask, "Capture Channel Mask");

/* Capture Default 64 KHz */
static int c_srate = 48000;
module_param(c_srate, uint, S_IRUGO);
MODULE_PARM_DESC(c_srate, "Capture Sampling Rate");

/* Capture Default 16bits/sample */
static int c_ssize = 2; //在电脑端将会只显示16bit的选择项，如果还需要显示24bit的，需要初始化为3
module_param(c_ssize, uint, S_IRUGO);
MODULE_PARM_DESC(c_ssize, "Capture Sample Size(bytes)");

static int c_debug = 0;
module_param(c_debug, uint, S_IRUGO);
MODULE_PARM_DESC(c_debug, "FiiO debug flag");

/* Keep everyone on toes */
#define USB_XFERS	8

/*
 * The driver implements a simple UAC_2 topology.
 * USB-OUT -> IT_1 -> OT_3 -> ALSA_Capture
 * ALSA_Playback -> IT_2 -> OT_4 -> USB-IN
 * Capture and Playback sampling rates are independently
 *  controlled by two clock sources :
 *    CLK_5 := c_srate, and CLK_6 := p_srate
 */
#define USB_OUT_IT_ID	1
#define IO_IN_IT_ID	2
#define IO_OUT_OT_ID	3
#define USB_IN_OT_ID	4
#define USB_OUT_CLK_ID	5
#define USB_IN_CLK_ID	6

#define CONTROL_ABSENT	0
#define CONTROL_RDONLY	1
#define CONTROL_RDWR	3

#define CLK_FREQ_CTRL	0
#define CLK_VLD_CTRL	2

#define COPY_CTRL	0
#define CONN_CTRL	2
#define OVRLD_CTRL	4
#define CLSTR_CTRL	6
#define UNFLW_CTRL	8
#define OVFLW_CTRL	10

//debug
//#define FIIO_DEBUG_UAC2
#ifdef FIIO_DEBUG_UAC2
#define fiio_debug(x...)  printk(KERN_INFO "[fiio_uac2] " x)
#else
#define fiio_debug(x...)
#endif
const char *uac2_name = "snd_uac2";

struct uac2_req {
	struct uac2_rtd_params *pp; /* parent param */
	struct usb_request *req;
};

struct uac2_rtd_params {

	struct snd_uac2_chip *uac2; /* parent chip */

	bool ep_enabled; /* if the ep is enabled */
	/* Size of the ring buffer */
	size_t dma_bytes;
	unsigned char *dma_area;

	struct snd_pcm_substream *ss;

	/* Ring buffer */
	ssize_t hw_ptr;

	void *rbuf;

	size_t period_size;

	unsigned max_psize;
	struct uac2_req ureq[USB_XFERS];

	spinlock_t lock;
};

struct snd_uac2_chip {
	struct platform_device pdev;
	struct platform_driver pdrv;

	struct uac2_rtd_params p_prm;
	struct uac2_rtd_params c_prm;

	struct snd_card *card;
	struct snd_pcm *pcm;
	/* timekeeping for the playback endpoint */
	unsigned int p_interval;
	unsigned int p_residue;

	/* pre-calculated values for playback iso completion */
	unsigned int p_pktsize;
	unsigned int p_pktsize_residue;
	unsigned int p_framesize;
};

#define BUFF_SIZE_MAX	(PAGE_SIZE * 16 * 4)
#define PRD_SIZE_MAX	PAGE_SIZE * 4
#define MIN_PERIODS	4
#define UAC2_MIN_RATE 32000
#define UAC2_MAX_RATE 192000
static struct snd_pcm_hardware uac2_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER
		 | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID
		 | SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.rates = SNDRV_PCM_RATE_CONTINUOUS,
	.periods_max = BUFF_SIZE_MAX / PRD_SIZE_MAX,
	.buffer_bytes_max = BUFF_SIZE_MAX,
	.period_bytes_max = PRD_SIZE_MAX * 4,
	.periods_min = MIN_PERIODS,
	.rate_min		= UAC2_MIN_RATE,
	.rate_max		= UAC2_MAX_RATE,
};

struct audio_dev {
	u8 ac_intf, ac_alt;
	u8 as_out_intf, as_out_alt;
	u8 as_in_intf, as_in_alt;

	struct usb_ep *in_ep, *out_ep;
	struct usb_function func;

	/* The ALSA Sound Card it represents on the USB-Client side */
	struct snd_uac2_chip uac2;
};

static struct audio_dev *agdev_g;

static inline
struct audio_dev *func_to_agdev(struct usb_function *f)
{
	return container_of(f, struct audio_dev, func);
}

static inline
struct audio_dev *uac2_to_agdev(struct snd_uac2_chip *u)
{
	return container_of(u, struct audio_dev, uac2);
}

static inline
struct snd_uac2_chip *pdev_to_uac2(struct platform_device *p)
{
	return container_of(p, struct snd_uac2_chip, pdev);
}

static inline
struct snd_uac2_chip *prm_to_uac2(struct uac2_rtd_params *r)
{
	struct snd_uac2_chip *uac2 = container_of(r,
					struct snd_uac2_chip, c_prm);

	if (&uac2->c_prm != r)
		uac2 = container_of(r, struct snd_uac2_chip, p_prm);

	return uac2;
}

static inline
uint num_channels(uint chanmask)
{
	uint num = 0;

	while (chanmask) {
		num += (chanmask & 1);
		chanmask >>= 1;
	}

	return num;
}
//////////////////////////////////////////////////
/*
    add for send uevent to system, when sample changed
*/
static int uac2_open(struct inode *ip, struct file *fp)
{
	return 0;
}

static int uac2_release(struct inode *ip, struct file *fp)
{
	return 0;
}

/* file operations */
static const struct file_operations uac2_fops = {
	.owner = THIS_MODULE,
	.open = uac2_open,
	.release = uac2_release,
};

static struct miscdevice uac2_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "usb_dac",
	.fops = &uac2_fops,
};

static int send_uac2_sample_change_uevent(bool status)
{
	char *start_envp[2] = { "UAC_STATE=START", NULL };
	char *stop_envp[2] = { "UAC_STATE=STOP", NULL };

	if (status) {
		fiio_debug("%s fiio send uac start play event[UAC_STATE=START]!\n",__func__);
		kobject_uevent_env(&uac2_device.this_device->kobj, KOBJ_CHANGE, start_envp);
	}  
	else {
		fiio_debug("%s fiio send uac stop play event[UAC_STATE=STOP]!\n",__func__);
		kobject_uevent_env(&uac2_device.this_device->kobj, KOBJ_CHANGE, stop_envp);
	}
	return 0;
}
static int UAC2_RATE_VALUE_32000 = 32;
static int UAC2_RATE_VALUE_44100 = 44;
static int UAC2_RATE_VALUE_48000 = 48;
static int UAC2_RATE_VALUE_88200 = 88;
static int UAC2_RATE_VALUE_96000 = 96;
static int UAC2_RATE_VALUE_176400 = 176;
static int UAC2_RATE_VALUE_192000 = 192;
static void init_crate_value() {
	switch(c_ssize) {
		case 2:
			UAC2_RATE_VALUE_32000 = 32;
			UAC2_RATE_VALUE_44100 = 44;
			UAC2_RATE_VALUE_48000 = 48;
			UAC2_RATE_VALUE_88200 = 88;
			UAC2_RATE_VALUE_96000 = 96;
			UAC2_RATE_VALUE_176400 = 176;
			UAC2_RATE_VALUE_192000 = 192;
			break;
		case 3:
			UAC2_RATE_VALUE_32000 = 48;
			UAC2_RATE_VALUE_44100 = 66;
			UAC2_RATE_VALUE_48000 = 72;
			UAC2_RATE_VALUE_88200 = 132;
			UAC2_RATE_VALUE_96000 = 144;
			UAC2_RATE_VALUE_176400 = 264;
			UAC2_RATE_VALUE_192000 = 288;
			break;
		case 4:
			UAC2_RATE_VALUE_32000 = 64;
			UAC2_RATE_VALUE_44100 = 88;
			UAC2_RATE_VALUE_48000 = 96;
			UAC2_RATE_VALUE_88200 = 176;
			UAC2_RATE_VALUE_96000 = 192;
			UAC2_RATE_VALUE_176400 = 352;
			UAC2_RATE_VALUE_192000 = 384;
		break;
	}
}
//32bit
// #define UAC2_RATE_VALUE_32000 64
// #define UAC2_RATE_VALUE_44100 88
// #define UAC2_RATE_VALUE_48000 96
// #define UAC2_RATE_VALUE_88200 176
// #define UAC2_RATE_VALUE_96000 192
// #define UAC2_RATE_VALUE_176400 352
// #define UAC2_RATE_VALUE_192000 384

//24bit
// #define UAC2_RATE_VALUE_32000 48
// #define UAC2_RATE_VALUE_44100 66
// #define UAC2_RATE_VALUE_48000 72
// #define UAC2_RATE_VALUE_88200 132
// #define UAC2_RATE_VALUE_96000 144
// #define UAC2_RATE_VALUE_176400 264
// #define UAC2_RATE_VALUE_192000 288

//16bit
// #define UAC2_RATE_VALUE_32000 32
// #define UAC2_RATE_VALUE_44100 44
// #define UAC2_RATE_VALUE_48000 48
// #define UAC2_RATE_VALUE_88200 88
// #define UAC2_RATE_VALUE_96000 96
// #define UAC2_RATE_VALUE_176400 176
// #define UAC2_RATE_VALUE_192000 192


#define UAC2_RATE_32000 32000
#define UAC2_RATE_44100 44100
#define UAC2_RATE_48000 48000
#define UAC2_RATE_88200 88200
#define UAC2_RATE_96000 96000
#define UAC2_RATE_176400 176400
#define UAC2_RATE_192000 192000
#define UAC2_DEFAULT_RATE 48000



static int reset_uac2_rate(int freq)
{
    int actual_rate = 0;
	#if 0
    switch(freq) {
        case UAC2_RATE_VALUE_32000:
            actual_rate = UAC2_RATE_32000;
            break;
        case UAC2_RATE_VALUE_44100:
            actual_rate = UAC2_RATE_44100;
            break;
        case UAC2_RATE_VALUE_48000:
            actual_rate = UAC2_RATE_48000;
            break;    
        case UAC2_RATE_VALUE_88200:
            actual_rate = UAC2_RATE_88200;
            break;
        case UAC2_RATE_VALUE_96000:
            actual_rate = UAC2_RATE_96000;
            break; 
        case UAC2_RATE_VALUE_176400:
            actual_rate = UAC2_RATE_176400;
            break; 
        case UAC2_RATE_VALUE_192000:
            actual_rate = UAC2_RATE_192000;
            break;

        default :
            actual_rate = UAC2_DEFAULT_RATE;
            break;            
    }
	#endif
	if (freq == UAC2_RATE_VALUE_32000) {
		actual_rate = UAC2_RATE_32000;
	}
	else if (freq == UAC2_RATE_VALUE_44100) {
		actual_rate = UAC2_RATE_44100;
	}
	else if (freq == UAC2_RATE_VALUE_48000) {
		actual_rate = UAC2_RATE_48000;
	}
	else if (freq == UAC2_RATE_VALUE_88200) {
		actual_rate = UAC2_RATE_88200;
	}
	else if (freq == UAC2_RATE_VALUE_96000) {
		actual_rate = UAC2_RATE_96000;
	}
	else if (freq == UAC2_RATE_VALUE_176400) {
		actual_rate = UAC2_RATE_176400;
	}
	else if (freq == UAC2_RATE_VALUE_192000) {
		actual_rate = UAC2_RATE_192000;
	}
	else {
		actual_rate = UAC2_DEFAULT_RATE;
	}
    return actual_rate;
}
static bool fiio_uac2_status = false;
static bool uac2_binded = false;
static void uac2_sendevent_work(struct work_struct *data)
{
    send_uac2_sample_change_uevent(fiio_uac2_status);
}

static bool dug_uac_flags = false;
static struct work_struct uevent_work;
static int mCheckUpdateSampleCounter = 0;
//////////////////////////////////////////////
static void
agdev_iso_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned pending;
	unsigned long flags;
	bool update_alsa = false;
	unsigned char *src, *dst;
	int status = req->status;
	struct uac2_req *ur = req->context;
	struct snd_pcm_substream *substream;
	struct uac2_rtd_params *prm = ur->pp;
	struct snd_uac2_chip *uac2 = prm_to_uac2(prm);
    int actual_value;
	unsigned int hw_ptr;
	/* i/f shutting down */
	if (!prm->ep_enabled || req->status == -ESHUTDOWN) {
		// fiio_debug("%s: iso_complete status(%d) %d/%d ESHUTDOWN(%d)\n",
		// 	__func__, status, req->actual, req->length,ESHUTDOWN);
		return;
	}
		

	/*
	 * We can't really do much about bad xfers.
	 * Afterall, the ISOCH xfers could fail legitimately.
	 */
	if (status)
		pr_debug("%s: iso_complete status(%d) %d/%d\n",
			__func__, status, req->actual, req->length);
		// fiio_debug("%s: iso_complete status(%d) %d/%d\n",
		// __func__, status, req->actual, req->length);
	//////////////////////////
	 if (dug_uac_flags) {
		mCheckUpdateSampleCounter++;
		actual_value = req->actual;
		if (actual_value != req->length && actual_value != 0) {
			fiio_debug("%s actual_value=%d req->length=%d mCheckUpdateSampleCounter=%d"
			,__func__,actual_value,req->length,mCheckUpdateSampleCounter);
		// }
		// if (mCheckUpdateSampleCounter > 10) {
		// 	fiio_debug("%s mCheckUpdateSampleCounter=%d",__func__,mCheckUpdateSampleCounter);
			mCheckUpdateSampleCounter = 0;
			actual_value = req->actual;
			dug_uac_flags = false;
			/* printk("%s: actual_value is  %d \n", __func__, actual_value);
			printk("%s: old opts->c_srate is  %d \n", __func__, opts->c_srate); */

			/* update current sample rate */
			c_srate = reset_uac2_rate(actual_value);
			fiio_debug("%s: new c_srate is  %d actual_value=%d req->length=%d \n",
			__func__, c_srate,actual_value,req->length);

			/* send Uevent to UsbAudioClass Activity */
			fiio_uac2_status = true;
			schedule_work(&uevent_work);
		}
		
	}
	/////////////////////////////////
	substream = prm->ss;

	/* Do nothing if ALSA isn't active */
	if (!substream)
		goto exit;

	spin_lock_irqsave(&prm->lock, flags);
	//delete it for solve play with noise,check with uac2 from kernel up to 3.18
#if 0
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		src = prm->dma_area + prm->hw_ptr;
		req->actual = req->length;
		dst = req->buf;
	} else {
		dst = prm->dma_area + prm->hw_ptr;
		src = req->buf;
	}

	pending = prm->hw_ptr % prm->period_size;
	pending += req->actual;
	if (pending >= prm->period_size)
		update_alsa = true;

	prm->hw_ptr = (prm->hw_ptr + req->actual) % prm->dma_bytes;

	spin_unlock_irqrestore(&prm->lock, flags);

	/* Pack USB load in ALSA ring buffer */
	memcpy(dst, src, req->actual);
#else
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*
		 * For each IN packet, take the quotient of the current data
		 * rate and the endpoint's interval as the base packet size.
		 * If there is a residue from this division, add it to the
		 * residue accumulator.
		 */
		req->length = uac2->p_pktsize;
		uac2->p_residue += uac2->p_pktsize_residue;

		/*
		 * Whenever there are more bytes in the accumulator than we
		 * need to add one more sample frame, increase this packet's
		 * size and decrease the accumulator.
		 */
		if (uac2->p_residue / uac2->p_interval >= uac2->p_framesize) {
			req->length += uac2->p_framesize;
			uac2->p_residue -= uac2->p_framesize *
					   uac2->p_interval;
		}

		req->actual = req->length;
	}

	if (c_debug == 1) {
		printk("prm->hw_ptr=%d prm->period_size=%d req->actual=%d prm->dma_bytes=%d\n",
			prm->hw_ptr,prm->period_size,req->actual,prm->dma_bytes);
	}
	pending = prm->hw_ptr % prm->period_size;
	pending += req->actual;
	if (pending >= prm->period_size)
		update_alsa = true;
	if (c_debug == 1) {
		printk("pending=%d ",pending);
	}
	hw_ptr = prm->hw_ptr;
	prm->hw_ptr = (prm->hw_ptr + req->actual) % prm->dma_bytes;

	spin_unlock_irqrestore(&prm->lock, flags);

	/* Pack USB load in ALSA ring buffer */
	pending = prm->dma_bytes - hw_ptr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (unlikely(pending < req->actual)) {
			memcpy(req->buf, prm->dma_area + hw_ptr, pending);
			memcpy(req->buf + pending, prm->dma_area,
			       req->actual - pending);
		} else {
			memcpy(req->buf, prm->dma_area + hw_ptr, req->actual);
		}
	} else {
		if (unlikely(pending < req->actual)) {
			memcpy(prm->dma_area + hw_ptr, req->buf, pending);
			memcpy(prm->dma_area, req->buf + pending,
			       req->actual - pending);
		} else {
			memcpy(prm->dma_area + hw_ptr, req->buf, req->actual);
		}
	}

#endif
exit:
	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(&uac2->pdev.dev, "%d Error!\n", __LINE__);

	if (update_alsa)
		snd_pcm_period_elapsed(substream);

	return;
}

static int
uac2_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct uac2_rtd_params *prm;
	unsigned long flags;
	int err = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac2->p_prm;
	else
		prm = &uac2->c_prm;

	spin_lock_irqsave(&prm->lock, flags);

	/* Reset */
	prm->hw_ptr = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		prm->ss = substream;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		prm->ss = NULL;
		break;
	default:
		err = -EINVAL;
	}

	spin_unlock_irqrestore(&prm->lock, flags);

	/* Clear buffer after Play stops */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && !prm->ss)
		memset(prm->rbuf, 0, prm->max_psize * USB_XFERS);

	return err;
}

static snd_pcm_uframes_t uac2_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct uac2_rtd_params *prm;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac2->p_prm;
	else
		prm = &uac2->c_prm;

	return bytes_to_frames(substream->runtime, prm->hw_ptr);
}

static int uac2_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct uac2_rtd_params *prm;
	int err;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac2->p_prm;
	else
		prm = &uac2->c_prm;

	err = snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
	if (err >= 0) {
		prm->dma_bytes = substream->runtime->dma_bytes;
		prm->dma_area = substream->runtime->dma_area;
		prm->period_size = params_period_bytes(hw_params);
	}

	return err;
}

static int uac2_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct uac2_rtd_params *prm;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac2->p_prm;
	else
		prm = &uac2->c_prm;

	prm->dma_area = NULL;
	prm->dma_bytes = 0;
	prm->period_size = 0;

	return snd_pcm_lib_free_pages(substream);
}

static int uac2_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->hw = uac2_pcm_hardware;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		fiio_debug("****%s %d SNDRV_PCM_STREAM_PLAYBACK p_srate=%d\n",__func__,__LINE__,p_srate);
		spin_lock_init(&uac2->p_prm.lock);
		runtime->hw.rate_min = p_srate;
		switch (p_ssize) {
		case 3:
			runtime->hw.formats = SNDRV_PCM_FMTBIT_S24_3LE;
			break;
		case 4:
			runtime->hw.formats = SNDRV_PCM_FMTBIT_S32_LE;
			break;
		default:
			runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE;
			break;
		}
		runtime->hw.channels_min = num_channels(p_chmask);
		runtime->hw.period_bytes_min = 2 * uac2->p_prm.max_psize
						/ runtime->hw.periods_min;
	} else {
		fiio_debug("****%s %d SNDRV_PCM_STREAM_CAPTURE c_srate=%d\n",__func__,__LINE__,c_srate);
		spin_lock_init(&uac2->c_prm.lock);
		fiio_debug("%s %d c_srate=%d\n",__func__,__LINE__,c_srate);
		runtime->hw.rate_min = c_srate;
		switch (c_ssize) {
		case 3:
			runtime->hw.formats = SNDRV_PCM_FMTBIT_S24_3LE;
			break;
		case 4:
			runtime->hw.formats = SNDRV_PCM_FMTBIT_S32_LE;
			break;
		default:
			runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE;
			break;
		}
		runtime->hw.channels_min = num_channels(c_chmask);
		runtime->hw.period_bytes_min = 2 * uac2->c_prm.max_psize
						/ runtime->hw.periods_min;
	}

	runtime->hw.rate_max = runtime->hw.rate_min;
	runtime->hw.channels_max = runtime->hw.channels_min;
	fiio_debug("%s %d runtime->hw.rate_min=%d\n",__func__,__LINE__,runtime->hw.rate_min);
	fiio_debug("%s %d runtime->hw.rate_max=%d\n",__func__,__LINE__,runtime->hw.rate_max);
	fiio_debug("%s %d runtime->hw.channels_max=%d\n",__func__,__LINE__,runtime->hw.channels_max);
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	return 0;
}

/* ALSA cries without these function pointers */
static int uac2_pcm_null(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_pcm_ops uac2_pcm_ops = {
	.open = uac2_pcm_open,
	.close = uac2_pcm_null,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = uac2_pcm_hw_params,
	.hw_free = uac2_pcm_hw_free,
	.trigger = uac2_pcm_trigger,
	.pointer = uac2_pcm_pointer,
	.prepare = uac2_pcm_null,
};

#include <asm/uaccess.h>
#include <linux/fs.h>   
#include <linux/syscalls.h>

int check_file(char *filen) {
	struct file  *cali_file;
	cali_file = filp_open(filen, O_RDONLY,0);
	 if(IS_ERR(cali_file))
	{
		fiio_debug("%s %d.\n",__func__,__LINE__);
	    return 0;
	}
	 else {
	 	fiio_debug("%s %d.\n",__func__,__LINE__);
	 	filp_close(cali_file,NULL);
		return 1;
	 }
} 

static int snd_uac2_probe(struct platform_device *pdev)
{
	struct snd_uac2_chip *uac2 = pdev_to_uac2(pdev);
	struct snd_card *card;
	struct snd_pcm *pcm;
	int err;
	int ret = 0;
	fiio_debug("%s: enter %d !!!\n", __func__,__LINE__);

	init_crate_value();

	/* Choose any slot, with no id */
	err = snd_card_create(-1, NULL, THIS_MODULE, 0, &card);
	if (err < 0)
		return err;

	uac2->card = card;

	/*
	 * Create first PCM device
	 * Create a substream only for non-zero channel streams
	 */
	err = snd_pcm_new(uac2->card, "UAC2 PCM", 0,
			       p_chmask ? 1 : 0, c_chmask ? 1 : 0, &pcm);
	if (err < 0)
		goto snd_fail;

	strcpy(pcm->name, "UAC2 PCM");
	pcm->private_data = uac2;

	uac2->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &uac2_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &uac2_pcm_ops);

	strcpy(card->driver, "UAC2_Gadget");
	strcpy(card->shortname, "UAC2_Gadget");
	sprintf(card->longname, "UAC2_Gadget %i", pdev->id);

	snd_card_set_dev(card, &pdev->dev);
	
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
	snd_dma_continuous_data(GFP_KERNEL), 0, BUFF_SIZE_MAX);

	err = snd_card_register(card);
	
	if (!err) {
		platform_set_drvdata(pdev, card);
		return 0;
	}


snd_fail:
	snd_card_free(card);

	uac2->pcm = NULL;
	uac2->card = NULL;

	return err;
}

static int snd_uac2_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct snd_card *card = platform_get_drvdata(pdev);
	fiio_debug("%s: enter %d !!!\n", __func__,__LINE__);
	if (card) {

		ret = snd_card_free(card);
		platform_set_drvdata(pdev, NULL);  

		return ret;
	}
		
	return 0;
}

static void snd_uac2_release(struct device *dev)
{
	fiio_debug("%s: enter !!!\n", __func__);
}

static int alsa_uac2_init(struct audio_dev *agdev)
{
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	int err;

	fiio_debug("%s: enter !!!\n", __func__);

	uac2->pdrv.probe = snd_uac2_probe;
	uac2->pdrv.remove = snd_uac2_remove;
	uac2->pdrv.driver.name = uac2_name;

	uac2->pdev.id = 0;
	uac2->pdev.name = uac2_name;
	uac2->pdev.dev.release = snd_uac2_release;

	/* Register snd_uac2 driver */
	err = platform_driver_register(&uac2->pdrv);
	if (err)
		return err;

	/* Register snd_uac2 device */
	err = platform_device_register(&uac2->pdev);
	if (err)
		platform_driver_unregister(&uac2->pdrv);

	return err;
}


static void alsa_uac2_exit(struct audio_dev *agdev)
{
	struct snd_uac2_chip *uac2 = &agdev->uac2;

	fiio_debug("%s: enter !!!\n", __func__);

	platform_driver_unregister(&uac2->pdrv);
	platform_device_unregister(&uac2->pdev);
}


/* --------- USB Function Interface ------------- */

enum {
	STR_ASSOC,
	STR_IF_CTRL,
	STR_CLKSRC_OUT,
	STR_USB_IT,
	STR_IO_OT,
	STR_AS_OUT_ALT0,
	STR_AS_OUT_ALT1,
};

static char clksrc_out[8];


static struct usb_string strings_fn[] = {
	[STR_ASSOC].s = "FiiO M3K",
	[STR_IF_CTRL].s = "FiiO M3K",
	[STR_CLKSRC_OUT].s = "FiiO M3K",//clksrc_out,
	[STR_USB_IT].s = "FiiO M3K",
	[STR_IO_OT].s = "FiiO M3K",
	[STR_AS_OUT_ALT0].s = "FiiO M3K",
	[STR_AS_OUT_ALT1].s = "FiiO M3K",
	{ },
};


static struct usb_gadget_strings str_fn = {
	.language = 0x0409,	/* en-us */
	.strings = strings_fn,
};

static struct usb_gadget_strings *fn_strings[] = {
	&str_fn,
	NULL,
};

static struct usb_qualifier_descriptor devqual_desc = {
	.bLength = sizeof devqual_desc,
	.bDescriptorType = USB_DT_DEVICE_QUALIFIER,

	.bcdUSB = cpu_to_le16(0x200),
	.bDeviceClass = USB_CLASS_MISC,
	.bDeviceSubClass = 0x02,
	.bDeviceProtocol = 0x01,
	.bNumConfigurations = 1,
	.bRESERVED = 0,
};

static struct usb_interface_assoc_descriptor iad_desc = {
	.bLength = sizeof iad_desc,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface = 0,
	.bInterfaceCount = 3,
	.bFunctionClass = USB_CLASS_AUDIO,
	.bFunctionSubClass = UAC2_FUNCTION_SUBCLASS_UNDEFINED,
	.bFunctionProtocol = UAC_VERSION_2,
};

/* Audio Control Interface */
static struct usb_interface_descriptor std_ac_if_desc = {
	.bLength = sizeof std_ac_if_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOCONTROL,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* Clock source for OUT traffic */
struct uac_clock_source_descriptor out_clk_src_desc = {
	.bLength = sizeof out_clk_src_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC2_CLOCK_SOURCE,
	.bClockID = USB_OUT_CLK_ID,
	.bmAttributes = UAC_CLOCK_SOURCE_TYPE_INT_PROG,//UAC_CLOCK_SOURCE_TYPE_INT_FIXED,
	//.bmControls = (CONTROL_RDONLY << CLK_FREQ_CTRL),
	.bmControls = 0x07, //设置支持的采样率个数
	.bAssocTerminal = 0,
};

/* Input Terminal for USB_OUT *///host
struct uac2_input_terminal_descriptor usb_out_it_desc = {
	.bLength = sizeof usb_out_it_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_INPUT_TERMINAL,
	.bTerminalID = USB_OUT_IT_ID,
	.wTerminalType = cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal = 0,
	.bCSourceID = USB_OUT_CLK_ID,
	.iChannelNames = 0,
	.bmControls = cpu_to_le16(0x0),//(CONTROL_RDWR << COPY_CTRL),
};

/* Ouput Terminal for I/O-Out */
struct uac2_output_terminal_descriptor io_out_ot_desc = {
	.bLength = sizeof io_out_ot_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_OUTPUT_TERMINAL,
	.bTerminalID = IO_OUT_OT_ID,
	.wTerminalType = cpu_to_le16(UAC_OUTPUT_TERMINAL_HEADPHONES),
	.bAssocTerminal = 0,
	.bSourceID = USB_OUT_IT_ID,
	.bCSourceID = USB_OUT_CLK_ID,
	.bmControls = 0x0,//(CONTROL_RDWR << COPY_CTRL),
};

static struct uac2_ac_header_descriptor ac_hdr_desc = {
	.bLength = sizeof ac_hdr_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_MS_HEADER,
	.bcdADC = cpu_to_le16(0x200),
	.bCategory = UAC2_FUNCTION_HEADSET,
	.wTotalLength =  0x40,
	.bmControls = 0,
};

/* Audio Streaming OUT Interface - Alt0 */
static struct usb_interface_descriptor std_as_out_if0_desc = {
	.bLength = sizeof std_as_out_if0_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* Audio Streaming OUT Interface - Alt1 */
static struct usb_interface_descriptor std_as_out_if1_desc = {
	.bLength = sizeof std_as_out_if1_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* Audio Stream OUT Intface Desc */
struct uac2_as_header_descriptor as_out_hdr_desc = {
	.bLength = sizeof as_out_hdr_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_AS_GENERAL,
	.bTerminalLink = USB_OUT_IT_ID,
	.bmControls = 0,
	.bFormatType = UAC_FORMAT_TYPE_I,
	.bmFormats = cpu_to_le32(UAC_FORMAT_TYPE_I_PCM),
	.iChannelNames = 0,
};

/* Audio USB_OUT Format */
struct uac2_format_type_i_descriptor as_out_fmt1_desc = {
	.bLength = sizeof as_out_fmt1_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = UAC_FORMAT_TYPE,
	.bFormatType = UAC_FORMAT_TYPE_I,
};

/* STD AS ISO OUT Endpoint */
struct usb_endpoint_descriptor fs_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT,

	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE,
	//.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,

	.wMaxPacketSize = cpu_to_le16(384), 
	
	.bInterval = 1,

	.bRefresh = 0,
	.bSynchAddress = 0,
};

struct usb_endpoint_descriptor hs_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE,
	//.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,

	.wMaxPacketSize = cpu_to_le16(384),
	
	.bInterval = 2,
	.bRefresh = 0,
	.bSynchAddress = 0,
};

/* CS AS ISO OUT Endpoint */
static struct uac2_iso_endpoint_descriptor as_iso_out_desc = {
	.bLength = sizeof as_iso_out_desc,
	.bDescriptorType = USB_DT_CS_ENDPOINT,

	.bDescriptorSubtype = UAC_EP_GENERAL,
	.bmAttributes = 0,
	.bmControls = 0,
	.bLockDelayUnits = 0,
	.wLockDelay = 0,
};

static struct usb_descriptor_header *fs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&out_clk_src_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&fs_epout_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&out_clk_src_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&hs_epout_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,
	NULL,
};

static struct cntrl_cur_lay3 {
	__u32	dCUR;
};

static struct cntrl_rang_sub {
	__u32	dMIN;
	__u32	dMAX;
	__u32	dRES;
} __packed;

struct cntrl_range_lay3 {
	__u16	wNumSubRanges;

    struct cntrl_rang_sub sub[7];
} __packed;

static int sample_rate[7] = {32000, 44100, 48000, 88200, 
                             96000, 176400, 192000};

static struct cntrl_range_lay3 cntrl_r;

static inline void
free_ep(struct uac2_rtd_params *prm, struct usb_ep *ep)
{
	struct snd_uac2_chip *uac2 = prm_to_uac2(prm);
	int i;

	if (!prm->ep_enabled)
		return;
	prm->ep_enabled = false;

	for (i = 0; i < USB_XFERS; i++) {
		if (prm->ureq[i].req) {
			usb_ep_dequeue(ep, prm->ureq[i].req);
			usb_ep_free_request(ep, prm->ureq[i].req);
			prm->ureq[i].req = NULL;
		}
	}

	if (usb_ep_disable(ep))
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
}

static int __init
afunc_bind(struct usb_configuration *cfg, struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	struct usb_composite_dev *cdev = cfg->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct uac2_rtd_params *prm;
	int ret;

	//////////////////////////////
	struct usb_string *us;

 	fiio_debug("%s: enter !!!\n", __func__);

	us = usb_gstrings_attach(cdev, fn_strings, ARRAY_SIZE(strings_fn));
	if (IS_ERR(us))
		return PTR_ERR(us);

	//iad_desc.iFunction = us[STR_ASSOC].id;
	std_ac_if_desc.iInterface = us[STR_IF_CTRL].id;
	out_clk_src_desc.iClockSource = us[STR_CLKSRC_OUT].id;
	usb_out_it_desc.iTerminal = us[STR_USB_IT].id;
	io_out_ot_desc.iTerminal = us[STR_IO_OT].id;
	std_as_out_if0_desc.iInterface = us[STR_AS_OUT_ALT0].id;
	std_as_out_if1_desc.iInterface = us[STR_AS_OUT_ALT1].id;

	/* Initialize the configurable parameters */
	
	usb_out_it_desc.bNrChannels = num_channels(c_chmask);
	as_out_hdr_desc.bNrChannels = num_channels(c_chmask);
	#if 0
	usb_out_it_desc.bmChannelConfig = cpu_to_le32(c_chmask);
	
	as_out_hdr_desc.bmChannelConfig = cpu_to_le32(c_chmask);
	#endif
	as_out_fmt1_desc.bSubslotSize = c_ssize;
	as_out_fmt1_desc.bBitResolution = c_ssize * 8;

	snprintf(clksrc_out, sizeof(clksrc_out), "%uHz", c_srate);
	/////////////////////////////
	ret = usb_interface_id(cfg, fn);
	if (ret < 0) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}
	std_ac_if_desc.bInterfaceNumber = ret;
	agdev->ac_intf = ret;
	agdev->ac_alt = 0;

	ret = usb_interface_id(cfg, fn);
	if (ret < 0) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}
	std_as_out_if0_desc.bInterfaceNumber = ret;
	std_as_out_if1_desc.bInterfaceNumber = ret;
	agdev->as_out_intf = ret;
	agdev->as_out_alt = 0;

	agdev->out_ep = usb_ep_autoconfig(gadget, &fs_epout_desc);
	if (!agdev->out_ep) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		goto err;
	}
	agdev->out_ep->driver_data = agdev;

	//uac2->p_prm.uac2 = uac2;
	//uac2->c_prm.uac2 = uac2;

	hs_epout_desc.bEndpointAddress = fs_epout_desc.bEndpointAddress;
	hs_epout_desc.wMaxPacketSize = fs_epout_desc.wMaxPacketSize;

	ret = usb_assign_descriptors(fn, fs_audio_desc, hs_audio_desc, NULL);
	if (ret)
		goto err_free_descs;

	prm = &agdev->uac2.c_prm;
	prm->max_psize = hs_epout_desc.wMaxPacketSize;
	prm->rbuf = kzalloc(prm->max_psize * USB_XFERS, GFP_KERNEL);
	if (!prm->rbuf) {
		prm->max_psize = 0;
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		goto err_free_descs;
	}
        
    if (!uac2_binded) {
        ret = alsa_uac2_init(agdev);
        if (ret)
	        goto err_free_descs;
            uac2_binded = true;
    }

	return 0;

err_free_descs:
usb_free_all_descriptors(fn);

	kfree(agdev->uac2.p_prm.rbuf);
	kfree(agdev->uac2.c_prm.rbuf);
err:	
	if (agdev->in_ep)
		agdev->in_ep->driver_data = NULL;
	if (agdev->out_ep)
		agdev->out_ep->driver_data = NULL;
	return -EINVAL;
}

static void
afunc_unbind(struct usb_configuration *cfg, struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct uac2_rtd_params *prm;

	alsa_uac2_exit(agdev);

	fiio_debug("%s: enter !!!\n", __func__);
#if 1
	flush_work(&uevent_work);
#endif
	prm = &agdev->uac2.p_prm;
	if (prm->rbuf) {
		kfree(prm->rbuf);
		prm->rbuf = NULL;
	}

	prm = &agdev->uac2.c_prm;
	if (prm->rbuf) {
		kfree(prm->rbuf);
		prm->rbuf = NULL;
	}

	usb_free_all_descriptors(fn);

	if (agdev->in_ep)
		agdev->in_ep->driver_data = NULL;
	if (agdev->out_ep)
		agdev->out_ep->driver_data = NULL;
#if 0
	if (agdev != NULL)
		kfree(agdev);
#endif
}

static int
afunc_set_alt(struct usb_function *fn, unsigned intf, unsigned alt)
{
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	struct usb_gadget *gadget = cdev->gadget;
	struct device *dev = &uac2->pdev.dev;
	struct usb_request *req;
	struct usb_ep *ep;
	struct uac2_rtd_params *prm;
	int req_len, i;
    
	fiio_debug("+++++++++++++++++begin %s %d\n",__func__,__LINE__);

	/* No i/f has more than 2 alt settings */
	if (alt > 1) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (intf == agdev->ac_intf) {
		/* Control I/f has only 1 AltSetting - 0 */
		if (alt) {
			dev_err(&uac2->pdev.dev,
				"%s:%d Error!\n", __func__, __LINE__);
			return -EINVAL;
		}
		return 0;
	}
	fiio_debug("%s %d intf=%d as_out_intf=%d as_in_intf=%d alt=%d\n",
	__func__,__LINE__,intf,agdev->as_out_intf,agdev->as_in_intf,alt);
	if (intf == agdev->as_out_intf) {
		ep = agdev->out_ep;
		prm = &uac2->c_prm;
		config_ep_by_speed(gadget, fn, ep);
		agdev->as_out_alt = alt;
		req_len = prm->max_psize;
		fiio_debug(">>>>%s line=%d req_len=%d\n",__func__,__LINE__,req_len);
	} 
	else if (intf == agdev->as_in_intf) {
	#if 1
		unsigned int factor, rate;
		struct usb_endpoint_descriptor *ep_desc;
	#endif
		ep = agdev->in_ep;
		prm = &uac2->p_prm;
		config_ep_by_speed(gadget, fn, ep);
		agdev->as_in_alt = alt;
		req_len = prm->max_psize;
	#if 1
		/* pre-calculate the playback endpoint's interval */
		if (gadget->speed == USB_SPEED_FULL) {
			ep_desc = &fs_epout_desc;
			factor = 1000;
		} else {
			ep_desc = &hs_epout_desc;
			factor = 125;
		}
		fiio_debug("%s line=%d factor=%d\n",__func__,__LINE__,factor);
		/* pre-compute some values for iso_complete() */
		uac2->p_framesize = p_ssize *
				    num_channels(p_chmask);
		fiio_debug("%s %d p_srate=%d\n",__func__,__LINE__,p_srate);
		rate = p_srate * uac2->p_framesize;
		uac2->p_interval = (1 << (ep_desc->bInterval - 1)) * factor;
		uac2->p_pktsize = min_t(unsigned int, rate / uac2->p_interval,
					prm->max_psize);
		fiio_debug("%s line=%d p_framesize=%d rate=%d uac2->p_interval=%d uac2->p_pktsize=%d\n"
		,__func__,__LINE__,uac2->p_framesize,rate,uac2->p_interval,uac2->p_pktsize);
		if (uac2->p_pktsize < prm->max_psize)
			uac2->p_pktsize_residue = rate % uac2->p_interval;
		else
			uac2->p_pktsize_residue = 0;

		req_len = uac2->p_pktsize;
		uac2->p_residue = 0;
		fiio_debug("%s line=%d req_len=%d prm->max_psize=%d\n",__func__,__LINE__,req_len,prm->max_psize);
	#endif
	} 
	else {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	
	if (alt == 0) {
		free_ep(prm, ep);
		return 0;
	}
		
	prm->ep_enabled = true;
	usb_ep_enable(ep);

	for (i = 0; i < USB_XFERS; i++) {
		if (!prm->ureq[i].req) {
			req = usb_ep_alloc_request(ep, GFP_ATOMIC);
			if (req == NULL)
				return -ENOMEM;

			prm->ureq[i].req = req;
			prm->ureq[i].pp = prm;

			req->zero = 0;
			req->context = &prm->ureq[i];
			req->length = req_len;
			req->complete = agdev_iso_complete;
			req->buf = prm->rbuf + i * prm->max_psize;
			fiio_debug("%s line=%d prm->max_psize=%d req_len=%d\n"
			,__func__,__LINE__,prm->max_psize,req_len);
		}

		if (usb_ep_queue(ep, prm->ureq[i].req, GFP_ATOMIC))
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
	}

	///////////////////////////
	if (fiio_uac2_status) {
		fiio_uac2_status = false;
		schedule_work(&uevent_work);
	}
	///////////////////////////
	//////////////////////
	dug_uac_flags = true;
	///////////////////////
	return 0;
}

static int
afunc_get_alt(struct usb_function *fn, unsigned intf)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;

	if (intf == agdev->ac_intf)
		return agdev->ac_alt;
	else if (intf == agdev->as_out_intf)
		return agdev->as_out_alt;
	else if (intf == agdev->as_in_intf)
		return agdev->as_in_alt;
	else
		dev_err(&uac2->pdev.dev,
			"%s:%d Invalid Interface %d!\n",
			__func__, __LINE__, intf);

	return -EINVAL;
}

static void
afunc_disable(struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;

	fiio_debug("%s: enter !!!\n", __func__);

	free_ep(&uac2->p_prm, agdev->in_ep);
	agdev->as_in_alt = 0;

	free_ep(&uac2->c_prm, agdev->out_ep);
	agdev->as_out_alt = 0;
}

static int
in_rq_cur(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_request *req = fn->config->cdev->req;
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	u16 w_length = le16_to_cpu(cr->wLength);
	u16 w_index = le16_to_cpu(cr->wIndex);
	u16 w_value = le16_to_cpu(cr->wValue);
	u8 entity_id = (w_index >> 8) & 0xff;
	u8 control_selector = w_value >> 8;
	int value = -EOPNOTSUPP;

	if (control_selector == UAC2_CS_CONTROL_SAM_FREQ) {
		struct cntrl_cur_lay3 c;

		c.dCUR = c_srate;

		value = min_t(unsigned, w_length, sizeof c);
		memcpy(req->buf, &c, value);
	} else if (control_selector == UAC2_CS_CONTROL_CLOCK_VALID) {
		*(u8 *)req->buf = 1;
		value = min_t(unsigned, w_length, 1);
	} else {
		dev_err(&uac2->pdev.dev,
			"%s:%d control_selector=%d TODO!\n",
			__func__, __LINE__, control_selector);
	}

	return value;
}

/* add multiple sampling rates support */

static int set_support_sample_rates(void)
{
	int i;

	for(i = 0; i < 7; i++)
	{
		cntrl_r.sub[i].dMIN = sample_rate[i];
		cntrl_r.sub[i].dMAX = sample_rate[i];
		cntrl_r.sub[i].dRES = 0;
	}

	return 0;
}


static int
in_rq_range(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_request *req = fn->config->cdev->req;
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	u16 w_length = le16_to_cpu(cr->wLength);
	u16 w_index = le16_to_cpu(cr->wIndex);
	u16 w_value = le16_to_cpu(cr->wValue);
	u8 control_selector = w_value >> 8;
	int value = -EOPNOTSUPP;

	if (control_selector == UAC2_CS_CONTROL_SAM_FREQ) {
		cntrl_r.wNumSubRanges = 7;
                
    	set_support_sample_rates();

		value = min_t(unsigned, w_length, sizeof cntrl_r);
		memcpy(req->buf, &cntrl_r, value);
	} else {
		dev_err(&uac2->pdev.dev,
			"%s:%d control_selector=%d TODO!\n",
			__func__, __LINE__, control_selector);
	}

	return value;
}

static int
ac_rq_in(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	if (cr->bRequest == UAC2_CS_CUR)
		return in_rq_cur(fn, cr);
	else if (cr->bRequest == UAC2_CS_RANGE)
		return in_rq_range(fn, cr);
	else
		return -EOPNOTSUPP;
}

static int
out_rq_cur(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	u16 w_length = le16_to_cpu(cr->wLength);
	u16 w_value = le16_to_cpu(cr->wValue);
	u8 control_selector = w_value >> 8;

	if (control_selector == UAC2_CS_CONTROL_SAM_FREQ)
		return w_length;

	return -EOPNOTSUPP;
}

static int
setup_rq_inf(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	u16 w_index = le16_to_cpu(cr->wIndex);
	u8 intf = w_index & 0xff;

	if (intf != agdev->ac_intf) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return -EOPNOTSUPP;
	}

	if (cr->bRequestType & USB_DIR_IN)
		return ac_rq_in(fn, cr);
	else if (cr->bRequest == UAC2_CS_CUR)
		return out_rq_cur(fn, cr);

	return -EOPNOTSUPP;
}

static int
afunc_setup(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	struct usb_request *req = cdev->req;
	u16 w_length = le16_to_cpu(cr->wLength);
	int value = -EOPNOTSUPP;

	/* Only Class specific requests are supposed to reach here */
	if ((cr->bRequestType & USB_TYPE_MASK) != USB_TYPE_CLASS)
		return -EOPNOTSUPP;

	if ((cr->bRequestType & USB_RECIP_MASK) == USB_RECIP_INTERFACE)
		value = setup_rq_inf(fn, cr);
	else
		dev_err(&uac2->pdev.dev, "%s:%d Error!\n", __func__, __LINE__);

	if (value >= 0) {
		req->length = value;
		req->zero = value < w_length;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0) {
			dev_err(&uac2->pdev.dev,
				"%s:%d Error!\n", __func__, __LINE__);
			req->status = 0;
		}
	}

	return value;
}

static void afunc_free(struct usb_function *f)
{
	struct audio_dev *agdev;
	struct f_uac2_opts *opts;

    fiio_debug("%s: enter !!!\n", __func__);
#if 0
	agdev = func_to_agdev(f);
	
	alsa_uac2_exit(agdev);

	kfree(agdev);
#endif

}


static int audio_bind_config(struct usb_configuration *cfg)
{
	int res;
	
	fiio_debug("%s: enter !!!\n", __func__);
	
	agdev_g = kzalloc(sizeof *agdev_g, GFP_KERNEL);
	if (agdev_g == NULL) {
		printk(KERN_ERR "Unable to allocate audio gadget\n");
		return -ENOMEM;
	}
#if 1
	res = usb_string_ids_tab(cfg->cdev, strings_fn);
	if (res)
		return res;
#endif
#if 0

	iad_desc.iFunction = strings_fn[STR_ASSOC].id;
	std_ac_if_desc.iInterface = strings_fn[STR_IF_CTRL].id;


	out_clk_src_desc.iClockSource = strings_fn[STR_CLKSRC_OUT].id;
	usb_out_it_desc.iTerminal = strings_fn[STR_USB_IT].id;


	io_out_ot_desc.iTerminal = strings_fn[STR_IO_OT].id;

	std_as_out_if0_desc.iInterface = strings_fn[STR_AS_OUT_ALT0].id;
	std_as_out_if1_desc.iInterface = strings_fn[STR_AS_OUT_ALT1].id;
#endif
	agdev_g->func.name = "uac2_func";
	agdev_g->func.strings = fn_strings;
	agdev_g->func.bind = afunc_bind;
	agdev_g->func.unbind = afunc_unbind;
	agdev_g->func.set_alt = afunc_set_alt;
	agdev_g->func.get_alt = afunc_get_alt;
	agdev_g->func.disable = afunc_disable;
	agdev_g->func.setup = afunc_setup;
	//agdev_g->func.free_func = afunc_free;
	/* Initialize the configurable parameters */
	#if 0
	usb_out_it_desc.bNrChannels = num_channels(c_chmask);
	as_out_hdr_desc.bNrChannels = num_channels(c_chmask);
	
	usb_out_it_desc.bmChannelConfig = cpu_to_le32(c_chmask);
	as_out_hdr_desc.bmChannelConfig = cpu_to_le32(c_chmask);

	as_out_fmt1_desc.bSubslotSize = c_ssize;
	as_out_fmt1_desc.bBitResolution = c_ssize * 8;
	fiio_debug("%s %d c_srate=%d\n",__func__,__LINE__,c_srate);

	snprintf(clksrc_out, sizeof(clksrc_out), "%uHz", c_srate);
	#endif

	res = usb_add_function(cfg, &agdev_g->func);
	if (res < 0)
		kfree(agdev_g);
	////////////begin////////////////
	#if 1
	fiio_uac2_status = false;

    INIT_WORK(&uevent_work, uac2_sendevent_work);

	res = misc_register(&uac2_device);
	if (res) {
		pr_err("%s: uac2_device register failed.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	uac2_binded = false;
	#endif
	////////////end//////////////
	return res;
}

static void
uac2_unbind_config(struct usb_configuration *cfg)
{
	
	fiio_debug("%s: enter !!!\n", __func__);

	
	kfree(agdev_g);
	agdev_g = NULL;
	#if 1
	///////////////////
	misc_deregister(&uac2_device);
	///////////////////
	#endif
}
MODULE_LICENSE("GPL");

