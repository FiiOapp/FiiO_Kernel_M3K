/*
 * f_audio.c -- USB Audio class function driver
  *
 * Copyright (C) 2008 Bryan Wu <cooloney@kernel.org>
 * Copyright (C) 2008 Analog Devices, Inc
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <asm/atomic.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>

#include "u_audio.h"
#define OUT_EP_MAX_PACKET_SIZE	512//1024  	// 200 -> 512 change by billy for fiio

static int req_buf_size = OUT_EP_MAX_PACKET_SIZE;
module_param(req_buf_size, int, S_IRUGO);
MODULE_PARM_DESC(req_buf_size, "ISO OUT endpoint request buffer size");

static int req_count = 64;//64;//256;
module_param(req_count, int, S_IRUGO);
MODULE_PARM_DESC(req_count, "ISO OUT endpoint request count");

static int audio_buf_size = 48000;
module_param(audio_buf_size, int, S_IRUGO);
MODULE_PARM_DESC(audio_buf_size, "Audio buffer size");

/* The first usb audio buf to alsa playback */
static int first_audio_buf_size = 65500;
module_param(first_audio_buf_size, int, S_IRUGO);
MODULE_PARM_DESC(first_audio_buf_size, "First Audio buffer size");


/* Capture(USB-OUT) Default Stereo - Fl/Fr */
static int c_chmask = 0x3;
module_param(c_chmask, uint, S_IRUGO);
MODULE_PARM_DESC(c_chmask, "Capture Channel Mask");

/* Capture Default 64 KHz */
static int c_srate = 44100;
module_param(c_srate, uint, S_IRUGO);
MODULE_PARM_DESC(c_srate, "Capture Sampling Rate");

/* Capture Default 16bits/sample */
static int c_ssize = 2;
module_param(c_ssize, uint, S_IRUGO);
MODULE_PARM_DESC(c_ssize, "Capture Sample Size(bytes)");

static struct f_audio *audio_for_usb;

int switch_control = 0;
bool dug_uac_flags = false;
int actual_rate = UAC_DEFAULT_RATE;
int actual_format = 16;
static bool first_copy_audio_buffer = true;
static bool uac2_diable_flag = false;
static bool uac2_enable_flag = false;
bool uac_flags = false;
static struct task_struct *uac2_task = NULL;

static int f_uac2_init(void);

/* Keep everyone on toes */
#define USB_XFERS	2

static unsigned int g_audio = 0;
static unsigned int g_rate = 0;

static ssize_t
g_audio_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u", actual_rate);
}

static ssize_t
g_audio_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	sscanf(buf, "%u", &g_audio) ;
	return n;
}

static struct kobj_attribute g_audio_attribute =
      __ATTR(g_audio, 0666, g_audio_show, g_audio_store);


static ssize_t
g_rate_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u", actual_format);
}

static ssize_t
g_rate_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	sscanf(buf, "%u", &g_rate);
	return n;
}

static struct kobj_attribute g_rate_attribute =
      __ATTR(g_rate, 0666, g_rate_show, g_rate_store);

static DEVICE_ATTR(g_audio, 0644, g_audio_show, g_audio_store);

static struct attribute *g_audio_attributes[] = {
	&g_audio_attribute.attr,
	&g_rate_attribute.attr,
	NULL
};

static const struct attribute_group g_audio_attr_group = {
	.attrs = g_audio_attributes,
};

static struct kobject *g_kobj;

/*
 * DESCRIPTORS ... most are static, but strings and full
 * configuration descriptors are built on demand.
 */
#define USB_OUT_IT_ID	1
#define IO_IN_IT_ID	2
#define IO_OUT_OT_ID	3
#define USB_IN_OT_ID	4
#define USB_OUT_CLK_ID	5
#define USB_IN_CLK_ID	6

/*******************/
static struct usb_string strings_dev[] = {
	[0].s = MANUFATURER,
	[1].s = DRIVER_DESC,
	[2].s = "FiiO X3",
	[3].s = "FiiO X3",
	[4].s = "Internal Clock",
	[5].s = "FiiO X3",
	[6].s = "Master Channel",
	[7].s = "Left Channel",
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *audio_strings[] = {
	&stringtab_dev,
	NULL,
};

/*
 * We have two interfaces- AudioControl and AudioStreaming
 * TODO: only supcard playback currently
 */
static struct usb_qualifier_descriptor devqual_desc = {
	.bLength = sizeof devqual_desc,
	.bDescriptorType = USB_DT_DEVICE_QUALIFIER,
	.bcdUSB =  cpu_to_le16(0x200),
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
	.bLength =		 sizeof std_ac_if_desc,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bAlternateSetting = 0,
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_AUDIO,
	.bInterfaceSubClass =	USB_SUBCLASS_AUDIOCONTROL,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* B.3.2  Class-Specific AC Interface Descriptor */
static struct uac2_ac_header_descriptor ac_hdr_desc = {
	.bLength = sizeof ac_hdr_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_MS_HEADER,
	.bcdADC = cpu_to_le16(0x0200),
	.bCategory = UAC2_FUNCTION_HEADSET,
	.wTotalLength = 0x40,
	.bmControls =	0,
};

/* Clock source for OUT traffic */
static struct uac_clock_source_descriptor out_clk_src_desc = {
	.bLength = sizeof out_clk_src_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC2_CLOCK_SOURCE,
	.bClockID = USB_OUT_CLK_ID,		
	.bmAttributes = UAC_CLOCK_SOURCE_TYPE_INT_PROG,		
	.bmControls = 0x07,		
	.bAssocTerminal = 0,		
};

static struct uac2_input_terminal_descriptor usb_out_it_desc = {
	.bLength =	sizeof usb_out_it_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_INPUT_TERMINAL,
	.bTerminalID =		USB_OUT_IT_ID,
	.wTerminalType =	cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal =	0,
	.bCSourceID = USB_OUT_CLK_ID,
	.iChannelNames = 0,
	.bmControls = cpu_to_le16(0x0),
};

/* Ouput Terminal for I/O-Out */
static struct uac2_output_terminal_descriptor io_out_ot_desc = {
	.bLength		=  sizeof io_out_ot_desc,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubtype	= UAC_OUTPUT_TERMINAL,
	.bTerminalID		= IO_OUT_OT_ID,
	.wTerminalType		= cpu_to_le16(UAC_OUTPUT_TERMINAL_HEADPHONES),
	.bAssocTerminal		= 0,
	.bSourceID		= USB_OUT_IT_ID,
	.bCSourceID = USB_OUT_CLK_ID,
	.bmControls = 0x0,
};

/* B.4.1  Standard AS Interface Descriptor */
static struct usb_interface_descriptor std_as_out_if0_desc = {
	.bLength = sizeof std_as_out_if0_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
};

static struct usb_interface_descriptor std_as_out_if1_desc = {
	.bLength = sizeof std_as_out_if1_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
};

/* B.4.2  Class-Specific AS Interface Descriptor */
static struct uac2_as_header_descriptor as_out_hdr_desc = {
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
static struct uac2_format_type_i_descriptor as_out_fmt1_desc = {
        .bLength = sizeof as_out_fmt1_desc,
        .bDescriptorType = USB_DT_CS_INTERFACE,
        .bDescriptorSubtype = UAC_FORMAT_TYPE,
        .bFormatType = UAC_FORMAT_TYPE_I,
};

/* Standard ISO OUT Endpoint Descriptor */

static struct usb_endpoint_descriptor fs_epout_desc  = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT,

	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE,
        //.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,

	.wMaxPacketSize = cpu_to_le16(512),
	.bInterval = 2,
};

static struct usb_endpoint_descriptor hs_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE,
        //.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,

	.wMaxPacketSize = cpu_to_le16(512),
	.bInterval = 2,
};

DECLARE_UAC_FEATURE_UNIT_DESCRIPTOR(0);
/*
#define INPUT_TERMINAL_ID	1
static struct uac_input_terminal_descriptor input_terminal_desc = {
		.bLength =		sizeof input_terminal_desc,//UAC_DT_INPUT_TERMINAL_SIZE,
		.bDescriptorType =	USB_DT_CS_INTERFACE,
		.bDescriptorSubtype =	UAC_INPUT_TERMINAL,
		.bTerminalID =		INPUT_TERMINAL_ID,
		.wTerminalType =	UAC_TERMINAL_STREAMING,
		.bAssocTerminal =	0,
		.wChannelConfig =	0x3,//0x3,
};

#define FEATURE_UNIT_ID		2
static struct uac_feature_unit_descriptor_0 feature_unit_desc = {
		.bLength		= sizeof feature_unit_desc,//UAC_DT_FEATURE_UNIT_SIZE(0),
		.bDescriptorType	= USB_DT_CS_INTERFACE,
		.bDescriptorSubtype	= UAC_FEATURE_UNIT,
		.bUnitID		= FEATURE_UNIT_ID,
		.bSourceID		= INPUT_TERMINAL_ID,
		.bControlSize		= 2,
		.bmaControls[0]		= (UAC_FU_MUTE | UAC_FU_VOLUME),
};

#define OUTPUT_TERMINAL_ID	3
static struct uac1_output_terminal_descriptor output_terminal_desc = {
        .bLength		= sizeof output_terminal_desc,//UAC_DT_OUTPUT_TERMINAL_SIZE,
        .bDescriptorType	= USB_DT_CS_INTERFACE,
        .bDescriptorSubtype	= UAC_OUTPUT_TERMINAL,
        .bTerminalID		= OUTPUT_TERMINAL_ID,
        .wTerminalType		= UAC_OUTPUT_TERMINAL_SPEAKER,
        .bAssocTerminal		= FEATURE_UNIT_ID,
        .bSourceID		= FEATURE_UNIT_ID,
};

*/

DECLARE_UAC_FORMAT_TYPE_I_DISCRETE_DESC(7);

static struct uac_format_type_i_discrete_descriptor_7 as_type_i_desc = {
        .bLength =		sizeof as_type_i_desc,//UAC_FORMAT_TYPE_I_DISCRETE_DESC_SIZE(1),
        .bDescriptorType =	USB_DT_CS_INTERFACE,
        .bDescriptorSubtype =	UAC_FORMAT_TYPE,
        .bFormatType =		UAC_FORMAT_TYPE_I,
        .bSubframeSize =	4,//2,//4,
        .bBitResolution =	24,//16,//24,
        .bSamFreqType =		7,
};
/* Standard ISO OUT Endpoint Descriptor */
/*
static struct usb_endpoint_descriptor as_out_ep_desc  = {
        .bLength =		sizeof as_out_ep_desc,//USB_DT_ENDPOINT_AUDIO_SIZE,
        .bDescriptorType =	USB_DT_ENDPOINT,
        .bEndpointAddress =	USB_DIR_OUT,
        .bmAttributes =		USB_ENDPOINT_SYNC_ADAPTIVE
                               | USB_ENDPOINT_XFER_ISOC,
        .wMaxPacketSize =	__constant_cpu_to_le16(OUT_EP_MAX_PACKET_SIZE),
        .bInterval =		2,//4,
};*/
/* Class-specific AS ISO OUT Endpoint Descriptor */
static struct uac2_iso_endpoint_descriptor as_iso_out_desc = {
	.bLength = sizeof as_iso_out_desc,
	.bDescriptorType = USB_DT_CS_ENDPOINT,

	.bDescriptorSubtype = UAC_EP_GENERAL,
	.bmAttributes = 0,
	.bmControls = 0,
	.bLockDelayUnits = 2,
	.wLockDelay =8,
};

static struct usb_descriptor_header *fs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

   // (struct usb_descriptor_header *)&input_terminal_desc,
   // (struct usb_descriptor_header *)&output_terminal_desc,
   // (struct usb_descriptor_header *)&feature_unit_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&out_clk_src_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&fs_epout_desc,
    (struct usb_descriptor_header *)&as_type_i_desc,
    //(struct usb_descriptor_header *)&as_out_ep_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

    //(struct usb_descriptor_header *)&input_terminal_desc,
    //(struct usb_descriptor_header *)&output_terminal_desc,
    //(struct usb_descriptor_header *)&feature_unit_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&out_clk_src_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&hs_epout_desc,
    (struct usb_descriptor_header *)&as_type_i_desc,
    //(struct usb_descriptor_header *)&as_out_ep_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,
	NULL,
};

/*
 * This function is an ALSA sound card following USB Audio Class Spec 1.0.
 */
bool uac_enable_flag(void)
{
        return uac2_enable_flag;
}
EXPORT_SYMBOL_GPL(uac_enable_flag);

/*-------------------------------------------------------------------------*/
struct f_audio_buf {
	u8 *buf;
	int actual;
	struct list_head list;
};

static struct f_audio_buf *f_audio_buffer_alloc(int buf_size)
{
	struct f_audio_buf *copy_buf;

	copy_buf = kzalloc(sizeof *copy_buf, GFP_ATOMIC);
	if (!copy_buf)
		return ERR_PTR(-ENOMEM);

	copy_buf->buf = kzalloc(buf_size, GFP_ATOMIC);
	if (!copy_buf->buf) {
		kfree(copy_buf);
		return ERR_PTR(-ENOMEM);
	}

	return copy_buf;
}

static void f_audio_buffer_free(struct f_audio_buf *audio_buf)
{
	if(audio_buf)
	{
		kfree(audio_buf->buf);
		kfree(audio_buf);
	}
}
/*-------------------------------------------------------------------------*/

struct f_audio {
	struct gaudio			card;
		
	
	/* endpoints handle full and/or high speeds */
	struct usb_ep			*out_ep;
	struct usb_endpoint_descriptor	*out_desc;

	spinlock_t			lock;
	struct f_audio_buf *copy_buf;
	struct work_struct playback_work;
	struct list_head play_queue;

	//***********cyadd***********
	atomic_t			online;
		u8 ac_intf, ac_alt;
	u8 as_out_intf, as_out_alt;
	u8 as_in_intf, as_in_alt;

	struct mutex                    uac2_mutex;
    struct delayed_work             close_work;
};

struct cntrl_cur_lay3 {
	__u32	dCUR;
};

struct cntrl_rang_sub {
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

static inline struct f_audio *func_to_audio(struct usb_function *f)
{
	return container_of(f, struct f_audio, card.func);
}

/*-------------------------------------------------------------------------*/

static void f_audio_playback_work(struct work_struct *data)
{
	struct f_audio *audio = container_of(data, struct f_audio,
					playback_work);
	struct f_audio_buf *play_buf;
	int i = 0, j = 0;
	int count = 0;
	unsigned char* tempbuf = NULL;

	if (!atomic_read(&audio->online)) {
		printk("%s offline\n", __func__);
		return;
	}
	
	spin_lock_irq(&audio->lock);
	if (list_empty(&audio->play_queue)) {
		spin_unlock_irq(&audio->lock);
		return;
	}
	play_buf = list_first_entry(&audio->play_queue,
			struct f_audio_buf, list);
	list_del(&play_buf->list);
	spin_unlock_irq(&audio->lock);
/*
#if 0
	count = play_buf->actual;
	tempbuf = kzalloc(count, GFP_ATOMIC);
	memset(tempbuf, 0, count);
	for(i =0,j=0;i<count;i++)
	{
		if((i % 4 == 2) ||(i % 4 == 3))
		{
			tempbuf[j++] = (unsigned char)play_buf->buf[i];
		}
	}
	count = count/2;
//	if(!(__gpio_get_pin(DAC_MUTE)))		__gpio_set_pin(DAC_MUTE);
	u_audio_playback(&audio->card, tempbuf, count);
	
	memset(tempbuf, 0, count);
	if(tempbuf)	kfree(tempbuf);
	tempbuf = NULL;
#else
 */
	u_audio_playback(&audio->card, play_buf->buf, play_buf->actual);
//#endif
	f_audio_buffer_free(play_buf);
}

static int f_audio_out_ep_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_audio *audio = req->context;
	struct usb_composite_dev *cdev = audio->card.func.config->cdev;
	struct f_audio_buf *copy_buf = audio->copy_buf;
	int err;
	if (!copy_buf)
		return -EINVAL;

	/* Copy buffer is full, add it to the play_queue */
        if (!first_copy_audio_buffer) {
	    if (audio_buf_size - copy_buf->actual < req->actual) {
		spin_lock_irq(&audio->lock);
		if (!list_empty(&audio->play_queue)) {
			INFO(cdev, "over-runs, audio write slow.. drop the packet\n");
			f_audio_buffer_free(copy_buf);
		} else {
			list_add_tail(&copy_buf->list, &audio->play_queue);
		}
		spin_unlock_irq(&audio->lock);

		schedule_work(&audio->playback_work);
		copy_buf = f_audio_buffer_alloc(audio_buf_size);
		if (IS_ERR(copy_buf))
			return -ENOMEM;
	    }
        } else {
	    if (first_audio_buf_size - copy_buf->actual < req->actual) {
		spin_lock_irq(&audio->lock);
		if (!list_empty(&audio->play_queue)) {
			INFO(cdev, "over-runs, audio write slow.. drop the packet\n");
			f_audio_buffer_free(copy_buf);
		} else {
			list_add_tail(&copy_buf->list, &audio->play_queue);
		}
		spin_unlock_irq(&audio->lock);

		schedule_work(&audio->playback_work);
		copy_buf = f_audio_buffer_alloc(audio_buf_size);
		if (IS_ERR(copy_buf))
			return -ENOMEM;

                first_copy_audio_buffer = false;
	    }
        }

	memcpy(copy_buf->buf + copy_buf->actual, req->buf, req->actual);
	copy_buf->actual += req->actual;
	audio->copy_buf = copy_buf;

	err = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (err)
		ERROR(cdev, "%s queue req: %d\n", ep->name, err);

	return 0;
}

//#define STEP    1
static void reset_uac_rate(int freq)
{
    int rate;

    rate = freq;
    switch(rate) {
        case UAC_RATE_VALUE_32000:
            actual_rate = UAC_RATE_32000;
            break;
        case UAC_RATE_VALUE_44100:
            actual_rate = UAC_RATE_44100;
            break;
        case UAC_RATE_VALUE_48000:
            actual_rate = UAC_RATE_48000;
            break;    
        case UAC_RATE_VALUE_88200:
            actual_rate = UAC_RATE_88200;
            break;
        case UAC_RATE_VALUE_96000:
            actual_rate = UAC_RATE_96000;
            break; 
        case UAC_RATE_VALUE_176400:
            actual_rate = UAC_RATE_176400;
            break; 
        case UAC_RATE_VALUE_192000:
            actual_rate = UAC_RATE_192000;
            break;
        default :
            actual_rate = UAC_DEFAULT_RATE;
            break;            
    }
}

static void f_audio_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_audio *audio = req->context;
	int status = req->status;
	u32 data = 0;
	struct usb_ep *out_ep = audio->out_ep;
	struct usb_composite_dev *cdev = audio->card.func.config->cdev;
	 int actual_value;
	 
	 if (dug_uac_flags) {
            actual_value = req->actual;
            dug_uac_flags = false;
            printk("%s: actual_value is  %d \n", __func__, actual_value);

            reset_uac_rate(actual_value);
        }
	
	switch (status) {
	case 0:				/* normal completion? */
		if (ep == out_ep)
			f_audio_out_ep_complete(ep, req);
		break;
	 /* fall through to to free buffer and req */
	case -ECONNRESET:
	case -ESHUTDOWN:
		kfree(req->buf);
		usb_ep_free_request(ep, req);
		break;
	default:
		break;
	}
}

static int
in_rq_cur(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request *req = f->config->cdev->req;
	struct f_audio *audio = func_to_audio(f);
	u16 w_length = le16_to_cpu(ctrl->wLength);
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u8 entity_id = (w_index >> 8) & 0xff;
	u8 control_selector = w_value >> 8;
	int value = -EOPNOTSUPP;

        DBG(cdev, "%s: enter !!!\n", __func__);

	if (control_selector == UAC2_CS_CONTROL_SAM_FREQ) {
		struct cntrl_cur_lay3 c;

		c.dCUR = c_srate;

		value = min_t(unsigned, w_length, sizeof c);
		memcpy(req->buf, &c, value);
	} else if (control_selector == UAC2_CS_CONTROL_CLOCK_VALID) {
		*(u8 *)req->buf = 1;
		value = min_t(unsigned, w_length, 1);
	} else {
		ERROR(cdev,
			"%s:%d control_selector=%d TODO!\n",
			__func__, __LINE__, control_selector);
	}

	return value;
}

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
in_rq_range(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request *req = f->config->cdev->req;
	struct f_audio *audio = func_to_audio(f);
	u16 w_length = le16_to_cpu(ctrl->wLength);
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u8 entity_id = (w_index >> 8) & 0xff;
	u8 control_selector = w_value >> 8;
	int value = -EOPNOTSUPP;

	if (control_selector == UAC2_CS_CONTROL_SAM_FREQ) {
		cntrl_r.wNumSubRanges = 7;
                
                set_support_sample_rates();

		value = min_t(unsigned, w_length, sizeof cntrl_r);
		memcpy(req->buf, &cntrl_r, value);
	} else {
		ERROR(cdev,
			"%s:%d control_selector=%d TODO!\n",
			__func__, __LINE__, control_selector);
	}

	return value;
}

static int
ac_rq_in(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;

	if (ctrl->bRequest == UAC2_CS_CUR)
		return in_rq_cur(f, ctrl);
	else if (ctrl->bRequest == UAC2_CS_RANGE)
		return in_rq_range(f, ctrl);
	else
		return -EOPNOTSUPP;
}

static int
out_rq_cur(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	u16 w_length = le16_to_cpu(ctrl->wLength);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u8 control_selector = w_value >> 8;
	struct usb_composite_dev *cdev = f->config->cdev;

	if (control_selector == UAC2_CS_CONTROL_SAM_FREQ)
		return w_length;

	return -EOPNOTSUPP;
}

static int
setup_rq_inf(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct f_audio *audio = func_to_audio(f);
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u8 intf = w_index & 0xff;

	if (intf != audio->ac_intf) {
		ERROR(cdev,
			"%s:%d Error!\n", __func__, __LINE__);
		return -EOPNOTSUPP;
	}

	if (ctrl->bRequestType & USB_DIR_IN)
		return ac_rq_in(f, ctrl);
	else if (ctrl->bRequest == UAC2_CS_CUR)
		return out_rq_cur(f, ctrl);

	return -EOPNOTSUPP;
}



static int
f_audio_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	
	/* Only Class specific requests are supposed to reach here */
	if ((ctrl->bRequestType & USB_TYPE_MASK) != USB_TYPE_CLASS)
		return -EOPNOTSUPP;

	if ((ctrl->bRequestType & USB_RECIP_MASK) == USB_RECIP_INTERFACE)
		value = setup_rq_inf(f, ctrl);
	else
		ERROR(cdev, "%s:%d Error!\n", __func__, __LINE__);

	/* respond with data transfer or status phase? */
	if (value >= 0) {

		req->zero = value < w_length;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "audio response on err %d\n", value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static int f_audio_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_audio		*audio = func_to_audio(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_ep *out_ep = audio->out_ep;
	struct usb_request *req;
	//struct usb_request *req2;
	int i = 0, err = 0;

	if (cdev->gadget->speed == USB_SPEED_FULL) {
        audio->out_desc = &fs_epout_desc;
        //audio->out_ep->desc = &fs_epout_desc;
	} else {
        audio->out_desc = &hs_epout_desc;
	 //audio->out_ep->desc = &hs_epout_desc;
	}
	atomic_set(&audio->online, 1);
	printk("intf %d, alt %d\n", intf, alt);
	
	if (intf == audio->as_out_intf) {
		if (audio->as_out_alt == alt) {
			INFO(cdev, "Alt interface is already set to %d. Do nothing.\n",
				alt);

			return 0;
		}
		if (alt == 1) {
			dug_uac_flags = true;
//			__i2s_enable();
			//usb_ep_enable(out_ep, audio->out_desc);
            out_ep->desc=audio->out_desc;
            usb_ep_enable(out_ep);
			out_ep->driver_data = audio;
			audio->copy_buf = f_audio_buffer_alloc(first_audio_buf_size);
			if (IS_ERR(audio->copy_buf))
				return -ENOMEM;
			/*
			 * allocate a bunch of read buffers
			 * and queue them all at once.
			 */
			for (i = 0; i < req_count && err == 0; i++) {
				req = usb_ep_alloc_request(out_ep, GFP_ATOMIC);
				if (req) {
					req->buf = kzalloc(req_buf_size,
							GFP_ATOMIC);
					if (req->buf) {
						req->length = req_buf_size;
						req->context = audio;
						req->complete =
							f_audio_complete;
						err = usb_ep_queue(out_ep,
							req, GFP_ATOMIC);
						if (err) {
							ERROR(cdev,
							"%s queue req: %d\n",
							out_ep->name, err);
						}	
					} else
						err = -ENOMEM;
				} else 
					err = -ENOMEM;
			}
		} else {
			struct f_audio_buf *copy_buf = audio->copy_buf;
			//if(__gpio_get_pin(DAC_MUTE))
			//	__gpio_clear_pin(DAC_MUTE);
			//mdelay(50);
/*			__i2s_disable_replay();
			__i2s_disable_transmit_dma();
			__i2s_disable();
			mdelay(10);
			__aic_write_tfifo(0x0);
			__aic_write_tfifo(0x0);
			mdelay(400);
*/			if (copy_buf) {			
			list_add_tail(&copy_buf->list,&audio->play_queue);
				schedule_work(&audio->playback_work);
				//memset(audio->copy_buf, 0, sizeof(audio->copy_buf));
                //audio->copy_buf = NULL;
			}
			else
			{	
				printk("playback_buf is empty. Stop.");
			}
			 first_copy_audio_buffer = true;
		}
		//audio->as_out_alt = alt;
	}
	else
	{
			printk("Interface %d. Do nothing. Return %d\n", intf, err);
	}

	return err;
}

static int
uac2_get_alt(struct usb_function *f, unsigned intf)
{
	struct f_audio *audio = func_to_audio(f);
	struct usb_composite_dev *cdev = f->config->cdev;

        DBG(cdev, "%s: intf=%d\n", __func__, intf);

	if (intf == audio->ac_intf)
		return audio->ac_alt;
	else if (intf == audio->as_out_intf)
		return audio->as_out_alt;
	else if (intf == audio->as_in_intf)
		return audio->as_in_alt;
	else
		ERROR(cdev,
			"%s:%d Invalid Interface %d!\n",
			__func__, __LINE__, intf);

	return -EINVAL;
}

static void uac2_disable_reset(void)
{
        actual_rate = UAC_RATE_44100;//UAC_RATE_192000;//
}

static void f_audio_close_work(struct work_struct *data)
{
    struct f_audio *audio = container_of(data, struct f_audio, close_work);

    pr_info("close audio files\n");
    mutex_lock(&audio->uac2_mutex);
    gaudio_cleanup();
    mutex_unlock(&audio->uac2_mutex);
/*
	if (audio_task != NULL)
	{
		kthread_stop(audio_task);
                audio_task = NULL;
	}
*/
    kfree(audio);
}

static void f_audio_disable(struct usb_function *f)
{
	struct f_audio	*audio = func_to_audio(f);
	struct usb_composite_dev *cdev = audio->card.func.config->cdev;

	INFO(cdev, "%s:  enter  !!!\n", __FUNCTION__);
#if 0
	if (uac2_diable_flag)
		return;

	uac2_diable_flag = true;
	uac2_enable_flag = false;
#endif
	audio->as_in_alt = 0;
	audio->as_out_alt = 0;
	atomic_set(&audio->online, 0);

	usb_ep_disable(audio->out_ep);

	//actual_rate = UAC_RATE_44100;

	flush_work(&audio->playback_work);		
}

/*-------------------------------------------------------------------------*/

static void f_audio_build_desc(struct f_audio *audio)
{
    struct gaudio *card = &audio->card;
    u8 *sam_freq;
    int rate;

    /* Set channel numbers */
   // input_terminal_desc.bNrChannels = u_audio_get_playback_channels(card);
    as_type_i_desc.bNrChannels = u_audio_get_playback_channels(card);

    rate = u_audio_get_playback_rate(card);
    sam_freq = as_type_i_desc.tSamFreq[0];
    memcpy(sam_freq, &rate, 3);
    /* Set sample rates */
    /*
    rate = UAC_RATE_32000;
    sam_freq = as_type_i_desc.tSamFreq[0];
    memcpy(sam_freq, &rate, 3);

    rate = u_audio_get_playback_rate(card);
    sam_freq = as_type_i_desc.tSamFreq[1];
    memcpy(sam_freq, &rate, 3);

    rate = UAC_RATE_48000;
    sam_freq = as_type_i_desc.tSamFreq[2];
    memcpy(sam_freq, &rate, 3);

    rate = UAC_RATE_88200;
    sam_freq = as_type_i_desc.tSamFreq[3];
    memcpy(sam_freq, &rate, 3);

    rate = UAC_RATE_96000;
    sam_freq = as_type_i_desc.tSamFreq[4];
    memcpy(sam_freq, &rate, 3);

    rate = UAC_RATE_176400;
    sam_freq = as_type_i_desc.tSamFreq[5];
    memcpy(sam_freq, &rate, 3);

    rate = UAC_RATE_192000;
    sam_freq = as_type_i_desc.tSamFreq[6];
    memcpy(sam_freq, &rate, 3);
*/
    /* Todo: Set Sample bits and other parameters */

    return;
}

/* audio function driver setup/binding */
static int __init
f_audio_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_audio		*audio = func_to_audio(f);
	int			ret,status;
	struct usb_ep		*ep;

	/* set up ASLA audio devices */
	status = gaudio_setup(&audio->card);
	if (status < 0)
	    goto fail;

    f_audio_build_desc(audio);

	/* allocate instance-specific interface IDs, and patch descriptors */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	
	std_ac_if_desc.bInterfaceNumber = status;
	audio->ac_intf = status;
	audio->ac_alt = 0;

	status = usb_interface_id(c, f);
	if (status < 0)
	{
		printk("%s:%d Error!\n",__func__,__LINE__);
		return status;
	}
	std_as_out_if0_desc.bInterfaceNumber = status;
	std_as_out_if1_desc.bInterfaceNumber = status;
	audio->as_out_intf = status;
	audio->as_out_alt = 0;

	status = -ENODEV;


	/* allocate instance-specific endpoints */
	ep  = usb_ep_autoconfig(cdev->gadget, &fs_epout_desc);
	if (!ep )
		goto fail;
	
	audio->out_ep = ep;
	ep->driver_data = cdev;	/* claim */

	status = -ENOMEM;
	
	hs_epout_desc.bEndpointAddress = fs_epout_desc.bEndpointAddress;
	hs_epout_desc.wMaxPacketSize = fs_epout_desc.wMaxPacketSize;

	/* supcard all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */

	/* copy descriptors, and track endpoint copies */
/*
    f->descriptors = usb_copy_descriptors(fs_audio_desc);

	if (gadget_is_dualspeed(c->cdev->gadget)) {
		c->highspeed = true;
		f->hs_descriptors = usb_copy_descriptors(hs_audio_desc);
	}*/
    ret = usb_assign_descriptors(f, fs_audio_desc, hs_audio_desc, NULL);
    if (ret)
        return ret;
	return 0;

fail:
	/*if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);*/
    usb_free_all_descriptors(f);

        if (uac2_enable_flag)
            uac2_enable_flag = false;
	gaudio_cleanup();
	if (ep)
		ep->driver_data = NULL;
	return status;
}

static void
f_audio_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_audio		*audio = func_to_audio(f);

	printk("====into %s\n", __FUNCTION__);

	if(g_kobj) {
		kobject_put(g_kobj);
		g_kobj = NULL;
	}
		
	if(uac2_task){
		kthread_stop(uac2_task);
		uac2_task = NULL;
	}
	
	if (!uac2_diable_flag) {
		uac2_disable_reset();
	}
	
	flush_work(&audio->playback_work);
	/*if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->descriptors);
	usb_free_descriptors(f->hs_descriptors);*/
    usb_free_all_descriptors(f);
}

/*-------------------------------------------------------------------------*/

/**
 * audio_bind_config - add USB audio function to a configuration
 * @c: the configuration to supcard the USB audio function
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 */
int __init audio_bind_config(struct usb_configuration *c)
{
	int status;
	int res;
	/* allocate and initialize one new instance */
	audio_for_usb = kzalloc(sizeof *audio_for_usb, GFP_KERNEL);
	if (!audio_for_usb)
		return -ENOMEM;

	 uac2_diable_flag = false;
	res = usb_string_ids_tab(c->cdev, strings_dev);
	if (res)
		return res;
	

	std_ac_if_desc.iInterface = strings_dev[1].id;
	out_clk_src_desc.iClockSource = strings_dev[2].id;
	usb_out_it_desc.iTerminal = strings_dev[3].id;
	io_out_ot_desc.iTerminal = strings_dev[4].id;
	std_as_out_if0_desc.iInterface = strings_dev[5].id;
	std_as_out_if1_desc.iInterface = strings_dev[6].id;

	INIT_LIST_HEAD(&audio_for_usb->play_queue);
	spin_lock_init(&audio_for_usb->lock);
    mutex_init(&audio_for_usb->uac2_mutex);

	audio_for_usb->card.func.name = "g_audio";
	audio_for_usb->card.gadget = c->cdev->gadget;
	audio_for_usb->card.func.strings = audio_strings;
	audio_for_usb->card.func.bind = f_audio_bind;
	audio_for_usb->card.func.unbind = f_audio_unbind;
	audio_for_usb->card.func.set_alt = f_audio_set_alt;
	audio_for_usb->card.func.get_alt = uac2_get_alt;
	audio_for_usb->card.func.setup = f_audio_setup;
	audio_for_usb->card.func.disable = f_audio_disable;
    //audio_for_usb->out_desc = &as_out_ep_desc;

	/* Initialize the configurable parameters */
	usb_out_it_desc.bNrChannels = num_channels(c_chmask);
	usb_out_it_desc.bmChannelConfig = cpu_to_le32(0x0);
	as_out_hdr_desc.bNrChannels = num_channels(c_chmask);
	as_out_hdr_desc.bmChannelConfig = cpu_to_le32(0x0);
	as_out_fmt1_desc.bSubslotSize = c_ssize*2;             //*2
	as_out_fmt1_desc.bBitResolution = 32;//c_ssize * 8 * 2; //24


	INIT_WORK(&audio_for_usb->playback_work, f_audio_playback_work);
    INIT_DELAYED_WORK(&audio_for_usb->close_work, f_audio_close_work);

	status = usb_add_function(c, &audio_for_usb->card.func);
	if (status)
		goto add_fail;

	g_kobj = kobject_create_and_add("for_usb_dac", kernel_kobj);
	if (!g_kobj)
		return -ENOMEM;
	status = sysfs_create_group(g_kobj, &g_audio_attr_group);
	if(status)
		kobject_put(g_kobj);

	f_uac2_init();

	INFO(c->cdev, "audio_buf_size %d, req_buf_size %d, req_count %d\n",
		audio_buf_size, req_buf_size, req_count);
	uac2_enable_flag = true;
	return status;

add_fail:
	gaudio_cleanup();
setup_fail:
	kfree(audio_for_usb);
	return status;
}

static int uac2_task_thread(void *arg)
{
	while(1)
	{
		set_current_state(TASK_UNINTERRUPTIBLE);
		if(kthread_should_stop())	break;
		if(g_audio) {
			printk("===get the value: %u\n", g_audio);
			break;
		}
		schedule_timeout(HZ);
	}

	return 0;
}

static int f_uac2_init(void)
{
	int status;

	uac2_task = kthread_create(uac2_task_thread, NULL, "f_uac2_task");
	if(IS_ERR(uac2_task)) {
		status = PTR_ERR(uac2_task);
		uac2_task = NULL;
		printk("unable to create kernel thread\n");
		return status ;	
	} 
	
	wake_up_process(uac2_task);

	return 0;
}

