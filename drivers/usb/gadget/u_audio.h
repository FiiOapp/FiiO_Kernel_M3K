/*
 * u_audio.h -- interface to USB gadget "ALSA AUDIO" utilities
 *
 * Copyright (C) 2008 Bryan Wu <cooloney@kernel.org>
 * Copyright (C) 2008 Analog Devices, Inc
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __U_AUDIO_H
#define __U_AUDIO_H

#include <linux/device.h>
#include <linux/err.h>
#include <linux/usb/audio.h>
#include <linux/usb/audio-v2.h>
#include <linux/usb/composite.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "gadget_chips.h"

#define UAC_RATE_32000     32000
#define UAC_RATE_44100     44100
#define UAC_RATE_48000     48000
#define UAC_RATE_88200     88200
#define UAC_RATE_96000     96000
#define UAC_RATE_176400    176400
#define UAC_RATE_192000    192000

#define UAC_DEFAULT_RATE   44100//192000//

#if 0
#define UAC_RATE_VALUE_32000     64//32
#define UAC_RATE_VALUE_44100     88//44//20
#define UAC_RATE_VALUE_48000     96//48//24
#define UAC_RATE_VALUE_88200     176//88//44
#define UAC_RATE_VALUE_96000     192//96//48
#define UAC_RATE_VALUE_176400    352//176//88
#define UAC_RATE_VALUE_192000    384//192//96
#else
#define UAC_RATE_VALUE_32000     32
#define UAC_RATE_VALUE_44100     44
#define UAC_RATE_VALUE_48000     48
#define UAC_RATE_VALUE_88200     88
#define UAC_RATE_VALUE_96000     96
#define UAC_RATE_VALUE_176400    176
#define UAC_RATE_VALUE_192000    192
#endif

/*
 * This represents the USB side of an audio card device, managed by a USB
 * function which provides control and stream interfaces.
 */

struct gaudio_snd_dev {
    struct gaudio			*card;
    struct file			*filp;
    struct snd_pcm_substream	*substream;
    int				access;
    int				format;
    int				channels;
    int				rate;
};

struct gaudio {
    struct usb_function		func;
    struct usb_gadget		*gadget;

    /* ALSA sound device interfaces */
    struct gaudio_snd_dev		control;
    struct gaudio_snd_dev		playback;
    struct gaudio_snd_dev		capture;

    /* TODO */
};

int gaudio_setup(struct gaudio *card);
void gaudio_cleanup(void);

#endif /* __U_AUDIO_H */
