#ifndef __INTERFACE_H__
#define __INTERFACE_H__

enum open_mode {
	EARLY_SLEEP = 1,
	DEEP_SLEEP,
	NORMAL_RECORD,
	NORMAL_WAKEUP
};


#define DMIC_IOCTL_SET_SAMPLERATE	0x200
#define DMIC_IOCTL_SET_CHANNEL		0x201


/* same define as kernel */
#define		SLEEP_BUFFER_SIZE	(32 * 1024)
#define		NR_BUFFERS			(8)

struct sleep_buffer {
	unsigned char *buffer[NR_BUFFERS];
	unsigned int nr_buffers;
	unsigned long total_len;
};

#define LOAD_ADDR	0x81f00000 /* 31M */
#define LOAD_SIZE	(256 * 1024)

extern int get_voice_wakeup_state();
extern int set_wakeup_source_type(int type);



#endif
