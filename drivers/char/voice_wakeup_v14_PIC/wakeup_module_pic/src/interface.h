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


int open(int mode);
int close(int mode);
int enter_suspend(int mode);
int exit_suspend(int mode);
int setup_voice_trigger(int mode);

int wakeup_handler(int par);
int set_callback_handler(void *handler);
unsigned int get_dma_address(void);
int ioctl(int cmd, unsigned long args);
void cache_prefetch(void);
unsigned char * get_resource_addr(void);
int process_data(void);
int is_cpu_wakeup_by_dmic(void);
int set_sleep_buffer(struct sleep_buffer *sleep_buffer);
int get_sleep_process(void);
int set_dma_channel(int channel);
int voice_wakeup_enable(int enable);
int is_voice_wakeup_enabled(void);

int module_init(void);


#endif
