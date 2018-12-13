#ifndef __WAKEUP_INTERFACE_H__
#define __WAKEUP_INTERFACE_H__

enum INTERFACE_CMD {
	ICMD_INIT=1,
	ICMD_EXIT,
	ICMD_WAKEUP_HANDLER,
	ICMD_OPEN,
	ICMD_CLOSE,
	ICMD_ENTER_SUSPEND,
	ICMD_EXIT_SUSPEND,
	ICMD_SETUP_VOICE_TRIGGER,
	ICMD_CACHE_PREFETCH,
	ICMD_GET_DMA_ADDRESS,
	ICMD_IOCTL,
	ICMD_GET_RESOURCE_ADDR,
	ICMD_PROCESS_DATA,
	ICMD_IS_CPU_WAKEUP_BY_DMIC,
	ICMD_SET_SLEEP_BUFFER,
	ICMD_GET_SLEEP_PROCESS,
	ICMD_SET_DMA_CHANNEL,
	ICMD_WAKEUP_ENABLE,
	ICMD_IS_WAKEUP_ENABLED,
	ICMD_SET_CALLBACK_HANDLER,
};

/*  */
struct common_args {
	int ret_val;
	long value;		/* value */
};


#define DEFAULT_RESERVE_MEMORY_SIZE (128*1024) /* 128KB */

struct init_args {
	int ret_val;
	int uart_id;		/* dma channel */
	int dma_id;		/* dma channel */
	int tcu_id;		/* tcu channel */
	int tcu_int_val;	/* tcu timer period second */
	int rtc_int_val;	/* rtc timer period second */
	char * mem;		/* reserve memory for recogonition, default size 128KB */
	int mem_size;
};

struct open_args {
	int ret_val;
	int mode;		/* mode */
};


struct ioctl_args {
	int ret_val;
	int cmd;		/* ioctl cmd */
	unsigned long args;
};


struct fw_shared_param
{
	unsigned int  pm_core_enter;
	unsigned char pmu_i2c_scl;           //default 0xff
	unsigned char pmu_i2c_sda;           //default 0xff
	unsigned char pmu_addr;               //default 0xff
	unsigned char pmu_reg;                //default 0xff
	unsigned char pmu_register_val;

	unsigned char pmu_pin;               //default 0xff
	unsigned char pmu_pin_func;          //default 0xff
	unsigned char uart_id;          //default 0xff

	unsigned int  prev_resume_pc;  //ddr is self-reflash default 0xffffffff
	unsigned int  post_resume_pc;  //ddr is ok. default 0xffffffff
	unsigned int  prev_sleep_pc;   //after flush cache. default 0xffffffff
	unsigned int  post_sleep_pc;   //before wait. default 0xffffffff
};

#endif	/* __WAKEUP_INTERFACE_H__ */
