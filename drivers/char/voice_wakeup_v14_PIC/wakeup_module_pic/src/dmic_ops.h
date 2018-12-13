#ifndef __DMIC_OPS_H_
#define __DMIC_OPS_H_

#include "common.h"
#include "jz_dmic.h"
#include "jz_dma.h"

enum dmic_status {
	WAITING_TRIGGER = 1,
	WAITING_DATA,
};

/* extern int dmic_current_state; */
/* extern unsigned int cur_thr_value; */
/* extern unsigned int wakeup_failed_times; */

void dump_dmic_regs(void);


extern int dmic_config(void);
//extern void dmic_init(void);
extern int dmic_enable(void);
extern int dmic_disable(void);

extern int dmic_handler(int);
extern int dmic_handler_cpu_mode(int);

extern void reconfig_thr_value(void);

extern int dmic_init_deep_sleep(void);

extern int dmic_init_record(void);

extern int dmic_init_normal_mode(int mode);
extern int dmic_close(int mode);


extern int dmic_ioctl(int, unsigned long);

extern int cpu_should_sleep(void);

extern int dmic_disable_tri(void);

extern int dmic_store_data_from_fifo_to_memory(char * buffer, int size);

extern int dmic_get_voice_data(unsigned char ** buf, int length);
extern int dmic_check_and_receive_data();
extern int reset_dmic_trigger_irq_arrival_state(void);
extern int is_dmic_trigger_irq_arrival(void);
extern int is_dmic_waiting_data(void);

extern int dmic_module_suspend_enter(int use_dma);
extern int dmic_module_suspend_exit(int use_dma);
extern int dmic_module_reset_voice_trigger(int dma);


#endif
