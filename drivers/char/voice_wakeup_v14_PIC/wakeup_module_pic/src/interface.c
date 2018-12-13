/*
 * main.c
 */
#include <common.h>
#include <jz_cpm.h>

#include "ivDefine.h"
#include "ivIvwDefine.h"
#include "ivIvwErrorCode.h"
#include "ivIVW.h"
#include "ivPlatform.h"

#include "recognization.h"
#include "dma_ops.h"
#include "dmic_ops.h"
#include "jz_dmic.h"
#include "rtc_ops.h"
#include "cpm_ops.h"
#include "dmic_config.h"
#include "jz_dma.h"
#include "tcu_timer.h"
#include "wakeup_handler.h"
#include "irq.h"
#include "print.h"
#include "mem_buffer.h"
#include "string.h"
#include "interface.h"

/* #define INTERFACE_VOICE_DEBUG */
#define TAG	"[voice_wakeup]"

/* global config: default value */

static unsigned int cpu_wakeup_by = 0;
static unsigned int open_cnt = 0;
static struct sleep_buffer *g_sleep_buffer;
static int voice_wakeup_enabled = 0;
static int dmic_record_enabled = 0;
static int g_dma_mode = 0;
static int g_dmic_current_working_mode = 0;




#ifdef INTERFACE_VOICE_DEBUG
void dump_voice_wakeup(void)
{
	printk("###########dump voice wakeup status#######\n");
	printk("cpu_wakeup_by:			%d\n", cpu_wakeup_by);
	printk("open_cnt:				%d\n", open_cnt);
	printk("voice_wakeup_enabled:	%d\n", voice_wakeup_enabled);
	//printk("dmic_record_enabled:	%d\n", dmic_record_enabled);

	//printk("wakeup_failed_times:	%d\n", wakeup_failed_times);
	//printk("dmic_current_state:	%d\n", dmic_current_state);

	printk("###########dump voice wakeup status#######\n");

}
#endif

int open(int mode)
{
	if ( g_dmic_current_working_mode == mode ) {
		/* skip return for sleep again in pm_enter() */
		vtw_print(LOG_DEBUG, "open() g_dmic_current_working_mode == mode\r\n");
		return 0;
	}
	else {
	//if ( g_dmic_current_working_mode != mode) {
		open_cnt++;
		g_dmic_current_working_mode = mode;
	}

	/* init memory */
	mem_buffer_init(mode);

	switch (mode) {
	case EARLY_SLEEP:
		break;
	case DEEP_SLEEP:
		printk("deep sleep open: DEEP_SLEEP\n");
		return 0;
		break;
	case NORMAL_RECORD:
		dmic_init_normal_mode(NORMAL_RECORD);
		dmic_record_enabled = 1;
		break;
	case NORMAL_WAKEUP:
		g_dma_mode = DMA_MODE;
		wakeup_open();
		dmic_init_normal_mode(NORMAL_RECORD);
		break;
	default:
		printk("%s:bad open mode\n", TAG);
		break;
	}

	printk("module open open_cnt = %d\n",open_cnt);
	return 0;
}
int close(int mode)
{

	/* if ( g_dmic_current_working_mode != mode) { */
	/* 	return 0; */
	/* } */

	printk("module close open_cnt = %d\n", open_cnt);
	/* MASK INTC*/
	if(mode == DEEP_SLEEP) {
		return 0;
	} else if(mode == NORMAL_RECORD) {
		dmic_record_enabled = 0;
	}

	if( --open_cnt == 0 ) {
		printk("[voice wakeup] wakeup module closed dmic.\r\n");
		dmic_close(mode);
	}

	g_dmic_current_working_mode = -1;

	return 0;
}

/* save resource */
int enter_suspend(int mode)
{
	vtw_print(LOG_DEBUG, "enter_suspend\r\n");

	if(!voice_wakeup_enabled) {
		return 0;
	}

	g_dmic_current_working_mode = DEEP_SLEEP;

	//wakeup_open();

	cpm_init_save(0);

	/* save irq mask */
	intc_save(0);

	/* save rtc */
	rtc_save();
	rtc_init();

	/* save tcu */
	tcu_timer_save(TCU_CHANNEL);
	tcu_timer_request(TCU_CHANNEL);

#if DMIC_USE_DMA
	g_dma_mode = DMA_MODE;
#else
	g_dma_mode = CPU_MODE;
#endif

	dmic_module_suspend_enter(g_dma_mode);

	return 0;
}

/* restore resource */
int exit_suspend(int mode)
{
	vtw_print(LOG_DEBUG, "exit_suspend\r\n");

	if(!voice_wakeup_enabled) {
		return 0;
	}

	dmic_module_suspend_exit(g_dma_mode);

	rtc_exit();
	rtc_restore();

	tcu_timer_restore(0);

	intc_restore(0);

	cpm_exit_restore(0);

	//wakeup_close();

	g_dmic_current_working_mode = NORMAL_WAKEUP;
	g_dma_mode = DMA_MODE;

	return 0;
}

int setup_voice_trigger(int mode)
{

	if(!voice_wakeup_enabled) {
		return 0;
	}

	vtw_print(LOG_VERBOSE, "setup_voice_trigger\r\n");

	mode = DEEP_SLEEP;

	dmic_module_reset_voice_trigger(g_dma_mode);

	return 0;
}

/* wakeup handler */
int wakeup_handler(int par)
{
	return wakeup_handler_impl(par);
}

int set_callback_handler(void *handler)
{
	return set_printk_callback(handler);
}


/* @fn: used by recorder to get address for now.
 * @return : dma trans address.
 * */
unsigned int get_dma_address(void)
{
	return dma_get_dma_address();
}

/* @fn: used by recorder to change dmic config.
 *
 * */
int ioctl(int cmd, unsigned long args)
{
	return dmic_ioctl(cmd, args);
}

/* @fn: used by deep sleep procedure to load module to L2 cache.
 *
 * */
void cache_prefetch(void)
{
	int i;

	volatile unsigned int *addr = (unsigned int *)LOAD_ADDR;
	volatile unsigned int a;
	for(i = 0; i < LOAD_SIZE/32; i++) {
		a = *(addr + i * 8);
	}
	i = a;
}

/* @fn: used by wakeup driver to get voice_resrouce buffer address.
 *
 * */
unsigned char * get_resource_addr(void)
{
	return get_wakeup_key_resource_address();
}

/* used by wakeup driver.
 * @return SYS_WAKEUP_FAILED, SYS_WAKEUP_OK.
 * */
int process_data(void)
{
	return do_voice_recognization();
}

/* used by wakeup drvier */
int is_cpu_wakeup_by_dmic(void)
{
	return cpu_wakeup_by == WAKEUP_BY_DMIC ? 1 : 0;
}

/* used by wakeup driver when early sleep. */
int set_sleep_buffer(struct sleep_buffer *sleep_buffer)
{
	g_sleep_buffer = sleep_buffer;

	dma_stop( );

	/* switch to early sleep */
	dma_config_early_sleep(sleep_buffer);

	dma_start( );

	return 0;
}

/* used by cpu eary sleep.
 * @return SYS_WAKEUP_OK, SYS_WAKEUP_FAILED.
 * */
int get_sleep_process(void)
{
	struct sleep_buffer *sleep_buffer = g_sleep_buffer;
	int i, len, ret = SYS_WAKEUP_FAILED;

	if(!voice_wakeup_enabled) {
		return SYS_WAKEUP_FAILED;
	}

	len = sleep_buffer->total_len/sleep_buffer->nr_buffers;
	for(i = 0; i < sleep_buffer->nr_buffers; i++) {
		if(process_buffer_data(sleep_buffer->buffer[i], len) == SYS_WAKEUP_OK) {
			cpu_wakeup_by = WAKEUP_BY_DMIC;
			ret = SYS_WAKEUP_OK;
			break;
		}
	}
#if 0
	dma_stop();
	/* switch to dmic normal config */
	dma_config_normal();

	//wakeup_reset_fifo();

	dma_start();
#endif
	return ret;
}

int set_dma_channel(int channel)
{
	//_dma_channel = channel;
	dma_set_channel(channel);
	return 0;
}

int voice_wakeup_enable(int enable)
{
	voice_wakeup_enabled = enable;
	return voice_wakeup_enabled;
}

int is_voice_wakeup_enabled(void)
{
	return voice_wakeup_enabled;
}

/* internal function */
int get_voice_wakeup_state()
{
	//return g_dmic_current_working_mode;
	return DEEP_SLEEP;
}

/* internal function */
int set_wakeup_source_type(int type)
{
	cpu_wakeup_by = type;
	return 0;
}

/*
 * clear bss and init global variables
 */
int module_init(void)
{

	/*global init*/
	//dma_set_channel(3);  // init at wakeup_module_init()

	cpu_wakeup_by = 0;
	open_cnt = 0;
	g_dmic_current_working_mode = 0;
	g_sleep_buffer = NULL;
	voice_wakeup_enabled = 0;
	dmic_record_enabled = 0;

	g_dma_mode = DMA_MODE;

	/* setup memory buffer space */
	mem_buffer_init(0);

	return 0;
}


#if 0
static char bss_start[0] __attribute__((section(".__bss_start")));
static char bss_end[0] __attribute__((section(".__bss_end")));

/*
 * clear bss and init global variables
 */
int module_init(void)
{
	//int i;
	//unsigned char *p =(unsigned char *) __bss_start;

	/*clear bss*/
	/* for(i = 0; i < ((char*)&__bss_end[0] - &__bss_start[0]); i++) { */
	/* 	*p++ = 0; */
	/* } */

	memset((void*)&bss_start[0], 0, (&bss_end[0] - &bss_start[0]));

	/*global init*/
	//dma_set_channel(3);  // init at wakeup_module_init()

	cpu_wakeup_by = 0;
	open_cnt = 0;
	g_dmic_current_working_mode = 0;
	g_sleep_buffer = NULL;
	voice_wakeup_enabled = 0;
	dmic_record_enabled = 0;

	g_dma_mode = DMA_MODE;

	/* setup memory buffer space */
	mem_buffer_init(0);

	return 0;
}

int module_exit(void)
{
	return 0;
}

void * get_symbol(const char *symbol_name);


struct symbol_s {
	void * sym;
	char * name;
};

#define DEF_SYM(sym) {sym, #sym}
static struct symbol_s g_symbol[] = {
	DEF_SYM(get_symbol),
	DEF_SYM(wakeup_handler),
	DEF_SYM(open),
	DEF_SYM(close),
	DEF_SYM(enter_suspend),
	DEF_SYM(exit_suspend),
	DEF_SYM(setup_voice_trigger),
	DEF_SYM(cache_prefetch),
	DEF_SYM(set_callback_handler),
	DEF_SYM(get_dma_address),
	DEF_SYM(ioctl),
	DEF_SYM(get_resource_addr),
	DEF_SYM(process_data),
	DEF_SYM(is_cpu_wakeup_by_dmic),
	DEF_SYM(set_sleep_buffer),
	DEF_SYM(get_sleep_process),
	DEF_SYM(set_dma_channel),
	DEF_SYM(voice_wakeup_enable),
	DEF_SYM(is_voice_wakeup_enabled),
	DEF_SYM(module_init),
	DEF_SYM(module_exit),
};
#undef DEF_SYM

#define SYMBOLS_NUM (sizeof(g_symbol)/sizeof(struct symbol_s))

void * get_symbol(const char *symbol_name)
{
	int i;
	struct symbol_s *s;

	s = &g_symbol[0];
	for (i=0; i<SYMBOLS_NUM; i++) {
		if ( strcmp(s->name, symbol_name) == 0 ) {
			return s->sym;
		}
		s++;
	}

	return NULL;
}
#endif
