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

#include "interface.h"
#include "voice_wakeup.h"
#include "dma_ops.h"
#include "dmic_ops.h"
#include "jz_dmic.h"
#include "rtc_ops.h"
#include "dmic_config.h"
#include "jz_dma.h"
#include "tcu_timer.h"

/* #define INTERFACE_VOICE_DEBUG */
#define TAG	"[voice_wakeup]"

char __bss_start[0] __attribute__((section(".__bss_start")));
char __bss_end[0] __attribute__((section(".__bss_end")));

int (*h_handler)(const char *fmt, ...);

enum wakeup_source {
	WAKEUP_BY_OTHERS = 1,
	WAKEUP_BY_DMIC
};

/* global config: default value */
unsigned int _dma_channel = 3;
static unsigned int tcu_channel = 5;

unsigned int cpu_wakeup_by = 0;
static unsigned int open_cnt = 0;
unsigned int current_mode = 0;
struct sleep_buffer *g_sleep_buffer;
static int voice_wakeup_enabled = 0;
static int dmic_record_enabled = 0;
static unsigned int rtc_count = 0;
static unsigned int dmic_working = 0;
int g_dma_mode = 0;
static int g_dmic_current_working_mode = 0;

#ifdef INTERFACE_VOICE_DEBUG
void dump_voice_wakeup(void)
{
	printk("###########dump voice wakeup status#######\n");
	printk("cpu_wakeup_by:			%d\n", cpu_wakeup_by);
	printk("open_cnt:				%d\n", open_cnt);
	printk("voice_wakeup_enabled:	%d\n", voice_wakeup_enabled);
	printk("dmic_record_enabled:	%d\n", dmic_record_enabled);

	printk("wakeup_failed_times:	%d\n", wakeup_failed_times);
	printk("dmic_current_state:	%d\n", dmic_current_state);

	printk("###########dump voice wakeup status#######\n");

}
#endif

int open(int mode)
{


	/* rtc irq test */
	if (0 && mode == DEEP_SLEEP ) {
		g_dma_mode = CPU_MODE;
		/* DEEP_SLEEP */
		dmic_disable_tri();
		dmic_disable();
		rtc_set_alarm_and_polling_rtc_alarm_flag(1);
		return 0;
	}

	if ( g_dmic_current_working_mode == mode) {
		/* skip return for sleep again in pm_enter() */
		//return 0;
	}

	switch (mode) {
	case EARLY_SLEEP:
		break;
	case DEEP_SLEEP:
		printk("deep sleep open\n");
		if(!voice_wakeup_enabled) {
			return 0;
		}
		/* stop dma */
		dma_stop(_dma_channel);
#if DMIC_USE_DMA
		g_dma_mode = DMA_MODE;
#else
		g_dma_mode = CPU_MODE;
#endif
		rtc_init();
		dmic_init_mode(DEEP_SLEEP);
		wakeup_open();

#ifdef CONFIG_TCU_TIMER_WAKEUP
		tcu_timer_request(tcu_channel);
		REG32(0xB000100C) = 1<<26; /*tcu1 int en*/
#endif
		/* UNMASK INTC we used */
		REG32(0xB000100C) = 1<<0; /*dmic int en*/
		REG32(0xB000102C) = 1<<0; /*rtc int en*/
#ifdef INTERFACE_VOICE_DEBUG
		dump_voice_wakeup();
#endif
		break;
	case NORMAL_RECORD:
		dmic_init_mode(NORMAL_RECORD);
		dmic_record_enabled = 1;
		break;
	case NORMAL_WAKEUP:
		g_dma_mode = DMA_MODE;
		wakeup_open();
		dmic_init_mode(NORMAL_RECORD);
		break;
	default:
		printk("%s:bad open mode\n", TAG);
		break;
	}

	if ( mode != DEEP_SLEEP ) {
		if (g_dma_mode == DMA_MODE) { /* DMIC_USE_DMA	dma */
			dma_config_normal();
			dma_start(_dma_channel);
		}
		else {
			dma_stop(_dma_channel);
			/* dma_close(); ??? */
		}

		dmic_enable();
	}
	else {
		/* DEEP_SLEEP */
		/* if ( g_dmic_current_working_mode != mode) { */
		/* 	rtc_set_alarm_and_polling_rtc_alarm_flag(1); */
		/* } */

		if (g_dma_mode == DMA_MODE) { /* DMIC_USE_DMA	dma */
			/* reset dma, stop dma first. */
			dma_stop(_dma_channel);
			dma_config_normal();

			/* WARNING:
			 * Do not start dma before cpu resume.
			 * So move dma_start() to dmic trigger handler.
			 */
			//dma_start(_dma_channel);
		}
		else {
			dma_stop(_dma_channel);
			/* dma_close(); ??? */
		}

	}

	if ( g_dmic_current_working_mode != mode) {
		open_cnt++;
		g_dmic_current_working_mode = mode;
	}

	printk("module open open_cnt = %d\n",open_cnt);
	return 0;
}
int close(int mode)
{

	/* if ( g_dmic_current_working_mode != mode) { */
	/* 	return 0; */
	/* } */

	g_dmic_current_working_mode = -1;

	printk("module close open_cnt = %d\n", open_cnt);
	/* MASK INTC*/
	if(mode == DEEP_SLEEP) {
		if(voice_wakeup_enabled) {
			REG32(0xB0001008) |= 1<< 0;
			//REG32(0xB0001008) |= 1<< 26;
			REG32(0xB0001028) |= 1<< 0;
			dmic_disable_tri();
			wakeup_close();
			dmic_ioctl(DMIC_IOCTL_SET_SAMPLERATE, 16000);
#ifdef INTERFACE_VOICE_DEBUG
			dump_voice_wakeup();
#endif

			/* resume dma */
			if (g_dma_mode == CPU_MODE) { /* DMIC_USE_DMA	dma */
				g_dma_mode = DMA_MODE;
				wakeup_open();
				dmic_init_mode(NORMAL_RECORD);
				dma_config_normal();
				dma_start(_dma_channel);
				dmic_enable();
			}

		} else {
			return 0;
		}
	} else if(mode == NORMAL_RECORD) {
		dmic_record_enabled = 0;
	}

	if(--open_cnt == 0) {
		printk("[voice wakeup] wakeup module closed for real. \n");
		dmic_disable();
		dma_close();
	}
	return 0;
}

static inline void flush_dcache_all()
{
	u32 addr;

	for (addr = 0x80000000; addr < 0x80000000 + CONFIG_SYS_DCACHE_SIZE; \
	     addr += CONFIG_SYS_CACHELINE_SIZE) {
		cache_op(0x01, addr); /*Index Invalid. */
	}
}

static inline void cpu_deep_sleep(void)
{
	unsigned int opcr;
	unsigned int val;
	unsigned int lcr;

	TCSM_PCHAR('D');
	TCSM_PCHAR('s');
	TCSM_PCHAR('l');
	TCSM_PCHAR('p');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

	/* cpu enter sleep */
	lcr = REG32(CPM_IOBASE + CPM_LCR);
	lcr &= ~(3|(0xfff<<8));
	lcr |= 0xfff << 8;	/* power stable time */
	lcr |= 1;
	REG32(CPM_IOBASE + CPM_LCR) = lcr;

	opcr = (CPM_IOBASE + CPM_OPCR);
	opcr &= ~((1 << 7) | (1 << 6) | (1 << 4) | (0xfff << 8) | (1 << 22) | (1 << 25) | (1 << 28)| (1 << 3));
	opcr |= (1 << 31) | (1 << 30) | (1 << 25) | (1 << 23) | (0xfff << 8) | (1 << 2) | (1 << 3) | (1 << 4);
	REG32(CPM_IOBASE + CPM_OPCR) = opcr;
	opcr = REG32(CPM_IOBASE + CPM_OPCR);

	cache_prefetch_voice(LABLE1,512);
LABLE1:
	flush_dcache_all();
	while(!cpu_should_sleep());

	val = REG32(0xb0000000);
	val &= ~(0xff);
	val |= 0x73;
	REG32(0xb0000000) = val;
	while((REG32(0xB00000D4) & 7))
		TCSM_PCHAR('A');

	__asm__ volatile(".set mips32\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"wait\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			".set mips32 \n\t");
	/* can't be here ,it's err occurs!!! */
	while(1) {
		TCSM_PCHAR('S');
		TCSM_PCHAR('E');
		TCSM_PCHAR('R');
		TCSM_PCHAR('R');
		TCSM_PCHAR(' ');
	}
}

static inline void cpu_normal_sleep(void)
{
	unsigned int opcr;
	unsigned int lcr;

	TCSM_PCHAR('N');
	TCSM_PCHAR('s');
	TCSM_PCHAR('l');
	TCSM_PCHAR('p');
	TCSM_PCHAR('\r');
	TCSM_PCHAR('\n');

	/* cpu enter sleep */
	lcr = REG32(CPM_IOBASE + CPM_LCR);
	lcr &= ~(3|(0xfff<<8));
	lcr |= 0xfff << 8;	/* power stable time */
	lcr |= 1;
	REG32(CPM_IOBASE + CPM_LCR) = lcr;

	opcr = (CPM_IOBASE + CPM_OPCR);
	opcr &= ~((1 << 7) | (1 << 6) | (1 << 4) | (0xfff << 8) | (1 << 22) | (1 << 25) | (1 << 28)| (1 << 3));
	opcr |= (1 << 31) | (1 << 30) | (1 << 25) | (1 << 23) | (0xfff << 8) | (1 << 2) | (1 << 4);
	REG32(CPM_IOBASE + CPM_OPCR) = opcr;
	opcr = REG32(CPM_IOBASE + CPM_OPCR);

	cache_prefetch_voice(LABLE1,32);
LABLE1:
	__asm__ volatile(".set mips32\n\t"
			"nop\n\t"
			"nop\n\t"
			"wait\n\t"
			"nop\n\t"
			"nop\n\t"
			"nop\n\t"
			".set mips32 \n\t");
//	TCSM_PCHAR('s');
}

static inline void cpu_idle(void)
{
	unsigned int val;

	/* TCSM_PCHAR('I'); */
	/* TCSM_PCHAR('D'); */
	/* TCSM_PCHAR('L'); */
	/* TCSM_PCHAR('E'); */
	/* TCSM_PCHAR('\r'); */
	/* TCSM_PCHAR('\n'); */

	/* cpu enter sleep */
	val = REG32(CPM_IOBASE + CPM_LCR);
	val &= ~(3);
	REG32(CPM_IOBASE + CPM_LCR) = val;

	val = (CPM_IOBASE + CPM_OPCR);
	val &= ~((1 << 30) | (1 << 28) | (1 << 22) | (0xfff << 8) | (1 << 7) | (1 << 6) |  (1 << 3) | (1 << 2));
	val |=  (1 << 31) | (1 << 25) | (1 << 23) | (0xfff << 8) | (1 << 4);
	REG32(CPM_IOBASE + CPM_OPCR) = val;
	val = REG32(CPM_IOBASE + CPM_OPCR);

	__asm__ volatile(".set mips32\n\t"
			"nop\n\t"
			"wait\n\t"
			"nop\n\t"
			"nop\n\t"
			".set mips32\n\t");
//	TCSM_PCHAR('I');
}

/*	int mask that we care about.
 *	DMIC INTS: used to wakeup cpu.
 *	RTC	 INTS: used to sum wakeup failed times. and adjust thr value.
 *	TCU	 INTS: used for cpu to process data.
 * */
#define INTC0_MASK	0xfBfffffe
#define INTC1_MASK	0xfffffffe
#define RTC_IRQ         (0x1 << 0)
#define DMIC_IRQ        (0x1 << 0)

/* desc: this function is only called when cpu is in deep sleep
 * @par: no use.
 * @return : SYS_WAKEUP_OK, SYS_WAKEUP_FAILED.
 * */
static int dma_mode_handler(int par)
{
	volatile int ret;
	volatile unsigned int int0;
	volatile unsigned int int1;

	//TCSM_PCHAR('H');

	//rtc_set_alarm(ALARM_VALUE);

	while(1) {
		//TCSM_PCHAR('w'); //TCSM_PCHAR('\r');TCSM_PCHAR('\n');

		int0 = REG32(0xb0001010);
		int1 = REG32(0xb0001030);
#if 0
		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');
		TCSM_PCHAR('i');
		TCSM_PCHAR('r');
		TCSM_PCHAR('p');
		TCSM_PCHAR(' ');
		serial_put_hex(int0);
		TCSM_PCHAR(' ');
		serial_put_hex(int1);
		TCSM_PCHAR(' ');
#endif
		/* wakeup by rtc alarm, but irq miss. */
		if ( int0 == 0 && int1 == 0 ) {
			/* check rtc alarm flag */

			//int1 = RTC_IRQ;
			//cpu_deep_sleep();
		}

		if((int0 & INTC0_MASK) || (int1 & INTC1_MASK)) {
			/* serial_put_hex(REG32(0xb0001010)); */
			/* serial_put_hex(REG32(0xb0001030)); */
			cpu_wakeup_by = WAKEUP_BY_OTHERS;
			ret = SYS_WAKEUP_OK;
			break;
		}

		/* RTC interrupt pending */
		if(int1 & RTC_IRQ) {
			TCSM_PCHAR('R');
			TCSM_PCHAR('T');
			TCSM_PCHAR('C');
			TCSM_PCHAR('\r');
			TCSM_PCHAR('\n');
			ret = rtc_int_handler();
			if(ret == SYS_TIMER) {
				/* serial_put_hex(REG32(0xb0001010)); */
				/* serial_put_hex(REG32(0xb0001030)); */
				ret = SYS_WAKEUP_OK;
				cpu_wakeup_by = WAKEUP_BY_OTHERS;
				break;
			} else if (ret == DMIC_TIMER) {
					continue;
			}
		}

#ifdef CONFIG_TCU_TIMER_WAKEUP
#define TCU0_MASK (1<<27)
#define TCU1_MASK (1<<26)
#define TCU2_MASK (1<<25)

		if(int0 & TCU1_MASK) {
			tcu_timer_handler();
		}
#endif
		//TCSM_PCHAR('D');
		ret = dmic_handler(int1);
		if(ret == SYS_WAKEUP_OK) {
			cpu_wakeup_by = WAKEUP_BY_DMIC;
			break;
		} else if(ret == SYS_NEED_DATA){
			dmic_working ++;
			cpu_idle(); /* wait dmic fifo or tcu timer */
		} else if(ret == SYS_WAKEUP_FAILED) {
			rtc_count = 0;
			TCSM_PCHAR('F');
			TCSM_PCHAR('0');
			TCSM_PCHAR('\r');
			TCSM_PCHAR('\n');

			break;
		}

	} /* while (1) */

	if(1 || ret == SYS_WAKEUP_OK) {
		dmic_working = 0;
		rtc_count = 0;
		rtc_exit();
#ifdef CONFIG_TCU_TIMER_WAKEUP
		tcu_timer_release(tcu_channel);
#endif
	}

	//TCSM_PCHAR('R');
	return ret;
}


static int cpu_mode_handler(int par)
{
	volatile int ret;
	volatile unsigned int int0;
	volatile unsigned int int1;

	//TCSM_PCHAR('H');

	/* rtc irq wakeup test */
	if (0) {

		int0 = REG32(0xb0001010);
		int1 = REG32(0xb0001030);

		TCSM_PCHAR('i');
		TCSM_PCHAR('r');
		TCSM_PCHAR('p');
		TCSM_PCHAR(' ');
		serial_put_hex(int0);
		TCSM_PCHAR(' ');
		serial_put_hex(int1);
		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');

		//return SYS_WAKEUP_OK;
		return SYS_NEED_DATA;

		rtc_set_alarm_and_polling_rtc_alarm_flag(1);

		return SYS_NEED_DATA;
	}

	//rtc_set_alarm(ALARM_VALUE);


	do {
#if 0
	TCSM_PCHAR('W');
		int0 = REG32(0xb0001000);
		int1 = REG32(0xb0001020);

		TCSM_PCHAR('\r');
		TCSM_PCHAR('\n');

		TCSM_PCHAR('i');
		TCSM_PCHAR('r');
		TCSM_PCHAR('s');
		TCSM_PCHAR(' ');
		serial_put_hex(int0);
		TCSM_PCHAR(' ');
		serial_put_hex(int1);
		TCSM_PCHAR(' ');
#endif
		int0 = REG32(0xb0001010);
		int1 = REG32(0xb0001030);

#if 0
		TCSM_PCHAR('i');
		TCSM_PCHAR('r');
		TCSM_PCHAR('p');
		TCSM_PCHAR(' ');
		serial_put_hex(int0);
		TCSM_PCHAR(' ');
		serial_put_hex(int1);
		TCSM_PCHAR(' ');
#endif
		if((int0 & INTC0_MASK) || (int1 & INTC1_MASK)) {
			/* serial_put_hex(REG32(0xb0001010)); */
			/* serial_put_hex(REG32(0xb0001030)); */
			cpu_wakeup_by = WAKEUP_BY_OTHERS;
			ret = SYS_WAKEUP_OK;
			break;
		}

		/* if( !(int0 & DMIC_IRQ) ) { */
		/* 	ret = SYS_NEED_DATA; */
		/* 	break; */
		/* } */

		/* RTC interrupt pending */
		if(0 && int1 & RTC_IRQ) {
			TCSM_PCHAR('R');
			TCSM_PCHAR('T');
			TCSM_PCHAR('C');
			TCSM_PCHAR('\r');
			TCSM_PCHAR('\n');
			ret = rtc_int_handler();
			if(ret == SYS_TIMER) {
				/* serial_put_hex(REG32(0xb0001010)); */
				/* serial_put_hex(REG32(0xb0001030)); */
				ret = SYS_WAKEUP_OK;
				cpu_wakeup_by = WAKEUP_BY_OTHERS;
				break;
			} else if (ret == DMIC_TIMER) {
				if(dmic_working)
					continue;
			}
		}

		/* TCSM_PCHAR('\r'); */
		/* TCSM_PCHAR('\n'); */
		ret = dmic_handler_cpu_mode(int1);
		if(ret == SYS_WAKEUP_OK) {
			TCSM_PCHAR('O');
			TCSM_PCHAR('K');
			TCSM_PCHAR('\r');
			TCSM_PCHAR('\n');
			cpu_wakeup_by = WAKEUP_BY_DMIC;
			break;
		} else if(ret == SYS_NEED_DATA){
			dmic_working ++;
		} else if(ret == SYS_WAKEUP_FAILED) {
			rtc_count = 0;
			TCSM_PCHAR('F');
			TCSM_PCHAR('0');
			TCSM_PCHAR('\r');
			TCSM_PCHAR('\n');
		}

	} while(1);

	if(ret == SYS_WAKEUP_OK) {
		rtc_count = 0;
		rtc_exit();
#ifdef CONFIG_TCU_TIMER_WAKEUP
		tcu_timer_release(tcu_channel);
#endif
	}

	//TCSM_PCHAR('R');
	return ret;
}


int handler(int par)
{
	int ret;
	if (g_dma_mode == DMA_MODE) {
		ret = dma_mode_handler(par);
	}
	else {
		ret = cpu_mode_handler(par);
	}

	return ret;
}




int set_handler(void *handler)
{
	h_handler = (int(*)(const char *fmt, ...)) handler;

	return 0;
}


/* @fn: used by recorder to get address for now.
 * @return : dma trans address.
 * */
unsigned int get_dma_address(void)
{
	return pdma_trans_addr(_dma_channel, DMA_DEV_TO_MEM);
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
	return (unsigned char *)wakeup_res;
}

/* used by wakeup driver.
 * @return SYS_WAKEUP_FAILED, SYS_WAKEUP_OK.
 * */
int process_data(void)
{
	return process_dma_data();
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

	/* cancel sleep_buffer */
	if (sleep_buffer==NULL) {
		return 0;
	}

	dma_stop(_dma_channel);

	/* switch to early sleep */
	dma_config_early_sleep(sleep_buffer);

	dma_start(_dma_channel);

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
	if (sleep_buffer==NULL) {
		return SYS_WAKEUP_FAILED;
	}

	len = sleep_buffer->total_len/sleep_buffer->nr_buffers;
	for(i = 0; i < sleep_buffer->nr_buffers; i++) {
		if( sleep_buffer->buffer[i] != NULL
		    && process_buffer_data(sleep_buffer->buffer[i], len) == SYS_WAKEUP_OK) {
			cpu_wakeup_by = WAKEUP_BY_DMIC;
			ret = SYS_WAKEUP_OK;
			break;
		}
	}

	dma_stop(_dma_channel);
	/* switch to dmic normal config */
	dma_config_normal();

	wakeup_reset_fifo();

	dma_start(_dma_channel);

	return ret;
}

int set_dma_channel(int channel)
{
	_dma_channel = channel;
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
/*
 * clear bss and init global variables
 */
int module_init(void)
{
	int i;
	/*clear bss*/
	unsigned char *p =(unsigned char *) __bss_start;

	for(i = 0; i < ((char*)&__bss_end - __bss_start); i++) {
		*p++ = 0;
	}

	/*global init*/
	_dma_channel = 3;
	tcu_channel = 5;
	cpu_wakeup_by = 0;
	open_cnt = 0;
	current_mode = 0;
	g_sleep_buffer = NULL;
	voice_wakeup_enabled = 0;
	dmic_record_enabled = 0;

	g_dma_mode = DMA_MODE;

	return 0;
}

int module_exit(void)
{
	return 0;
}
