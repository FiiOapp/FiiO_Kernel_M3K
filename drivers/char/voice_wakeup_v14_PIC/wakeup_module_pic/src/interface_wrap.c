#include <cache.h>
//#include <uart.h>

#include "wakeup_interface.h"

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


void* program_stack;
//void* extern_func;
struct fw_shared_param *shared_param;


static int open_wrap(void * args)
{
	int mode;
	struct open_args *a = (struct open_args*)args;

	TCSM_PCHAR('O');
	mode = a->mode;
	a->ret_val = open(mode);
	return 0;
}

static int close_wrap(void * args)
{
	int mode;
	struct common_args *a = (struct common_args*)args;

	mode = a->value;
	a->ret_val = close(mode);
	return 0;
}

static int enter_suspend_wrap(void * args)
{
	int mode;
	struct common_args *a = (struct common_args*)args;

	mode = a->value;
	a->ret_val = enter_suspend(mode);
	return 0;
}

/* restore resource */
static int exit_suspend_wrap(void * args)
{
	int mode;
	struct common_args *a = (struct common_args*)args;

	mode = a->value;
	a->ret_val = exit_suspend(mode);
	return 0;
}

static int setup_voice_trigger_wrap(void * args)
{
	int mode;
	struct common_args *a = (struct common_args*)args;

	mode = a->value;
	mode = DEEP_SLEEP;
	a->ret_val = setup_voice_trigger(mode);

	return 0;
}


static int wakeup_handler_wrap(void * args)
{
	int value;
	struct common_args *a = (struct common_args*)args;

	value = a->value;
	a->ret_val = wakeup_handler(value);
	return 0;
}

static int set_callback_handler_wrap(void * args)
{
	int value;
	struct common_args *a = (struct common_args*)args;
	value = a->value;

	a->ret_val = set_callback_handler((void*)value);
	return 0;
}

static int get_dma_address_wrap(void * args)
{
	int value;
	struct common_args *a = (struct common_args*)args;
	value = a->value;

	a->ret_val = get_dma_address();
	return 0;
}

static int ioctl_wrap(void * args)
{
	int cmd;		/* ioctl cmd */
	unsigned long iargs;
	struct ioctl_args *a = (struct ioctl_args*)args;
	cmd = a->cmd;
	iargs = a->args;

	a->ret_val = ioctl(cmd, iargs);

	return 0;
}

static int cache_prefetch_wrap(void * args)
{
	struct common_args *a = (struct common_args*)args;

	cache_prefetch();

	a->ret_val = 0;
	return 0;
}

static int get_resource_addr_wrap(void * args)
{
	int value;
	struct common_args *a = (struct common_args*)args;
	value = a->value;

	a->ret_val = (unsigned int)get_resource_addr();
	return 0;
}

static int process_data_wrap(void * args)
{
	int value;
	struct common_args *a = (struct common_args*)args;
	value = a->value;

	a->ret_val = process_data();
	return 0;
}

static int is_cpu_wakeup_by_dmic_wrap(void * args)
{
	int value;
	struct common_args *a = (struct common_args*)args;
	value = a->value;

	a->ret_val = is_cpu_wakeup_by_dmic();

	return 0;
}

static int set_sleep_buffer_wrap(void * args)
{
	struct sleep_buffer *sleep_buffer;
	struct common_args *a = (struct common_args*)args;

	sleep_buffer = (struct sleep_buffer *)a->value;

	a->ret_val = set_sleep_buffer(sleep_buffer);
	return 0;
}

static int get_sleep_process_wrap(void * args)
{
	//int value;
	struct common_args *a = (struct common_args*)args;

	a->ret_val = get_sleep_process();
	return 0;
}

static int set_dma_channel_wrap(void * args)
{
	int value;
	struct common_args *a = (struct common_args*)args;
	value = a->value;

	set_dma_channel(value);

	a->ret_val = 0;
	return 0;
}

static int voice_wakeup_enable_wrap(void * args)
{
	int value;
	struct common_args *a = (struct common_args*)args;
	value = a->value;

	voice_wakeup_enable(value);

	a->ret_val = value;
	return 0;
}

static int is_voice_wakeup_enabled_wrap(void * args)
{
	int value;
	struct common_args *a = (struct common_args*)args;
	value = a->value;

	a->ret_val = is_voice_wakeup_enabled();
	return 0;
}


int fw_exit_wrap(void * args)
{
	struct init_args *a = (struct init_args*)args;

	a->ret_val = 1;
	return 0;
}

int fw_init_wrap(void * args)
{
	struct init_args *a = (struct init_args*)args;

	TCSM_PCHAR('I');
	TCSM_PCHAR(' ');

#if 0
	uart_put_hex((unsigned int)a);
	TCSM_PCHAR(' ');
	uart_put_hex(a->dma_id);
	TCSM_PCHAR(' '); 
	uart_put_hex(a->tcu_id);
	TCSM_PCHAR(' '); 
	uart_put_hex(a->rtc_int_val);
	TCSM_PCHAR('\r');TCSM_PCHAR('\n');
	TCSM_PCHAR('s'); 
	TCSM_PCHAR(' '); 
	uart_put_hex((unsigned int)serial_setid);
	TCSM_PCHAR(' '); 
	uart_put_hex((unsigned int)serial_putc);
	TCSM_PCHAR('\r');TCSM_PCHAR('\n');
#endif
	/* 
fw_init().
fw_init()......[    1.182706] Unhandled kernel unaligned access[#1]:
[    1.187662] CPU: 0 PID: 0 Comm:  Not tainted 3.10.14-00052-gd68b4a8-dirty #687

[    0.978236] ############################# load_addr=80b00000
[    0.984123] func((unsigned int)load_addr= 80b00000, param= 80a17000, stack=80cffffc
[    0.992017] fw_load() voice_firmwave_dispatch()= 80b001f8
[    0.997599] setup_ops() end...
[    1.000752] a.ret_val= -2138890660
C: 00000001
I  80B031F8 fw_init().
 80B03208 fw_init(). 80B03214 fw_init()... 80B03224 fw_init()..... 80B03234 fw_init().......
 80B031F8 fw_init().
 80B034E4 mem_buffer_init() mem_buffer_reserved:        80A20000 80B0350C  mem_buffer_size:     00020000 80B0333C 



 */
	set_debug_print_level(LOG_VERBOSE);
	vtw_print(LOG_INFO, "fw_init().\r\n");

#if 0
	serial_setid(2);
	serial_putc('I');

	serial_putc('N');
	serial_putc('I');
	serial_putc('T');
	serial_putc('\r');
	serial_putc('\n');
#endif

	//g_dma_mode = DMA_MODE;

	set_mem_buffer_reserved(a->mem, a->mem_size);

	/* setup memory buffer space */
	//mem_buffer_init(0);
	module_init();

	a->ret_val = 1;
	return 0;
}





/*
  dispatch commands
  called by start.S
 */
int command_dispatch(int cmd, void * args)
{
	TCSM_PCHAR('C'); 
	TCSM_PCHAR(':'); 
	TCSM_PCHAR(' '); 

	//uart_put_hex(cmd);
	serial_put_hex(cmd);
	TCSM_PCHAR('\r');TCSM_PCHAR('\n');

	switch (cmd) {
	case ICMD_INIT:
		fw_init_wrap(args);
		break;
	case ICMD_EXIT:
		fw_exit_wrap(args);
		break;
	case ICMD_WAKEUP_HANDLER:
		wakeup_handler_wrap(args);
		break;
	case ICMD_OPEN:
		open_wrap(args);
		break;
	case ICMD_CLOSE:
		close_wrap(args);
		break;
	case ICMD_ENTER_SUSPEND:
		enter_suspend_wrap(args);
		break;
	case ICMD_EXIT_SUSPEND:
		exit_suspend_wrap(args);
		break;
	case ICMD_SETUP_VOICE_TRIGGER:
		setup_voice_trigger_wrap(args);
		break;
	case ICMD_CACHE_PREFETCH:
		cache_prefetch_wrap(args);
		break;
	case ICMD_GET_DMA_ADDRESS:
		get_dma_address_wrap(args);
		break;
	case ICMD_IOCTL:
		ioctl_wrap(args);
		break;
	case ICMD_GET_RESOURCE_ADDR:
		get_resource_addr_wrap(args);
		break;
	case ICMD_PROCESS_DATA:
		process_data_wrap(args);
		break;
	case ICMD_IS_CPU_WAKEUP_BY_DMIC:
		is_cpu_wakeup_by_dmic_wrap(args);
		break;
	case ICMD_SET_SLEEP_BUFFER:
		set_sleep_buffer_wrap(args);
		break;
	case ICMD_GET_SLEEP_PROCESS:
		get_sleep_process_wrap(args);
		break;
	case ICMD_SET_DMA_CHANNEL:
		set_dma_channel_wrap(args);
		break;
	case ICMD_WAKEUP_ENABLE:
		voice_wakeup_enable_wrap(args);
		break;
	case ICMD_IS_WAKEUP_ENABLED:
		is_voice_wakeup_enabled_wrap(args);
		break;
	case ICMD_SET_CALLBACK_HANDLER:
		set_callback_handler_wrap(args);
		break;
	default:
		/* not support cmd */
		break;
	}

	TCSM_PCHAR('\r');TCSM_PCHAR('\n');
	TCSM_PCHAR('D'); TCSM_PCHAR('\r');TCSM_PCHAR('\n');
	return 0;
}

