#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/mm.h>
#include <asm/cacheops.h>
#include <linux/dma-mapping.h>
#include <mach/jzdma.h>
#include <linux/delay.h>

#include <linux/voice_wakeup_module.h>

#include "wakeup_module_pic/include/wakeup_interface.h"


#define VOICE_WAKEUP_DEBUG



static int wakeup_firmware [] = {
#include "voice_wakeup_firmware.hex"
};


static int (*voice_firmwave_dispatch)(int cmd, void*args);
struct fw_shared_param* voice_module_param;



#ifdef VOICE_WAKEUP_DEBUG
static void dump_firmware(void* load_addr)
{
	int i;
	//unsigned int *p = (unsigned int *)load_addr;
	printk("###################dump_firmware begine################\n");
	for(i = 0; i < 64; i++) {
		printk("%x: %p %08x\n", i*4, &wakeup_firmware[i], wakeup_firmware[i]);
		//printk("%x: %p %08x\n", i*4, p+i, *(p+i));
	}
	printk("###################dump_firmware end################\n");

}
#endif

static void load_firmware(void)
{
	void * load_addr;
	void * stack;
	void (*fw_load)(void* load_addr, void* interface, void* shared_param, void* stack);


	//load_addr = (void*)SLEEP_TCSM_SPACE;
	//load_addr = (void *)__get_free_pages(GFP_KERNEL, );
	load_addr = (void *)kmalloc(0x100000, GFP_KERNEL);
	if (!load_addr) {
		printk("kmalloc load_addr failed\n");
	}
	printk("load_addr= %p, sizeof(wakeup_firmware) = %d\n", load_addr, sizeof(wakeup_firmware));

#ifdef VOICE_WAKEUP_DEBUG
	dump_firmware(load_addr);
#endif
	/* load voice wakeup firmware */
	memcpy((void *)load_addr, wakeup_firmware, sizeof(wakeup_firmware));
	if (0) {
		int i, size;
		unsigned char * d, * s;
		size = sizeof(wakeup_firmware);
		d = (unsigned char *)load_addr;
		s = (unsigned char *)wakeup_firmware;
		for (i=0; i<size; i++) {
			if ((i&0xff)==0)
				printk("\t%p %p\n", d, s);
			*d++ = *s++;
		}
	}

	printk("load_addr= %p, sizeof(wakeup_firmware) = %d\n", load_addr, sizeof(wakeup_firmware));

	dma_cache_wback_inv((unsigned long)load_addr,sizeof(wakeup_firmware));
	//dma_cache_wback_inv((unsigned long)0x80000000, 1024*32);
	mb();


	printk("############################# load_addr=%p\n", load_addr);

	fw_load = (void (*)(void* load_addr, void* interface, void* shared_param, void* callback))load_addr;

	voice_module_param = (struct fw_shared_param *)__get_free_page(GFP_KERNEL);
	//stack = (void *)__get_free_page(GFP_KERNEL);
	//stack = (void *)__get_free_pages(GFP_KERNEL, 2);
	stack = (void *)kmalloc(0x100000, GFP_KERNEL);
	//memset((void*)stack, 0, 8192);
	printk("func((unsigned int)load_addr= %p, param= %p, stack=%p\n", load_addr, voice_module_param, (void*)(stack+0x100000-4));

	//msleep(3000);

	fw_load((void*)load_addr, (void*)&voice_firmwave_dispatch, (void*)voice_module_param, (void*)(stack+4096-4));
	printk("fw_load() voice_firmwave_dispatch()= %p\n", voice_firmwave_dispatch);

	printk("setup_ops() end...\n");
	return;
}

#ifdef VOICE_WAKEUP_DEBUG
void test_ops(void*load_addr)
{

	return ;
}
#endif



static int call_firmwave_interface(int cmd, void * args)
{
	if (voice_firmwave_dispatch) {
		voice_firmwave_dispatch(cmd, args);
	}
	else {
		printk("voice_firmwave_dispatch = NULL !\n");
	}

	return 0;
}


int wakeup_module_handler(int par)
{
	struct common_args a;

	a.value = (long)par;

	call_firmwave_interface(ICMD_WAKEUP_HANDLER, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_handler);

int wakeup_module_open(int mode)
{
	struct common_args a;

	a.value = (long)mode;

	call_firmwave_interface(ICMD_OPEN, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_open);

int wakeup_module_close(int mode)
{
	struct common_args a;

	a.value = (long)mode;

	call_firmwave_interface(ICMD_CLOSE, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_close);

/* int wakeup_module_enter_suspend(int mode); */
/* int wakeup_module_exit_suspend(int mode); */
/* int wakeup_module_setup_voice_trigger(int mode); */

int wakeup_module_enter_suspend(int mode)
{
	struct common_args a;
	a.value = (long)mode;
	call_firmwave_interface(ICMD_ENTER_SUSPEND, &a);
	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_enter_suspend);

int wakeup_module_exit_suspend(int mode)
{
	struct common_args a;

	a.value = (long)mode;
	call_firmwave_interface(ICMD_EXIT_SUSPEND, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_exit_suspend);

int wakeup_module_setup_voice_trigger(int mode)
{
	struct common_args a;

	a.value = (long)mode;
	call_firmwave_interface(ICMD_SETUP_VOICE_TRIGGER, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_setup_voice_trigger);


void wakeup_module_cache_prefetch(void)
{
	struct common_args a;

	//a.value = mode;
	call_firmwave_interface(ICMD_CACHE_PREFETCH, &a);

	//return a.ret_val;
	return;
}
EXPORT_SYMBOL(wakeup_module_cache_prefetch);

dma_addr_t wakeup_module_get_dma_address(void)
{
	struct common_args a;

	a.value = 0;
	call_firmwave_interface(ICMD_GET_DMA_ADDRESS, &a);

	return (dma_addr_t)a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_get_dma_address);

int wakeup_module_ioctl(int cmd, unsigned long args)
{
	struct ioctl_args a;
	printk("%s\n", __FUNCTION__);

	a.cmd = cmd;
	a.args = args;
	call_firmwave_interface(ICMD_IOCTL, &a);

	return (dma_addr_t)a.ret_val;

	return 0;
}
EXPORT_SYMBOL(wakeup_module_ioctl);

unsigned char * wakeup_module_get_resource_addr(void)
{
	struct common_args a;

	//a.value = mode;
	call_firmwave_interface(ICMD_GET_RESOURCE_ADDR, &a);

	return (unsigned char *)a.ret_val;
}

int wakeup_module_process_data(void)
{
	struct common_args a;

	//a.value = mode;
	call_firmwave_interface(ICMD_PROCESS_DATA, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_process_data);

int wakeup_module_is_cpu_wakeup_by_dmic(void)
{
	struct common_args a;

	//a.value = mode;
	call_firmwave_interface(ICMD_IS_CPU_WAKEUP_BY_DMIC, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_is_cpu_wakeup_by_dmic);

int wakeup_module_set_sleep_buffer(struct sleep_buffer * sleep_buffer)
{
	struct common_args a;

	a.value = (long)sleep_buffer;
	call_firmwave_interface(ICMD_SET_SLEEP_BUFFER, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_set_sleep_buffer);

int wakeup_module_get_sleep_process(void)
{
	struct common_args a;

	//a.value = mode;
	call_firmwave_interface(ICMD_GET_SLEEP_PROCESS, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_get_sleep_process);

int wakeup_module_set_dma_channel(int channel)
{
	struct common_args a;

	//a.value = mode;
	call_firmwave_interface(ICMD_SET_DMA_CHANNEL, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_set_dma_channel);

int wakeup_module_wakeup_enable(int enable)
{
	struct common_args a;

	a.value = (long)enable;
	call_firmwave_interface(ICMD_WAKEUP_ENABLE, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_wakeup_enable);

int wakeup_module_is_wakeup_enabled(void)
{
	struct common_args a;

	//a.value = mode;
	call_firmwave_interface(ICMD_IS_WAKEUP_ENABLED, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_is_wakeup_enabled);

int wakeup_module_set_callback_handler(void * h)
{
	struct common_args a;

	//a.value = (long)h;
	call_firmwave_interface(ICMD_SET_CALLBACK_HANDLER, &a);

	return a.ret_val;
}
EXPORT_SYMBOL(wakeup_module_set_callback_handler);


static int firmwave_exit(void)
{
	struct common_args a;

	//a.value = mode;
	call_firmwave_interface(ICMD_EXIT, &a);

	return a.ret_val;
}


static int firmware_init(void)
{
	struct init_args a;

	a.uart_id = 2;
	a.dma_id = 2;
	a.tcu_id = 5;
	a.rtc_int_val = 30;

	/* reserve memory for voice recogonition firmware */
	a.mem_size = DEFAULT_RESERVE_MEMORY_SIZE;
	a.mem = (char *) kmalloc(a.mem_size, GFP_KERNEL);

	printk("a.ret_val= %d\n", a.ret_val);
	call_firmwave_interface(ICMD_INIT, &a);
	printk("a.ret_val= %d\n", a.ret_val);
	return a.ret_val;
}



static int __init wakeup_module_init(void)
{
	printk("%d %s, BUILD: %s %s\n", __LINE__, __FUNCTION__, __DATE__, __TIME__);

	load_firmware();

	firmware_init();


	printk("%d %s\n", __LINE__, __FUNCTION__);
	return 0;
}
static void __exit wakeup_module_exit(void)
{
	firmwave_exit();
}
module_init(wakeup_module_init);
module_exit(wakeup_module_exit);

MODULE_AUTHOR("linggang<linggang.wang@ingenic.com>");
MODULE_DESCRIPTION("voice wakeup module driver");
MODULE_LICENSE("GPL");
