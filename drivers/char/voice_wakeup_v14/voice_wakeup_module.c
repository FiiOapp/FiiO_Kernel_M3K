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

#include <linux/voice_wakeup_module.h>

//#define VOICE_WAKEUP_DEBUG


#define FIRMWARE_LOAD_ADDRESS	0xa1f00000

static int wakeup_firmware [] = {

#include "voice_wakeup_firmware.hex"

};

#define SYM_STR_OPEN "open"

struct wakeup_module_ops {
	/*wakeup module for host*/
	void * (*get_symbol)(const char *symbol_name);
	int (*wakeup_handler)(int args);
	int (*open)(int mode);
	int (*close)(int mode);
	int (*enter_suspend)(int mode);
	int (*exit_suspend)(int mode);
	int (*setup_voice_trigger)(int mode);

	int (*cache_prefetch)(void);
	/*host for wakeup module*/
	int (*set_callback_handler)(void *);
	dma_addr_t (*get_dma_address)(void);
	int (*ioctl)(int cmd, unsigned long args);
	unsigned char* (*get_resource_addr)(void);
	int (*process_data)(void);
	int (*is_cpu_wakeup_by_dmic)(void);
	int (*set_sleep_buffer)(struct sleep_buffer *);
	int (*get_sleep_process)(void);
	int (*set_dma_channel)(int);
	int (*voice_wakeup_enable)(int);
	int (*is_voice_wakeup_enabled)(void);

	int (*_module_init)(void);
	int (*_module_exit)(void);
};

static struct wakeup_module_ops wakeup_ops;
static struct wakeup_module_ops *m_ops = &wakeup_ops;

#ifdef VOICE_WAKEUP_DEBUG
static void dump_firmware(void)
{
	int i;
	unsigned int *p = (unsigned int *)FIRMWARE_LOAD_ADDRESS;
	printk("###################dump_firmware begine################\n");
	for(i = 0; i < 64; i++) {
		printk("1.%p:%08x\n", &wakeup_firmware[i], wakeup_firmware[i]);
		printk("2.%p:%08x\n", p+i, *(p+i));
	}
	printk("###################dump_firmware end################\n");

}
#endif

static void setup_ops(void)
{
	unsigned long * get_symbol;

	printk("###############setup_ops##############\n");
	m_ops = &wakeup_ops;
	get_symbol = (unsigned long *)FIRMWARE_LOAD_ADDRESS;
	m_ops->get_symbol = (void *) *get_symbol;

	return;
}
#ifdef VOICE_WAKEUP_DEBUG
void test_ops(void)
{
	printk("get_symbol:%p\n", m_ops->get_symbol);
	printk("open:%p\n", m_ops->open);
	printk("handler:%p\n", m_ops->handler);
	printk("close:%p\n", m_ops->close);
	printk("set_callback_handler:%p\n", m_ops->set_callback_handler);
	printk("get_resource_addr:%p\n", m_ops->get_resource_addr);
	printk("_module_init:%p\n", m_ops->_module_init);
	printk("_module_exit:%p\n", m_ops->_module_exit);
	printk("###############ops end##############\n");

	/* get_symbol */
	{
		void * s;
#undef SYM_STR
#define SYM_STR "open"
		s = m_ops->get_symbol(SYM_STR); printk(SYM_STR ": %p\n", s);
		m_ops->open = s;
#undef SYM_STR
#define SYM_STR "close"
		s = m_ops->get_symbol(SYM_STR); printk(SYM_STR ": %p\n", s);
		m_ops->close = s;
	}

	printk("printk:%p", printk);
	//m_ops->set_callback_handler(printk);
	printk("###############test_ops##############\n");
	printk("m_ops.open:%x\n", m_ops->open(1));
//	printk("m_ops.handler:%x\n", m_ops->handler(1));
	printk("m_ops.close:%x\n", m_ops->close(1));
	printk("###############ops end##############\n");

	return ;
}
#endif

int wakeup_module_handler(int par)
{
	if (!m_ops->wakeup_handler) {
		m_ops->wakeup_handler = m_ops->get_symbol("wakeup_handler");
		if (!m_ops->wakeup_handler) {
			printk("[voice wakeup] failed get_symbol(wakeup_handler);\n");
			return -1;
		}
	}
	return m_ops->wakeup_handler(par);
}
EXPORT_SYMBOL(wakeup_module_handler);

int wakeup_module_open(int mode)
{
	if (!m_ops->open) {
		m_ops->open = m_ops->get_symbol("open");
		if (!m_ops->open) {
			printk("[voice wakeup] failed get_symbol(open);\n");
			return -1;
		}
	}

	return m_ops->open(mode);
}
EXPORT_SYMBOL(wakeup_module_open);

int wakeup_module_close(int mode)
{
	if (!m_ops->close) {
		m_ops->close = m_ops->get_symbol("close");
		if (!m_ops->close) {
			printk("[voice wakeup] failed get_symbol(close);\n");
			return -1;
		}
	}
	return m_ops->close(mode);
}
EXPORT_SYMBOL(wakeup_module_close);

int wakeup_module_enter_suspend(int mode);
int wakeup_module_exit_suspend(int mode);
int wakeup_module_setup_voice_trigger(int mode);

int wakeup_module_enter_suspend(int mode)
{
	if (!m_ops->enter_suspend) {
		m_ops->enter_suspend = m_ops->get_symbol("enter_suspend");
		if (!m_ops->enter_suspend) {
			printk("[voice wakeup] failed get_symbol(enter_suspend);\n");
			return -1;
		}
	}

	return m_ops->enter_suspend(mode);
}
EXPORT_SYMBOL(wakeup_module_enter_suspend);

int wakeup_module_exit_suspend(int mode)
{
	if (!m_ops->exit_suspend) {
		m_ops->exit_suspend = m_ops->get_symbol("exit_suspend");
		if (!m_ops->exit_suspend) {
			printk("[voice wakeup] failed get_symbol(exit_suspend);\n");
			return -1;
		}
	}

	return m_ops->exit_suspend(mode);
}
EXPORT_SYMBOL(wakeup_module_exit_suspend);

int wakeup_module_setup_voice_trigger(int mode)
{
	if (!m_ops->setup_voice_trigger) {
		m_ops->setup_voice_trigger = m_ops->get_symbol("setup_voice_trigger");
		if (!m_ops->setup_voice_trigger) {
			printk("[voice wakeup] failed get_symbol(setup_voice_trigger);\n");
			return -1;
		}
	}

	return m_ops->setup_voice_trigger(mode);
}
EXPORT_SYMBOL(wakeup_module_setup_voice_trigger);



void wakeup_module_cache_prefetch(void)
{
	if (!m_ops->cache_prefetch) {
		m_ops->cache_prefetch = m_ops->get_symbol("cache_prefetch");
		if (!m_ops->cache_prefetch) {
			printk("[voice wakeup] failed get_symbol(cache_prefetch);\n");
			return;
		}
	}
	m_ops->cache_prefetch();
}
EXPORT_SYMBOL(wakeup_module_cache_prefetch);

dma_addr_t wakeup_module_get_dma_address(void)
{
	if (!m_ops->get_dma_address) {
		m_ops->get_dma_address = m_ops->get_symbol("get_dma_address");
		if (!m_ops->get_dma_address) {
			printk("[voice wakeup] failed get_symbol(get_dma_address);\n");
			return -1;
		}
	}
	return	m_ops->get_dma_address();
}
EXPORT_SYMBOL(wakeup_module_get_dma_address);

int wakeup_module_ioctl(int cmd, unsigned long args)
{
	if (!m_ops->ioctl) {
		m_ops->ioctl = m_ops->get_symbol("ioctl");
		if (!m_ops->ioctl) {
			printk("[voice wakeup] failed get_symbol(ioctl);\n");
			return -1;
		}
	}

	return m_ops->ioctl(cmd, args);
}
EXPORT_SYMBOL(wakeup_module_ioctl);

unsigned char * wakeup_module_get_resource_addr(void)
{
	if (!m_ops->get_resource_addr) {
		m_ops->get_resource_addr = m_ops->get_symbol("get_resource_addr");
		if (!m_ops->get_resource_addr) {
			printk("[voice wakeup] failed get_symbol(get_resource_addr);\n");
			return NULL;
		}
	}

	return m_ops->get_resource_addr();
}

int wakeup_module_process_data(void)
{
	if (!m_ops->process_data) {
		m_ops->process_data = m_ops->get_symbol("process_data");
		if (!m_ops->process_data) {
			printk("[voice wakeup] failed get_symbol(process_data);\n");
			return -1;
		}
	}

	return m_ops->process_data();
}
EXPORT_SYMBOL(wakeup_module_process_data);

int wakeup_module_is_cpu_wakeup_by_dmic(void)
{
	if (!m_ops->is_cpu_wakeup_by_dmic) {
		m_ops->is_cpu_wakeup_by_dmic = m_ops->get_symbol("is_cpu_wakeup_by_dmic");
		if (!m_ops->is_cpu_wakeup_by_dmic) {
			printk("[voice wakeup] failed get_symbol(is_cpu_wakeup_by_dmic);\n");
			return -1;
		}
	}
	return m_ops->is_cpu_wakeup_by_dmic();
}
EXPORT_SYMBOL(wakeup_module_is_cpu_wakeup_by_dmic);

int wakeup_module_set_sleep_buffer(struct sleep_buffer * sleep_buffer)
{
	if (!m_ops->set_sleep_buffer) {
		m_ops->set_sleep_buffer = m_ops->get_symbol("set_sleep_buffer");
		if (!m_ops->set_sleep_buffer) {
			printk("[voice wakeup] failed get_symbol(set_sleep_buffer);\n");
			return -1;
		}
	}

	return m_ops->set_sleep_buffer(sleep_buffer);
}
EXPORT_SYMBOL(wakeup_module_set_sleep_buffer);

int wakeup_module_get_sleep_process(void)
{
	if (!m_ops->get_sleep_process) {
		m_ops->get_sleep_process = m_ops->get_symbol("get_sleep_process");
		if (!m_ops->get_sleep_process) {
			printk("[voice wakeup] failed get_symbol(get_sleep_process);\n");
			return -1;
		}
	}

	return m_ops->get_sleep_process();
}
EXPORT_SYMBOL(wakeup_module_get_sleep_process);

int wakeup_module_set_dma_channel(int channel)
{
	if (!m_ops->set_dma_channel) {
		m_ops->set_dma_channel = m_ops->get_symbol("set_dma_channel");
		if (!m_ops->set_dma_channel) {
			printk("[voice wakeup] failed get_symbol(set_dma_channel);\n");
			return -1;
		}
	}

	return m_ops->set_dma_channel(channel);
}
EXPORT_SYMBOL(wakeup_module_set_dma_channel);

int wakeup_module_wakeup_enable(int enable)
{
	if (!m_ops->voice_wakeup_enable) {
		m_ops->voice_wakeup_enable = m_ops->get_symbol("voice_wakeup_enable");
		if (!m_ops->voice_wakeup_enable) {
			printk("[voice wakeup] failed get_symbol(voice_wakeup_enable);\n");
			return -1;
		}
	}

	return m_ops->voice_wakeup_enable(enable);
}
EXPORT_SYMBOL(wakeup_module_wakeup_enable);

int wakeup_module_is_wakeup_enabled(void)
{
	if (!m_ops->is_voice_wakeup_enabled) {
		m_ops->is_voice_wakeup_enabled = m_ops->get_symbol("is_voice_wakeup_enabled");
		if (!m_ops->is_voice_wakeup_enabled) {
			printk("[voice wakeup] failed get_symbol(is_voice_wakeup_enabled);\n");
			return -1;
		}
	}

	return m_ops->is_voice_wakeup_enabled();
}
EXPORT_SYMBOL(wakeup_module_is_wakeup_enabled);

int wakeup_module_set_callback_handler(void * h)
{
	if (!m_ops->set_callback_handler) {
		m_ops->set_callback_handler = m_ops->get_symbol("set_callback_handler");
		if (!m_ops->set_callback_handler) {
			printk("[voice wakeup] failed get_symbol(set_callback_handler);\n");
			return -1;
		}
	}

	return m_ops->set_callback_handler(h);
}
EXPORT_SYMBOL(wakeup_module_set_callback_handler);

int _module_init(void)
{
	if (!m_ops->_module_init) {
		m_ops->_module_init = m_ops->get_symbol("module_init");
		if (!m_ops->_module_init) {
			printk("[voice wakeup] failed get_symbol(_module_init);\n");
			return -1;
		}
	}

	return m_ops->_module_init();
}

int _module_exit(void)
{
	if (!m_ops->_module_exit) {
		m_ops->_module_exit = m_ops->get_symbol("module_exit");
		if (!m_ops->_module_exit) {
			printk("[voice wakeup] failed get_symbol(_module_exit);\n");
			return -1;
		}
	}

	return m_ops->_module_exit();
}

static int __init wakeup_module_init(void)
{
	/* load voice wakeup firmware */
	memcpy((void *)FIRMWARE_LOAD_ADDRESS, wakeup_firmware, sizeof(wakeup_firmware));
	setup_ops();

#ifdef VOICE_WAKEUP_DEBUG
	dump_firmware();
	test_ops();
#endif
	wakeup_module_set_callback_handler((void*)printk);
	_module_init();
	wakeup_module_set_dma_channel(0);

	return 0;
}
static void __exit wakeup_module_exit(void)
{
	_module_exit();
}
module_init(wakeup_module_init);
module_exit(wakeup_module_exit);

MODULE_AUTHOR("qipengzhen<aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("voice wakeup module driver");
MODULE_LICENSE("GPL");
