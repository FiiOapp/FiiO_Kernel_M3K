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

#define FIRMWARE_LOAD_ADDRESS	0x8ff00000

static int wakeup_firmware [] = {

#include "voice_wakeup_firmware.hex"

};


struct wakeup_module_ops {
	/*pay attention to the order*/
	/*wakeup module for host*/
	int (*open)(int mode);
	int (*handler)(int args);
	int (*close)(int mode);
	int (*cache_prefetch)(void);

	/*host for wakeup module*/
	int (*set_handler)(void *);
	dma_addr_t (*get_dma_address)(void);
	int (*ioctl)(int cmd, unsigned long args);
	unsigned char * (*get_resource_addr)(void);
	int (*process_data)(void);
	int (*is_cpu_wakeup_by_dmic)(void);
	int (*set_sleep_buffer)(struct sleep_buffer *);
	int (*get_sleep_process)(void);
	int (*set_dma_channel)(int);
	int (*voice_wakeup_enable)(int);
	int (*is_voice_wakeup_enabled)(void);
	int (*cpu_should_sleep)(void);
	int (*set_record_buffer)(char *, unsigned int);
	int (*get_record_buffer)(void);
	int (*get_record_buffer_len)(void);
	int (*set_desc_addr)(char *, unsigned int);



	int (*_module_init)(void);
	int (*_module_exit)(void);
};

static struct wakeup_module_ops *m_ops;

#if 0
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
	printk("###############setup_ops##############\n");
	m_ops = (struct wakeup_module_ops *)FIRMWARE_LOAD_ADDRESS;
	m_ops->set_handler(printk);
	printk("open:%p\n", m_ops->open);
	printk("handler:%p\n", m_ops->handler);
	printk("close:%p\n", m_ops->close);
	printk("set_handler:%p\n", m_ops->set_handler);

	printk("_module_init:%p", m_ops->_module_init);
	printk("_module_exit:%p", m_ops->_module_exit);
	printk("###############ops end##############\n");
}

void test_ops(void)
{
	printk("printk:%p", printk);
	m_ops->set_handler(printk);
	printk("###############test_ops##############\n");
	printk("m_ops.open:%x\n", m_ops->open(1));
	printk("m_ops.handler:%x\n", m_ops->handler(1));
	printk("m_ops.close:%x\n", m_ops->close(1));
	printk("###############ops end##############\n");
}



int wakeup_module_open(int mode)
{
	return m_ops->open(mode);
}
EXPORT_SYMBOL(wakeup_module_open);

int wakeup_module_handler(int par)
{
	return m_ops->handler(par);
}
EXPORT_SYMBOL(wakeup_module_handler);

int wakeup_module_close(int mode)
{
	return m_ops->close(mode);
}
EXPORT_SYMBOL(wakeup_module_close);

void wakeup_module_cache_prefetch(void)
{
	m_ops->cache_prefetch();
}
EXPORT_SYMBOL(wakeup_module_cache_prefetch);

dma_addr_t wakeup_module_get_dma_address(void)
{
	return	m_ops->get_dma_address();
}
EXPORT_SYMBOL(wakeup_module_get_dma_address);

int wakeup_module_ioctl(int cmd, unsigned long args)
{

	return m_ops->ioctl(cmd, args);
}
EXPORT_SYMBOL(wakeup_module_ioctl);

unsigned char * wakeup_module_get_resource_addr(void)
{
	return m_ops->get_resource_addr();
}
int wakeup_module_process_data(void)
{
	return m_ops->process_data();
}
EXPORT_SYMBOL(wakeup_module_process_data);

int wakeup_module_is_cpu_wakeup_by_dmic(void)
{
	return m_ops->is_cpu_wakeup_by_dmic();
}
EXPORT_SYMBOL(wakeup_module_is_cpu_wakeup_by_dmic);

int wakeup_module_set_sleep_buffer(struct sleep_buffer * sleep_buffer)
{

	return m_ops->set_sleep_buffer(sleep_buffer);
}
EXPORT_SYMBOL(wakeup_module_set_sleep_buffer);

int wakeup_module_get_sleep_process(void)
{
	return m_ops->get_sleep_process();
}
EXPORT_SYMBOL(wakeup_module_get_sleep_process);

int wakeup_module_set_dma_channel(int channel)
{
	return m_ops->set_dma_channel(channel);
}
EXPORT_SYMBOL(wakeup_module_set_dma_channel);

int wakeup_module_wakeup_enable(int enable)
{
	return m_ops->voice_wakeup_enable(enable);
}
EXPORT_SYMBOL(wakeup_module_wakeup_enable);
int wakeup_module_is_wakeup_enabled(void)
{
	return m_ops->is_voice_wakeup_enabled();
}
EXPORT_SYMBOL(wakeup_module_is_wakeup_enabled);
int wakeup_module_cpu_should_sleep(void)
{
	return m_ops->cpu_should_sleep();
}
EXPORT_SYMBOL(wakeup_module_cpu_should_sleep);


int wakeup_module_set_record_buffer(char *buffer, unsigned int len)
{
	return m_ops->set_record_buffer(buffer, len);
}
EXPORT_SYMBOL(wakeup_module_set_record_buffer);

int wakeup_module_get_record_buffer(void)
{
	return m_ops->get_record_buffer();
}
EXPORT_SYMBOL(wakeup_module_get_record_buffer);

int wakeup_module_get_record_buffer_len(void)
{
	return m_ops->get_record_buffer_len();
}
EXPORT_SYMBOL(wakeup_module_get_record_buffer_len);

int wakeup_module_set_desc_addr(char * addr, unsigned int len)
{
	return m_ops->set_desc_addr(addr, len);
}
EXPORT_SYMBOL(wakeup_module_set_desc_addr);

#ifdef CONFIG_RECORD_DATA_TO_DDR
static unsigned char *record_buffer;
static dma_addr_t record_dma_addr;
#define RECORD_DATA_SIZE 	(1 * PAGE_SIZE)

static unsigned char *desc_addr;
static dma_addr_t desc_dma_addr;
#define DMA_DESC_SIZE		(1 * PAGE_SIZE)

#endif
static int __init wakeup_module_init(void)
{
	/* load voice wakeup firmware */
	memcpy((void*)FIRMWARE_LOAD_ADDRESS, wakeup_firmware, sizeof(wakeup_firmware));
	dma_cache_sync(NULL, (void*)FIRMWARE_LOAD_ADDRESS, sizeof(wakeup_firmware), DMA_TO_DEVICE);
	setup_ops();



	m_ops->_module_init();
	m_ops->set_dma_channel(JZDMA_REQ_I2S1 + 1);  /* dma phy id 5 */

#ifdef CONFIG_RECORD_DATA_TO_DDR
	/* record data to ddr instead of tcsm as default. */

	record_buffer = dma_alloc_noncoherent(NULL, RECORD_DATA_SIZE, &record_dma_addr, DMA_FROM_DEVICE);

	if(record_buffer == NULL) {
		printk("error allocat buffer for dmic recorder!\n");
		return -EFAULT;
	}
	printk("record buffer addr: %x, dma addr:%x\n", record_buffer, record_dma_addr);

	wakeup_module_set_record_buffer(record_buffer, RECORD_DATA_SIZE);

	desc_addr = dma_alloc_coherent(NULL, DMA_DESC_SIZE, &desc_dma_addr, DMA_FROM_DEVICE);
	if(desc_addr == NULL) {
		printk("failed to set desc addr!\n");
		return -EFAULT;
	}
	wakeup_module_set_desc_addr(desc_addr, DMA_DESC_SIZE);

#endif

	return 0;
}
static void __exit wakeup_module_exit(void)
{
	m_ops->_module_exit();

#ifdef CONFIG_RECORD_DATA_TO_DDR
	dma_free_noncoherent(NULL, RECORD_DATA_SIZE, record_buffer, record_dma_addr);

	dma_free_coherent(NULL, DMA_DESC_SIZE, desc_addr, desc_dma_addr);
#endif

}

module_init(wakeup_module_init);
module_exit(wakeup_module_exit);

MODULE_AUTHOR("qipengzhen<aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION("voice wakeup module driver");
MODULE_LICENSE("GPL");
