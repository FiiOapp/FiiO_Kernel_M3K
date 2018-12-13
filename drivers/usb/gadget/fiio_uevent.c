#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_device.h> 
#include <linux/delay.h>
#include <mach/platform.h>

//debug
//#define FIIO_DEBUG_TP
#ifdef FIIO_DEBUG_TP
#define fiio_debug(x...)  printk(KERN_INFO "[fiio_uevent] " x)
#else
#define fiio_debug(x...)
#endif
static struct class *remotectl_event_class;
static struct device *fiio_remotectl_event_device;
static struct kobject *fiio_remotectl_event_object;


static int __init remotectl_event_init(void)
{
	int err;
	remotectl_event_class = class_create(THIS_MODULE,"fiio");
	err = PTR_ERR(remotectl_event_class);
	if (IS_ERR(remotectl_event_class)) {
		printk("create fiio class !\n");
		goto out;
	}
	fiio_remotectl_event_device = device_create(remotectl_event_class,NULL,MKDEV(0,0),NULL,"uevent");
	err = PTR_ERR(fiio_remotectl_event_device);
	if (IS_ERR(fiio_remotectl_event_device)) {
		printk("create fiio uevent !\n");
		goto outclass;
	}
	fiio_remotectl_event_object = &fiio_remotectl_event_device->kobj;
	
	return 0;
outclass:
	class_destroy(remotectl_event_class);
out:
	return err;
}
/*
 *remove
 */
static void __exit remotectl_event_exit(void) {
	device_destroy(remotectl_event_class,MKDEV(0,0));
	class_destroy(remotectl_event_class);
}

/*
* name:fiio_remotectl_event_send
* function:send uevent
* params
* uevent_type:
* sprintf(spectator, "DEVPATH=%s", "/sys/devices/virtual/fiio/uevent");
* sprintf(subcode, "%s=%d", uevent_name,uevent);
**/
void fiio_remotectl_event_send(void) 
{
	char spectator[50];
	char subcode[50];
	char *envp[] = {spectator,subcode, NULL };
	sprintf(spectator, "DEVPATH=%s", "/sys/devices/virtual/fiio/uevent");
	sprintf(subcode, "%s=%d", "USB_RESET_STATE",1);
	kobject_uevent_env(fiio_remotectl_event_object, KOBJ_CHANGE, envp);
	printk("fiio reset usb fail,send uevent to app to restart dwc2!\n");
}
EXPORT_SYMBOL_GPL(fiio_remotectl_event_send);

module_init(remotectl_event_init);
module_exit(remotectl_event_exit);

MODULE_AUTHOR("pengweizhong <pengweizhong@fiio.net>");
MODULE_DESCRIPTION("driver for fiio uevent");
MODULE_LICENSE("GPL");
