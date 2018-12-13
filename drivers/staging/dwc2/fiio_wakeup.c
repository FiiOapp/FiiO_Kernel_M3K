/*
 * jz4780_platform.c - JZ4780 DWC2 controller platform driver
 *
 * Copyright (C) Matthijs Kooijman <matthijs@stdin.nl>
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#define DEBUG 1

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/sound.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#include <linux/regulator/machine.h>

//#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>

//debug
//#define FIIO_DEBUG_WAKEUP
#ifdef FIIO_DEBUG_WAKEUP
#define fiio_debug(x...)  printk(KERN_INFO "[fiio_wamkeup] " x)
#else
#define fiio_debug(x...)
#endif

#define  GPIO_USB_DETE GPIO_PB(11)

static int g_fiio_dwc_suspend_flag = 0;
static int fiio_irq = 0;

#define DELAY_WORK_JIFFIES        (10)//ms
static struct delayed_work fiio_wakeup_work;
static struct workqueue_struct *fiio_wakeup_workqueue = NULL;
static irqreturn_t process_isr(int irq, void *dev_id)
{
	queue_delayed_work(fiio_wakeup_workqueue,&fiio_wakeup_work, DELAY_WORK_JIFFIES);
	return IRQ_HANDLED;
}

static void fiio_wakeup_func(struct work_struct *work)
{
	fiio_debug("Enter %s !\n",__func__);
	int queue_player_state = gpio_get_value(GPIO_USB_DETE);
	fiio_debug("++++%s queue_player_state:%d\n",__func__,queue_player_state);
}


static int fiio_wakeup_probe(struct platform_device *pdev) {
	
	int error = 0,irq;
	printk("Enter %s !\n",__func__);
	if (gpio_is_valid(GPIO_USB_DETE)) {
		
	   error = gpio_request_one(GPIO_USB_DETE, GPIOF_IN, "fiio_wakup");
	   if (error < 0) {
		   printk("Failed to request GPIO %d, error %d\n",
			   GPIO_USB_DETE, error);
		   return error;
	   }
	   irq = gpio_to_irq(GPIO_USB_DETE);
	   if (irq < 0) {
			error = irq;
			printk("Unable to get irq number for GPIO %d, error %d\n",
				GPIO_USB_DETE, error);
			goto fail;
		}
	}
	else {
		printk("%s %d key=%d can't use!\n",__func__,__LINE__,GPIO_USB_DETE);
	}
	
	INIT_DELAYED_WORK(&fiio_wakeup_work, fiio_wakeup_func);
	fiio_wakeup_workqueue = create_singlethread_workqueue("fiio_wakeup");

    error = request_any_context_irq(irq, process_isr,  (IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING), pdev->name, NULL);
    if (error < 0) {
        printk("Unable to claim irq %d; error %d\n",irq, error);
        goto fail;
    }
	
	device_init_wakeup(&pdev->dev, 1);
	
	printk("end %s !\n",__func__);
	return 0;
	fail:
		if (gpio_is_valid(GPIO_USB_DETE))
			gpio_free(GPIO_USB_DETE);
	return 0;
}

static int fiio_wakeup_remove(struct platform_device *pdev)
{
	printk("Enter %s !\n",__func__);
	
	device_init_wakeup(&pdev->dev, 0);
	
	free_irq(gpio_to_irq(GPIO_USB_DETE),NULL);
	
	if (gpio_is_valid(GPIO_USB_DETE)) {
		printk("Enter %s free gpio!\n",__func__);
		gpio_free(GPIO_USB_DETE);
	}
		
	//
	//cancel_work_sync(&fiio_wakeup_work);
	cancel_delayed_work_sync(&fiio_wakeup_work);
	
	printk("end %s !\n",__func__);
	return 0;
}



static int fiio_wakeup_suspend(struct device *dev)
{
	printk("Enter %s !\n",__func__);
	if (0 == g_fiio_dwc_suspend_flag) {
		g_fiio_dwc_suspend_flag = 1;
		if (device_may_wakeup(dev)) {
			if(gpio_is_valid(GPIO_USB_DETE)) {
				printk("%s %d key=%d irq=%d\n",__func__,__LINE__,GPIO_USB_DETE,gpio_to_irq(GPIO_USB_DETE));
				enable_irq_wake(gpio_to_irq(GPIO_USB_DETE));
			}
			else {
				printk("ERROR %s %d\n",__func__,__LINE__);
			}
		}
		else {
			printk("ERROR %s %d\n",__func__,__LINE__);
		}
		
	}
	
    return 0;
}
static int fiio_wakeup_resume(struct device *dev)
{
	printk("Enter %s !\n",__func__);
	if (1 == g_fiio_dwc_suspend_flag) {
		g_fiio_dwc_suspend_flag = 0;
		if (device_may_wakeup(dev)) {
			if(gpio_is_valid(GPIO_USB_DETE)) {
				fiio_debug("%s %d key=%d irq=%d\n",__func__,__LINE__,GPIO_USB_DETE,gpio_to_irq(GPIO_USB_DETE));
				disable_irq_wake(gpio_to_irq(GPIO_USB_DETE));
			}
			else {
				printk("ERROR %s %d\n",__func__,__LINE__);
			}
		}
		else {
			printk("ERROR %s %d\n",__func__,__LINE__);
		}
		
	}
    return 0;
}
static SIMPLE_DEV_PM_OPS(fiio_wakeup_pm_ops, fiio_wakeup_suspend, fiio_wakeup_resume);
static struct of_device_id fiio_wakeup_of_match[] = {
	{ .compatible = "fiio_wakeup", },
	{ },
};


static struct platform_driver fiio_platform_driver = {
	.probe		= fiio_wakeup_probe,
	.remove		= fiio_wakeup_remove,
	.driver		= {
		.name	= "fiio_wakeup",
		.owner	= THIS_MODULE,
		.pm	= &fiio_wakeup_pm_ops,
		.of_match_table = of_match_ptr(fiio_wakeup_of_match),
	},
};

static void fiio_wakeup_release(struct device *dev)
{
	printk("releasing '%s'\n", dev_name(dev));
}


struct platform_device  fiio_platform_device = {
	.name = "fiio_wakeup",
	.id = -1,
	.dev = {
		.release = fiio_wakeup_release,
	},
};

static int __init fiio_module_init(void) {
	int err = 0;
	printk("Enter %s !\n",__func__);
	err = platform_driver_register(&fiio_platform_driver);
	if (err) {
		printk("%s platform_driver_register error!\n",__func__);
		return err;
	}
		

	/* Register snd_uac2 device */
	err = platform_device_register(&fiio_platform_device);
	if (err) {
		printk("%s platform_device_register error!\n",__func__);
		platform_driver_unregister(&fiio_platform_driver);
	}
	printk("end %s !\n",__func__);
	return 0;
}

static void __exit fiio_module_exit(void)
{
	printk("Enter %s !\n",__func__);

	platform_driver_unregister(&fiio_platform_driver);
	platform_device_unregister(&fiio_platform_device);
	printk("end %s !\n",__func__);
}

module_init(fiio_module_init);
module_exit(fiio_module_exit);


MODULE_DESCRIPTION("FiiO Wakeup controller platform driver");
MODULE_AUTHOR("pengweizhong<pengweizhong@fiio.net>");
MODULE_LICENSE("GPL");

