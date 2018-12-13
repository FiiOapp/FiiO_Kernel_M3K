#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>

static int wlan_irq_num = 0;

int get_wifi_board_irq_num(void)
{
	return wlan_irq_num;
}

static int wifi_probe(struct platform_device *pdev)
{
	int irq_num = 0;
	irq_num = platform_get_irq(pdev, 0);
	if (irq_num == NULL) {
		printk("platform wifi irq num get failed\n");
		return -1;
	} else {
		wlan_irq_num = irq_num;
	}
	return 0;
}

static int wifi_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id wifi_device_dt_match[] = {
	{.compatible = "android,ap6181_bcmdhd_wlan",},
	{},
};

static struct platform_driver wifi_irq_device = {
	.probe = wifi_probe,
	.remove = wifi_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		   .name = "ap6181_bcmdhd_wlan",
		   .of_match_table = wifi_device_dt_match,
		   }
};

int __init wifi_board_dri_reg(void)
{
	int irq_num = 0;
	int err = 0;
	err = platform_driver_register(&wifi_irq_device);
	if (err) {
		printk("platform driver wifi_irq_device register failed\n");
		return err;
	}
	return 0;
}

__initcall(wifi_board_dri_reg);
