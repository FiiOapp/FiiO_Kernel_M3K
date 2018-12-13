#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/wlan_plat.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include "board_base.h"

static char *wifi_mac_str = NULL;
static struct wifi_platform_data bcmdhd_wlan_pdata;

static int get_wifi_mac_addr(unsigned char* buf)
{
	int i = 0;
	int num = 0;
	char *tmp_addr = NULL;

	if(wifi_mac_str == NULL) {
		printk("%s wifi_mac_str == NULL, please checked!\n", __func__);
		return -1;
	}
	tmp_addr = kmalloc(3, GFP_KERNEL);
	for (num = 0; num <= 10; num = num + 2) {
		memcpy(tmp_addr, wifi_mac_str + num, 2);
		buf[i] = (char)simple_strtol(tmp_addr, NULL, 16);
		i++;
	}
	printk("buf = %02x:%02x:%02x:%02x:%02x:%02x \n",
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
	return 0;
}


static struct resource wlan_resources[] = {
	[0] = {
		.start = GPIO_WIFI_WAKE,
		.end = GPIO_WIFI_WAKE,
		.name = "bcmdhd_wlan_irq",
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct resource ap6181_wlan_resources[] = {
	[0] = {
		.start = GPIO_WIFI_WAKE,
		.end = GPIO_WIFI_WAKE,
		.name = "ap6181_bcmdhd_wlan_irq",
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device wlan_device = {
	.name   = "bcmdhd_wlan",
	.id     = 1,
	.dev    = {
		.platform_data = &bcmdhd_wlan_pdata,
	},
	.resource	= wlan_resources,
	.num_resources	= ARRAY_SIZE(wlan_resources),
};

static struct platform_device ap6181_wlan_device = {
	.name   = "ap6181_bcmdhd_wlan",
	.id     = 1,
	.dev    = {
		.platform_data = NULL,
	},
	.resource	= ap6181_wlan_resources,
	.num_resources	= ARRAY_SIZE(ap6181_wlan_resources),
};

static int __init get_wifi_mac_addr_from_cmdline(char *str)
{
	wifi_mac_str = str;
	return 1;
}

__setup("wifi_mac=", get_wifi_mac_addr_from_cmdline);

static int __init wlan_device_init(void)
{
	int ret;
	memset(&bcmdhd_wlan_pdata, 0, sizeof(bcmdhd_wlan_pdata));

	bcmdhd_wlan_pdata.get_mac_addr = get_wifi_mac_addr;

	if (wifi_mac_str == NULL) {
		wlan_device.dev.platform_data = NULL;
	}
	ret = platform_device_register(&wlan_device);
	ret = platform_device_register(&ap6181_wlan_device);
	if(ret){
		printk("ap6181_wlan_dev register failed\n");
		return ret;
	}
	return ret;
}

late_initcall(wlan_device_init);
//MODULE_DESCRIPTION("Broadcomm wlan driver");
//MODULE_LICENSE("GPL");
