
#include <osl.h>
#include "jz_6181_bsp.h"
#include <linux/gpio.h>

#ifdef CUSTOMER_HW

#ifdef CONFIG_MACH_ODROID_4210
#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>

#include <plat/sdhci.h>
#include <plat/devs.h>		// modifed plat-samsung/dev-hsmmcX.c EXPORT_SYMBOL(s3c_device_hsmmcx) added

#define	sdmmc_channel	s3c_device_hsmmc0
#endif

#ifdef CUSTOMER_OOB
int bcm_wlan_get_oob_irq(void)
{
	int host_oob_irq = 0;
	int err = 0;
	host_oob_irq = get_wifi_board_irq_num();
	err = gpio_request(host_oob_irq, "bcmdhd_wlan_irq");
	gpio_direction_input(host_oob_irq);
	host_oob_irq = gpio_to_irq(host_oob_irq);
#ifdef CONFIG_MACH_ODROID_4210
	printk("GPIO(WL_HOST_WAKE) = EXYNOS4_GPX0(7) = %d\n", EXYNOS4_GPX0(7));
	host_oob_irq = gpio_to_irq(EXYNOS4_GPX0(7));
	gpio_direction_input(EXYNOS4_GPX0(7));
#endif
	printk("host_oob_irq: %d \r\n", host_oob_irq);

	return host_oob_irq;
}
#endif

//void bcm_wlan_power_on(int flag)
//{
//      if (flag == 1) {
//              printk("======== PULL WL_REG_ON HIGH! ========\n");
//#ifdef CONFIG_MACH_ODROID_4210
//              gpio_set_value(EXYNOS4_GPK1(0), 1);
//              /* Lets customer power to get stable */
//              mdelay(100);
//              printk("======== Card detection to detect SDIO card! ========\n");
//              sdhci_s3c_force_presence_change(&sdmmc_channel, 1);
//#endif
//      } else {
//              printk("======== PULL WL_REG_ON HIGH! (flag = %d) ========\n", flag);
//#ifdef CONFIG_MACH_ODROID_4210
//              gpio_set_value(EXYNOS4_GPK1(0), 1);
//#endif
//      }
//}
//
//void bcm_wlan_power_off(int flag)
//{
//      if (flag == 1) {
//              printk("======== Card detection to remove SDIO card! ========\n");
//#ifdef CONFIG_MACH_ODROID_4210
//              sdhci_s3c_force_presence_change(&sdmmc_channel, 0);
//              mdelay(100);
//              printk("======== PULL WL_REG_ON LOW! ========\n");
//              gpio_set_value(EXYNOS4_GPK1(0), 0);
//#endif
//      } else {
//              printk("======== PULL WL_REG_ON LOW! (flag = %d) ========\n", flag);
//#ifdef CONFIG_MACH_ODROID_4210
//              gpio_set_value(EXYNOS4_GPK1(0), 0);
//#endif
//      }
//}

#endif /* CUSTOMER_HW */
