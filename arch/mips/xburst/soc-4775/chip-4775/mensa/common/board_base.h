#ifndef __BOARD_BASE_H__
#define __BOARD_BASE_H__

#include <board.h>
#include <linux/spi/spi.h>

#ifdef CONFIG_LEDS_GPIO
extern struct platform_device jz_led_rgb;
#endif

#ifdef CONFIG_JZ4775_INTERNAL_CODEC
extern struct snd_codec_data codec_data;
#endif

#ifdef CONFIG_JZMMC_V11_MMC0
extern struct jzmmc_platform_data inand_pdata;
#endif
#ifdef CONFIG_JZMMC_V11_MMC2
extern struct jzmmc_platform_data tf_pdata;
#endif
#ifdef CONFIG_JZMMC_V11_MMC1
extern struct jzmmc_platform_data sdio_pdata;
#endif

#ifdef CONFIG_BROADCOM_RFKILL
extern struct platform_device	bt_power_device;
extern struct platform_device	bluesleep_device;
#endif

#ifdef CONFIG_USB_DWC2
extern struct platform_device   jz_dwc_otg_device;
#endif
#ifdef CONFIG_USB_OHCI_HCD
extern struct platform_device jz_ohci_device;
#endif

#ifdef CONFIG_SPI0_JZ47XX
extern struct jz47xx_spi_info spi0_info_cfg;
#endif
#ifdef CONFIG_JZ_SPI_NOR
extern struct spi_board_info jz_spi0_board_info[];
extern int jz_spi0_devs_size;
#endif
#ifdef CONFIG_SPI_GPIO
extern struct platform_device jz47xx_spi_gpio_device;
#endif

#ifdef CONFIG_JZ_EFUSE_V11
extern struct jz_efuse_platform_data jz_efuse_pdata;
#endif

#ifdef CONFIG_I2C0_V12_JZ
extern struct platform_device jz_i2c0_device;
#endif
#ifdef CONFIG_I2C1_V12_JZ
extern struct platform_device jz_i2c1_device;
#endif
#ifdef CONFIG_I2C2_V12_JZ
extern struct platform_device jz_i2c2_device;
#endif
#ifdef CONFIG_I2C3_V12_JZ
extern struct platform_device jz_i2c3_device;
#endif
#ifdef CONFIG_MFD_JZ_SADC_V12
extern struct platform_device jz_adc_device;
#endif

#ifdef CONFIG_JZ_MAC
extern struct platform_device jz_mii_bus;
extern struct platform_device jz_mac_device;
#endif

#if defined(CONFIG_SND_ASOC_JZ_AIC_V12)
extern struct platform_device snd_alsa_device;
#endif

#ifdef CONFIG_FB_JZ_V11
extern struct jzfb_platform_data jzfb0_pdata;
#endif

#ifdef CONFIG_LCD_BYD_8991FTGF
extern struct platform_device byd_8991_device;
#endif

#ifdef CONFIG_LCD_KFM701A21_1A
extern struct platform_device kfm701a21_1a_device;
#endif

#ifdef CONFIG_MTD_NAND_JZ
extern struct platform_device jz_mtd_nand_device;
#endif
#ifdef CONFIG_MTD_NAND_JZ_NORMAL
extern struct xburst_nand_chip_platform_data nand_chip_data;
#endif
#ifdef CONFIG_BCM4330_RFKILL
extern struct platform_device bcm4330_bt_power_device;
#endif
#ifdef CONFIG_BACKLIGHT_DIGITAL_PULSE
extern struct platform_device digital_pulse_backlight_device;
extern struct platform_digital_pulse_backlight_data bl_data;
#endif
#ifdef CONFIG_FB_JZ4775_ANDROID_EPD
extern struct platform_device jz_epdce_device;
extern struct platform_device jz_epd_device;
extern struct jz_epd_platform_data jz_epd_pdata;
#endif

#if (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C0_V12_JZ))
extern struct i2c_board_info jz_i2c0_devs[];
extern int jz_i2c0_devs_size;
#endif
#if (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C1_V12_JZ))
extern struct i2c_board_info jz_i2c1_devs[];
extern int jz_i2c1_devs_size;
#endif
#if (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C2_V12_JZ))
extern struct i2c_board_info jz_i2c2_devs[];
extern int jz_i2c2_devs_size;
#endif
#if (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C3_V12_JZ))
extern struct i2c_board_info jz_i2c3_devs[];
extern int jz_i2c3_devs_size;
#endif
#ifdef CONFIG_I2C_GPIO
#ifndef CONFIG_I2C0_V12_JZ
extern struct platform_device i2c0_gpio_device;
#endif
#ifndef CONFIG_I2C1_V12_JZ
extern struct platform_device i2c1_gpio_device;
#endif
#ifndef CONFIG_I2C2_V12_JZ
extern struct platform_device i2c2_gpio_device;
#endif
#ifndef CONFIG_I2C3_V12_JZ
extern struct platform_device i2c3_gpio_device;
#endif
#endif
#endif
