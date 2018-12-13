/*
 * Copyright (c) 2017 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * JZ-X1000 orion board lcd setup routines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/lcd.h>
#include <linux/regulator/consumer.h>
#include <mach/jzfb.h>
#include "../board_base.h"

struct kd035hvfmd057_power{
	struct regulator *vlcdio;
	struct regulator *vlcdvcc;
	int inited;
};

static struct kd035hvfmd057_power lcd_power = {
	NULL,
    NULL,
    0
};

int kd035hvfmd057_power_init(struct lcd_device *ld)
{
    int ret ;
    printk("======kd035hvfmd057_power_init==============\n");
    if(GPIO_LCD_RST > 0){
        ret = gpio_request(GPIO_LCD_RST, "lcd rst");
        if (ret) {
            printk(KERN_ERR "can's request lcd rst\n");
            return ret;
        }
    }
    if(GPIO_LCD_CS > 0){
        ret = gpio_request(GPIO_LCD_CS, "lcd cs");
        if (ret) {
            printk(KERN_ERR "can's request lcd cs\n");
            return ret;
        }
    }
    if(GPIO_LCD_RD > 0){
        ret = gpio_request(GPIO_LCD_RD, "lcd rd");
        if (ret) {
            printk(KERN_ERR "can's request lcd rd\n");
            return ret;
        }
    }
    printk("set lcd_power.inited  =======1 \n");
    lcd_power.inited = 1;
    return 0;
}
int kd035hvfmd057_power_reset(struct lcd_device *ld)
{
	if (!lcd_power.inited)
		return -EFAULT;
	gpio_direction_output(GPIO_LCD_RST, 0);
	mdelay(20);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(10);

	return 0;
}

int kd035hvfmd057_power_on(struct lcd_device *ld, int enable)
{
	if (!lcd_power.inited && kd035hvfmd057_power_init(ld))
		return -EFAULT;

	if (enable) {
		gpio_direction_output(GPIO_LCD_CS, 1);
		gpio_direction_output(GPIO_LCD_RD, 1);

		kd035hvfmd057_power_reset(ld);

		mdelay(5);
		gpio_direction_output(GPIO_LCD_CS, 0);

	} else {
		mdelay(5);
		gpio_direction_output(GPIO_LCD_CS, 0);
		gpio_direction_output(GPIO_LCD_RD, 0);
		gpio_direction_output(GPIO_LCD_RST, 0);
	}
	return 0;
}

struct lcd_platform_data kd035hvfmd057_pdata = {
	.reset    = kd035hvfmd057_power_reset,
	.power_on = kd035hvfmd057_power_on,
};

/* LCD Panel Device */
struct platform_device kd035hvfmd057_device = {
	.name		= "kd035hvfmd057_slcd",
	.dev		= {
		.platform_data	= &kd035hvfmd057_pdata,
	},
};
static struct smart_lcd_data_table kd035hvfmd057_data_table[] = {
    /* LCD init code */
    {SMART_CONFIG_CMD, 0xE0}, //P-Gamma
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x13},
    {SMART_CONFIG_DATA, 0x18},
    {SMART_CONFIG_DATA, 0x04},
    {SMART_CONFIG_DATA, 0x0F},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x3A},
    {SMART_CONFIG_DATA, 0x56},
    {SMART_CONFIG_DATA, 0x4D},
    {SMART_CONFIG_DATA, 0x03},
    {SMART_CONFIG_DATA, 0x0A},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x30},
    {SMART_CONFIG_DATA, 0x3E},
    {SMART_CONFIG_DATA, 0x0F},

    {SMART_CONFIG_CMD, 0XE1}, //N-Gamma
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x13},
    {SMART_CONFIG_DATA, 0x18},
    {SMART_CONFIG_DATA, 0x01},
    {SMART_CONFIG_DATA, 0x11},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x38},
    {SMART_CONFIG_DATA, 0x34},
    {SMART_CONFIG_DATA, 0x4D},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x0D},
    {SMART_CONFIG_DATA, 0x0B},
    {SMART_CONFIG_DATA, 0x31},
    {SMART_CONFIG_DATA, 0x37},
    {SMART_CONFIG_DATA, 0x0F},

    {SMART_CONFIG_CMD, 0XC0},   //Power Control 1
    {SMART_CONFIG_DATA, 0x18}, //Vreg1out
    {SMART_CONFIG_DATA, 0x17}, //Verg2out

    {SMART_CONFIG_CMD, 0xC1},   //Power Control 2
    {SMART_CONFIG_DATA, 0x41}, //VGH,VGL

    {SMART_CONFIG_CMD, 0xC5},   //Power Control 3
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x1A}, //Vcom
    {SMART_CONFIG_DATA, 0x80},

    {SMART_CONFIG_CMD, 0x36},   //Memory Access
    {SMART_CONFIG_DATA, 0x48},   //48

    {SMART_CONFIG_CMD, 0x3A},   // Interface Pixel Format
    {SMART_CONFIG_DATA, 0x55}, //16bit

    {SMART_CONFIG_CMD, 0XB0},   // Interface Mode Control
    {SMART_CONFIG_DATA, 0x00},

    {SMART_CONFIG_CMD, 0xB1},   //Frame rate
    {SMART_CONFIG_DATA, 0xA0}, //60Hz

    {SMART_CONFIG_CMD, 0xB4},   //Display Inversion Control
    {SMART_CONFIG_DATA, 0x02}, //2-dot

    {SMART_CONFIG_CMD, 0XB6},   //RGB/MCU Interface Control
    {SMART_CONFIG_DATA, 0x02}, //MCU RGB
    {SMART_CONFIG_DATA, 0x02}, //Source,Gate scan dieection

    {SMART_CONFIG_CMD, 0XE9},    // Set Image Function
    {SMART_CONFIG_DATA, 0x00},  //disable 24 bit data input

    {SMART_CONFIG_CMD, 0xF7},    // Adjust Control
    {SMART_CONFIG_DATA, 0xA9},
    {SMART_CONFIG_DATA, 0x51},
    {SMART_CONFIG_DATA, 0x2C},
    {SMART_CONFIG_DATA, 0x82},  // D7 stream, loose

    {SMART_CONFIG_CMD, 0x21},    //Normal Black
    {SMART_CONFIG_CMD, 0x11},    //Sleep out
    {SMART_CONFIG_UDELAY, 120000},
    {SMART_CONFIG_CMD, 0x29},
    {SMART_CONFIG_CMD, 0x2C},
};

unsigned long kd035hvfmd057_cmd_buf[]= {
	0x2C2C2C2C,
};

struct fb_videomode jzfb0_videomode = {
	.name = "320x480",
	.refresh = 60,
	.xres = 320,
	.yres = 480,
	.pixclock = KHZ2PICOS(18432),
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};


struct jzfb_platform_data jzfb_pdata = {
	.num_modes = 1,
	.modes = &jzfb0_videomode,
	.lcd_type = LCD_TYPE_SLCD,
	.bpp    = 16,
	.width = 49,
	.height = 74,
	.pinmd  = 0,

	.smart_config.rsply_cmd_high       = 0,
	.smart_config.csply_active_high    = 0,
	.smart_config.newcfg_fmt_conv =  1,
	/* write graphic ram command, in word, for example 8-bit bus, write_gram_cmd=C3C2C1C0. */
	.smart_config.write_gram_cmd = kd035hvfmd057_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(kd035hvfmd057_cmd_buf),
	.smart_config.bus_width = 8,
        .smart_config.length_data_table =  ARRAY_SIZE(kd035hvfmd057_data_table),
        .smart_config.data_table = kd035hvfmd057_data_table,
	.dither_enable = 0,
};
/**************************************************************************************************/
#ifdef CONFIG_BACKLIGHT_PWM

static int backlight_init(struct device *dev)
{
	int ret;
#if 0
	ret = gpio_request(GPIO_LCD_PWM, "Backlight");
	if (ret) {
		printk(KERN_ERR "failed to request GPF for PWM-OUT1\n");
		return ret;
	}
#endif
	printk("------------------------------------------------");
	ret = gpio_request(GPIO_BL_PWR_EN, "BL PWR");
	if (ret) {
		printk(KERN_ERR "failed to reqeust BL PWR\n");
		return ret;
	}
	gpio_direction_output(GPIO_BL_PWR_EN, 1);
	return 0;
}

#define MAX_BRIGHTNESS_STEP	16				/* SGM3146 supports 16 brightness step */
#define CONVERT_FACTOR		(256/MAX_BRIGHTNESS_STEP)	/* System support 256 brightness step */

#if 1
static void send_low_pulse(int num)
{
        unsigned int i;

        gpio_direction_output(GPIO_BL_PWR_EN,0);
        udelay(5000);
        gpio_direction_output(GPIO_BL_PWR_EN,1);
        udelay(100);

        for (i = num; i > 0; i--)	{
                gpio_direction_output(GPIO_BL_PWR_EN,0);
                udelay(1);
                gpio_direction_output(GPIO_BL_PWR_EN,1);
                udelay(3);
        }
}
#endif

static int backlight_notify(struct device *dev, int brightness)
{
        int pulse_num = MAX_BRIGHTNESS_STEP - brightness / CONVERT_FACTOR - 1;
        /* int pulse_num = MAX_BRIGHTNESS_STEP - MAX_BRIGHTNESS_STEP * brightness / 256 - 1; */
	if (brightness)
                send_low_pulse(pulse_num);
	else
		gpio_direction_output(GPIO_BL_PWR_EN, 0);

	return brightness;
}

static void backlight_exit(struct device *dev)
{
	gpio_free(GPIO_LCD_PWM);
}

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 120,
	.pwm_period_ns	= 30000,
	.init		= backlight_init,
	.exit		= backlight_exit,
	.notify		= backlight_notify,
};

struct platform_device backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.platform_data	= &backlight_data,
	},
};
#endif
