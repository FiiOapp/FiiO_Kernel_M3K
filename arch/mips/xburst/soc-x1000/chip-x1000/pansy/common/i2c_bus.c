#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/interrupt.h>
#include "board_base.h"
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/jzsnd.h>

//extern struct platform_device jz_i2c2_device;
#ifdef CONFIG_WM8594_CODEC_V12
static struct snd_codec_data wm8594_codec_pdata = {
	.codec_sys_clk = 1200000,

};
#endif

#if (defined(CONFIG_SOFT_I2C0_GPIO_V12_JZ) || defined(CONFIG_I2C0_V12_JZ))
struct i2c_board_info jz_i2c0_devs[] __initdata = {

};
int jz_i2c0_devs_size = ARRAY_SIZE(jz_i2c0_devs);

struct i2c_board_info jz_v4l2_camera_devs[] __initdata = {
};
int jz_v4l2_devs_size = ARRAY_SIZE(jz_v4l2_camera_devs);
#endif

#if (defined(CONFIG_SOFT_I2C2_GPIO_V12_JZ) || defined(CONFIG_I2C2_V12_JZ))
struct i2c_board_info jz_i2c2_devs[] __initdata = {
};
#endif

#if defined(CONFIG_SOFT_I2C2_GPIO_V12_JZ) || defined(CONFIG_I2C2_V12_JZ)
int jz_i2c2_devs_size = ARRAY_SIZE(jz_i2c2_devs);
#endif

#ifdef CONFIG_I2C_GPIO
#define DEF_GPIO_I2C(NO)						\
	static struct i2c_gpio_platform_data i2c##NO##_gpio_data = {    \
		.sda_pin    = GPIO_I2C##NO##_SDA,			\
		.scl_pin    = GPIO_I2C##NO##_SCK,			\
	};								\
	struct platform_device i2c##NO##_gpio_device = {		\
		.name   = "i2c-gpio",					\
		.id = NO,						\
		.dev    = { .platform_data = &i2c##NO##_gpio_data,},    \
	};
#ifdef CONFIG_SOFT_I2C1_GPIO_V12_JZ
DEF_GPIO_I2C(1);
#endif
#ifdef CONFIG_SOFT_I2C0_GPIO_V12_JZ
DEF_GPIO_I2C(0);
#endif
#endif /*CONFIG_I2C_GPIO*/
