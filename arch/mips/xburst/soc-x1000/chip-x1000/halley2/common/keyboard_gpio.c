#include <linux/platform_device.h>

#include <linux/gpio_keys.h>
#include <linux/input.h>
#include "board_base.h"

struct gpio_keys_button __attribute__((weak)) board_buttons[] = {
#ifdef GPIO_HOME_KEY
{
		.gpio		= GPIO_HOME_KEY,
		.code   	= KEY_HOME,
		.code_long_press = KEY_F9,
		.desc		= "home key",
		.active_low	= ACTIVE_LOW_HOME,
	},
#endif
#ifdef GPIO_ENTER_KEY
{
		.gpio		= GPIO_ENTER_KEY,
		.code   	= KEY_ENTER,
		.code_long_press = KEY_F10,
		.desc		= "menu key",
		.active_low	= ACTIVE_LOW_ENTER,
	},
#endif
#ifdef GPIO_BACK_KEY
{
		.gpio		= GPIO_BACK_KEY,
		.code   	= KEY_BACKSPACE,
		.code_long_press = KEY_NUMLOCK,
		.desc		= "back key",
		.active_low	= ACTIVE_LOW_BACK,
	},
#endif
#ifdef GPIO_VOLUMEDOWN_KEY
{
		.gpio		= GPIO_VOLUMEDOWN_KEY,
		.code   	= KEY_DOWN,
		 .code_long_press = KEY_F4,
		.desc		= "volum down key",
		.active_low	= ACTIVE_LOW_VOLUMEDOWN,
	},
#endif
#ifdef GPIO_VOLUMEUP_KEY
{
		.gpio		= GPIO_VOLUMEUP_KEY,
		.code   	= KEY_UP,
		.code_long_press = KEY_F5,
		.desc		= "volum up key",
		.active_low	= ACTIVE_LOW_VOLUMEUP,
	},
#endif

#ifdef GPIO_ENDCALL_KEY
{
		.gpio           = GPIO_ENDCALL_KEY,
		.code           = KEY_POWER,
		.code_long_press = KEY_F8,
		.desc           = "end call key",
		.active_low     = ACTIVE_LOW_ENDCALL,
		.wakeup         = 1,
	},
#endif
#ifdef CONFIG_BOARD_HAS_NO_DETE_FACILITY
#ifdef GPIO_USB_DETE
{
		.gpio           = GPIO_USB_DETE,
        .code           = KEY_F13,
        .desc           = "usb detect",
        .active_low     = ACTIVE_LOW_ENDCALL,
        .wakeup         = 1,
},
#endif
#endif
#ifdef GPIO_PRE_KEY
{
		.gpio		= GPIO_PRE_KEY,
		.code   	= KEY_LEFT,//KEY_PREVIOUSSONG,
		.code_long_press = KEY_F6,
		.desc		= "pre song key",
		.active_low	= ACTIVE_LOW_PRE,
	},
#endif
#ifdef GPIO_NEXT_KEY
{
		.gpio		= GPIO_NEXT_KEY,
		.code   	= KEY_RIGHT,//KEY_NEXTSONG,
		.code_long_press = KEY_F7,
		.desc		= "next song key",
		.active_low	= ACTIVE_LOW_NEXT,
	},
#endif
#ifdef GPIO_LEFT_KEY
{
		.gpio		= GPIO_LEFT_KEY,
		.code   	= KEY_F1,//KEY_PREVIOUSSONG,
		//.code_long_press= KEY_F11,
		.desc		= "left key",
		.active_low	= ACTIVE_LOW_LEFT,
	},
#endif
#ifdef GPIO_RIGHT_KEY
{
		.gpio		= GPIO_RIGHT_KEY,
		.code   	= KEY_F2,//KEY_NEXTSONG,
		.desc		= "right key",
		.active_low	= ACTIVE_LOW_RIGHT,
	},
#endif
};

static struct gpio_keys_platform_data board_button_data = {
	.buttons	= board_buttons,
	.nbuttons	= ARRAY_SIZE(board_buttons),
};

struct platform_device jz_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
                .platform_data	= &board_button_data,
	}
};
