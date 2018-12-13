#ifndef __BOARD_H__
#define __BOARD_H__
#include <gpio.h>
#include <soc/gpio.h>

#define FRONT_CAMERA_INDEX	0
#define BACK_CAMERA_INDEX	1

#define FRONT_CAMERA_SENSOR_RESET	GPIO_PA(27)
#define FRONT_CAMERA_SENSOR_EN		GPIO_PB(6)

#define BACK_CAMERA_SENSOR_RESET	-1/*GPIO_PB(2)*/
#define BACK_CAMERA_SENSOR_EN		-1/*GPIO_PE(28)*/

#ifndef CONFIG_BOARD_NAME
#define CONFIG_BOARD_NAME "mensa"
#endif

/* MSC GPIO Definition */
#define GPIO_SD0_VCC_EN_N	GPIO_PB(3)
#define GPIO_SD0_CD_N		GPIO_PB(2)

/*efuse-vddq*/
#define GPIO_EFUSE_VDDQ GPIO_PE(2)


/**
 * lcd gpio
 **/
#define GPIO_LCD_PWM		GPIO_PE(1)
#define GPIO_LCD_DISP		GPIO_PB(30)

/* EPD Power Pins */
#define GPIO_EPD_SIG_CTRL_N GPIO_PB(20)
#define GPIO_EPD_PWR_EN     GPIO_PB(06)
#define GPIO_EPD_PWR0       GPIO_PB(10)
#define GPIO_EPD_PWR1       GPIO_PB(11)
#define GPIO_EPD_PWR2       GPIO_PB(16)
#define GPIO_EPD_PWR3       GPIO_PB(17)
#define GPIO_EPD_PWR4       GPIO_PF(8)

/**
 * TP gpio
 **/
#define GPIO_TP_RESET		GPIO_PB(28)
#define GPIO_TP_INT		GPIO_PB(29)

#define GPIO_HP_MUTE		-1	/*hp mute gpio*/
#define GPIO_HP_MUTE_LEVEL		-1		/*vaild level*/

#define GPIO_SPEAKER_EN			GPIO_PG(14)/*speaker enable gpio*/
#define GPIO_SPEAKER_EN_LEVEL	1

#define GPIO_HANDSET_EN		  -1		/*handset enable gpio*/
#define GPIO_HANDSET_EN_LEVEL -1

#define	GPIO_HP_DETECT		GPIO_PA(17)	/*hp detect gpio*/
#define GPIO_HP_INSERT_LEVEL    1
#define GPIO_MIC_SELECT		-1		/*mic select gpio*/
#define GPIO_BUILDIN_MIC_LEVEL	-1		/*builin mic select level*/
#define GPIO_MIC_DETECT		-1
#define GPIO_MIC_INSERT_LEVEL -1
#define GPIO_MIC_DETECT_EN		-1  /*mic detect enable gpio*/
#define GPIO_MIC_DETECT_EN_LEVEL	-1 /*mic detect enable gpio*/

/**
 * CIM gpio
 **/
#define GPIO_OV3640_EN		GPIO_PB(6)
#define GPIO_OV3640_RST		GPIO_PA(27)

#define GPIO_OV5640_PWDN	GPIO_PB(6)
#define GPIO_OV5640_RST		GPIO_PA(27)

#define GPIO_OV2650_EN          GPIO_PB(6)
#define GPIO_OV2650_RST         GPIO_PA(27)

#define GPIO_OV7675_EN          GPIO_PD(20)
#define GPIO_OV7675_RST         GPIO_PA(27)

/**
 * KEY gpio
 **/
// For the layout on Mensa, key 'menu' is necessary. So replacing 'home' with 'menu'.
//#define GPIO_HOME		GPIO_PG(15)
#if defined(CONFIG_JZ_CIM1) || defined(CONFIG_VIDEO_JZ4780_CIM_HOST)
#define GPIO_MENU		(-1)
#else
/*#define GPIO_MENU		GPIO_PG(15)*/
#endif

#define GPIO_BACK		GPIO_PD(19)
#define GPIO_VOLUMEDOWN		GPIO_PD(17)
#define GPIO_VOLUMEUP		GPIO_PD(18)
#define GPIO_ENDCALL            GPIO_PA(30)

#define ACTIVE_LOW_HOME		1
#define ACTIVE_LOW_MENU         1
#define ACTIVE_LOW_BACK		1
#define ACTIVE_LOW_ENDCALL      1

#if defined(CONFIG_NAND)
#define ACTIVE_LOW_VOLUMEDOWN	0
#define ACTIVE_LOW_VOLUMEUP	1
#else
#define ACTIVE_LOW_VOLUMEDOWN	1
#define ACTIVE_LOW_VOLUMEUP	0
#endif

/**
 * USB detect pin
 **/
#define GPIO_USB_DETE                   GPIO_PA(16)

/**
 * pmem information
 **/
/* auto allocate pmem in arch_mem_init(), do not assigned base addr, just set 0 */
#define JZ_PMEM_ADSP_BASE   0x0         // 0x3e000000
#define JZ_PMEM_ADSP_SIZE   0x02000000


#endif /* __BOARD_H__ */
