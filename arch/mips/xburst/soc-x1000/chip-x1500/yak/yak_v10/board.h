#ifndef __BOARD_H__
#define __BOARD_H__
#include <gpio.h>
#include <soc/gpio.h>


/* ****************************GPIO KEY START******************************** */
/* #define GPIO_HOME_KEY		GPIO_PD(18) */
/* #define ACTIVE_LOW_HOME		1 */

/* #define GPIO_VOLUMEDOWN_KEY         GPIO_PD(18) */
/* #define ACTIVE_LOW_VOLUMEDOWN	0 */

#define GPIO_ENDCALL_KEY            GPIO_PB(31)
#define ACTIVE_LOW_ENDCALL      0

/* ****************************GPIO KEY END********************************** */


/* MSC GPIO Definition */
#define GPIO_SD0_CD_N       -1

#ifdef CONFIG_SPI_GPIO
#define GPIO_SPI_SCK  GPIO_PA(26)
#define GPIO_SPI_MOSI GPIO_PA(29)
#define GPIO_SPI_MISO GPIO_PA(28)
#endif

#if defined(CONFIG_JZ_SPI) || defined(CONFIG_JZ_SFC)
#define SPI_CHIP_ENABLE GPIO_PA(27)
#endif

/* ****************************GPIO USB START******************************** */
#define GPIO_USB_ID            	GPIO_PC(21)
#define GPIO_USB_ID_LEVEL       LOW_ENABLE
#ifdef CONFIG_BOARD_HAS_NO_DETE_FACILITY
#define GPIO_USB_DETE           -1 /*GPIO_PC(22)*/
#define GPIO_USB_DETE_LEVEL     LOW_ENABLE
#else
#define GPIO_USB_DETE           GPIO_PD(3)
#define GPIO_USB_DETE_LEVEL     LOW_ENABLE
#endif
#define GPIO_USB_DRVVBUS        GPIO_PB(25)
#define GPIO_USB_DRVVBUS_LEVEL      HIGH_ENABLE
/* ****************************GPIO USB END********************************** */

/* ****************************GPIO I2C START******************************** */
#ifndef CONFIG_I2C0_V12_JZ
#define GPIO_I2C0_SDA GPIO_PB(24)
#define GPIO_I2C0_SCK GPIO_PB(23)
#endif
#ifndef CONFIG_I2C1_V12_JZ
#define GPIO_I2C1_SDA GPIO_PC(27)
#define GPIO_I2C1_SCK GPIO_PC(26)
#endif
#ifndef CONFIG_I2C2_V12_JZ
#define GPIO_I2C2_SDA GPIO_PD(1)
#define GPIO_I2C2_SCK GPIO_PD(0)
#endif
/* ****************************GPIO I2C END********************************** */

#ifdef CONFIG_SENSORS_BMA2X2
#define GPIO_GSENSOR_INTR	GPIO_PB(2)
#endif

#ifdef CONFIG_VIDEO_JZ_CIM_HOST_V13
#define FRONT_CAMERA_INDEX  0
#define BACK_CAMERA_INDEX   1

#define CAMERA_SENSOR_RESET GPIO_PB(22)
#define CAMERA_FRONT_SENSOR_PWDN  GPIO_PB(21)
#define CAMERA_VDD_EN	(-ENODEV)
#endif

#define GPIO_EFUSE_VDDQ		(-ENODEV)/* GPIO_PB(27)	*//* EFUSE must be -ENODEV or a gpio */

#endif /* __BOARD_H__ */
