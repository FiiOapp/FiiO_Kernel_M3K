#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <mach/jzssi.h>
#include <linux/spi/microarray.h>
#include <linux/spi/goodix_fp.h>
#include <linux/spi/fpc_pdata.h>
#include <linux/spi/byd_fps.h>
#include <linux/spi/sky1311s.h>
#include <linux/spi/sky1311t.h>

#include "board_base.h"


#if defined(CONFIG_JZ_SPI0) || defined(CONFIG_SPI_GPIO)
struct spi_board_info jz_spi0_board_info[] = {
#ifdef CONFIG_MTD_JZ_SPI_NOR
	[0] ={
		.modalias        =  "jz_spi_norflash",
		.platform_data   = NULL,
		.controller_data = (void *)SPI_CHIP_ENABLE, /* cs for spi gpio */
		.max_speed_hz    = 12000000,
		.bus_num         = 0,
		.chip_select     = 0,
	},
#endif
#ifdef CONFIG_MTD_JZ_SPI_NAND
	[0] = {
		.modalias        = "jz_spi_nand",
		.platform_data   = NULL,
		.controller_data = (void *)SPI_CHIP_ENABLE, /* cs for spi gpio */
		.max_speed_hz    = 12000000,
		.bus_num         = 0,
		.chip_select     = 0,
	},
#endif
};
int jz_spi0_devs_size = ARRAY_SIZE(jz_spi0_board_info);
#endif

#ifdef CONFIG_JZ_SPI0

#if defined(CONFIG_FINGERPRINT_MICROARRAY)
static struct microarray_platform_data spi_microarray_pdata = {
    .power_2v8         = FINGERPRINT_POWER_2V8,
    .power_1v8         = FINGERPRINT_POWER_1V8,
    .power_en          = FINGERPRINT_POWER_EN,
    .gpio_int          = FINGERPRINT_INT,
    .reset             = FINGERPRINT_RESET,
};
#elif defined(CONFIG_FINGERPRINT_GOODIX_GF5X)
static struct goodix_platform_data spi_goodix_pdata = {
    .pwr_en_pin        = GOODIX_FP_PWR_EN,
    .int_pin           = GOODIX_FP_INT,
    .reset_pin         = GOODIX_FP_RESET,
};
#elif defined(CONFIG_INPUT_BYD_FPS_SPI)
static struct byd_platform_data spi_byd_fps_pdata = {
	.pwr_en_pin        = BYD_FP_PWR_EN,
    .int_pin           = BYD_FP_INT,
    .reset_pin         = BYD_FP_RESET,
};
#elif defined(CONFIG_FINGERPRINT_FPC)
static struct fpc_platform_data spi_fpc_pdata = {
    .en_pin            = FPC_FP_EN,
    .int_pin           = FPC_FP_INT,
    .reset_pin         = FPC_FP_RESET,
    .cs_pin            = FPC_FP_CS,
    .wkup_ring_sel_pin = FPC_FP_WKUP_RING_SEL,
#ifdef CONFIG_MCLK_PROVIDED_TO_ENCRYPT_IC
    .encrypt_ic_clk_pin= ENCRYPT_IC_CLK,
#endif
    .encrypt_ic_rst_pin= ENCRYPT_IC_RST
};
#endif

#ifdef CONFIG_SKY1311S
struct sky1311s_platform_data spi_sky1311s_data = {
    .spi_cs_pin = SKY1311S_SPI_CS_PIN,
    .ic_int_pin = SKY1311S_IC_INT_PIN,
    .ic_pd2_pin = SKY1311S_IC_PD2_PIN,
};
#endif

#ifdef CONFIG_SKY1311T
struct sky1311t_platform_data spi_sky1311t_data = {
    .spi_cs_pin = SKY1311T_SPI_CS_PIN,
    .ic_int_pin = SKY1311T_IC_INT_PIN,
    .ic_pd2_pin = SKY1311T_IC_PD2_PIN,
};
#endif

#ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
struct spi_board_info spi0_devs_info[] = {
#if defined(CONFIG_FINGERPRINT_MICROARRAY)
    {
        .modalias        = MICROARRAY_DRV_NAME,
        .mode            = SPI_MODE_0,
        .max_speed_hz    = 10000000,
        .bus_num         = 0,
        .chip_select     = 0,
        .controller_data = NULL, /* cs for spi gpio */
        .platform_data   = &spi_microarray_pdata,
    },
#elif defined(CONFIG_FINGERPRINT_GOODIX_GF5X)
    {
        .modalias        = "goodix_fp",
        .mode            = SPI_MODE_0,
        .max_speed_hz    = 10000000,
        .bus_num         = 0,
        .chip_select     = 0,
        .controller_data = NULL, /* cs for spi gpio */
        .platform_data   = &spi_goodix_pdata,
    },
#elif defined(CONFIG_FINGERPRINT_FPC)
    {
        .modalias        = "fpc_fp",
        .mode            = SPI_MODE_0,
        .max_speed_hz    = 10000000,
        .bus_num         = 0,
        .chip_select     = 0,
        .controller_data = NULL, /* cs for spi gpio */
        .platform_data   = &spi_fpc_pdata,
    },
#elif defined(CONFIG_INPUT_BYD_FPS_SPI)
    {
        .modalias        = "byd_fps",
        .mode            = SPI_MODE_0,
        .max_speed_hz    = 50000000,
        .bus_num         = 0,
        .chip_select     = 0,
        .controller_data = NULL, /* cs for spi gpio */
        .platform_data   = &spi_byd_fps_pdata,
    },
#else
    {
        .modalias        = "spidev",
        .mode            = SPI_MODE_0,
        .max_speed_hz    = 10000000,
        .bus_num         = 0,
        .chip_select     = 0,
        .controller_data = NULL, /* cs for spi gpio */
        .platform_data   = NULL,
    },
#endif

#if defined(CONFIG_SKY1311S)
    {
        .modalias        = SKY1311S_DEV_NAME,
        .mode            = SPI_MODE_0,
        .max_speed_hz    = 10000000,
        .bus_num         = 0,
        .chip_select     = 1,
        .controller_data = NULL, /* cs for spi gpio */
        .platform_data   = &spi_sky1311s_data,
    },
#elif defined(CONFIG_SKY1311T)
    {
        .modalias        = SKY1311T_DEV_NAME,
        .mode            = SPI_MODE_0,
        .max_speed_hz    = 10000000,
        .bus_num         = 0,
        .chip_select     = 1,
        .controller_data = NULL, /* cs for spi gpio */
        .platform_data   = &spi_sky1311t_data,
    },
#endif /* #if defined(CONFIG_SKY1311S) */
};
#endif /* #ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER */

struct spi_cs_ctrl chipselect_ctrl[] = {
#if defined CONFIG_FINGERPRINT_FPC
    {
        .cs_pin  = SPI0_CHIP_SELECT0,
        .hold_cs = false,
    },
#else
    {
        .cs_pin  = SPI0_CHIP_SELECT0,
        .hold_cs = true,
    },
#endif

#if defined (CONFIG_SKY1311S_DRV_HOLD_CS)
    {
        .cs_pin  = SPI0_CHIP_SELECT1,
        .hold_cs = false,
    }
#elif defined (CONFIG_SKY1311T_DRV_HOLD_CS)
    {
        .cs_pin  = SPI0_CHIP_SELECT1,
        .hold_cs = false,
    }
#else
    {
        .cs_pin  = SPI0_CHIP_SELECT1,
        .hold_cs = true,
    }
#endif /* #if defined (CONFIG_SKY1311S_DRV_HOLD_CS) */
};
struct jz_spi_info spi0_info_cfg = {
    .chnl                = 0,
    .bus_num             = 0,
#ifdef CONFIG_JZ_SPI_CLK_SOURCE_EXTERNAL
    .max_clk             = 12000000,
#else
    .max_clk             = 50000000,
#endif
    .num_chipselect      = 2,
    .allow_cs_same       = 1,
    .chipselect          = chipselect_ctrl,
#ifdef CONFIG_JZ_SPI_BOARD_INFO_REGISTER
    .board_info          = spi0_devs_info,
    .board_size          = ARRAY_SIZE(spi0_devs_info),
#endif
};

#endif /* CONFIG_JZ_SPI0 */

#ifdef CONFIG_SPI_GPIO
static struct spi_gpio_platform_data jz_spi_gpio_data = {

	.sck	= GPIO_SPI_SCK,
	.mosi	= GPIO_SPI_MOSI,
	.miso	= GPIO_SPI_MISO,
	.num_chipselect = 1,
};

struct platform_device jz_spi_gpio_device = {
	.name   = "spi_gpio",
	.dev    = {
		.platform_data = &jz_spi_gpio_data,
	},
};
#endif

