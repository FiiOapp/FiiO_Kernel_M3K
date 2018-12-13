#include <mach/jzsfc.h>
#include <mach/spinand.h>
#include <mach/spinor.h>
#include "board_base.h"


#ifdef CONFIG_JZ_SFC_NOR
#include "sfc_nor_table.h"
#define SPINOR_UBOOT_OFF    0
#define SPINOR_UBOOT_SIZE   0x40000
#define SPINOR_KERNEL_OFF   (SPINOR_UBOOT_OFF + SPINOR_UBOOT_SIZE)
#define SPINOR_KERNEL_SIZE  0x300000
#define SPINOR_ROOTFS_OFF   0x360000
#define SPINOR_ROOTFS_SIZE  MAX_PART_SIZE
#define MAX_PART_SIZE   -1

#define SPINOR_INFO_NUM 0
#define SPINOR_FS_ERASE_32K 32768
#define SPINOR_FS_ERASE_64K 65536
#define SPINOR_QUAD_MODE    1
#define SPINOR_STANDARD_MODE    0

struct nor_partition sfc_nor_partition[] = {
    {
        .name = "uboot",
        .offset = SPINOR_UBOOT_OFF,
        .size = SPINOR_UBOOT_SIZE,
    },
    {
        .name = "kernel",
        .offset = SPINOR_KERNEL_OFF,
        .size = SPINOR_KERNEL_SIZE,
    },
    {
        .name = "rootfs",
        .offset = SPINOR_ROOTFS_OFF,
        .size = SPINOR_ROOTFS_SIZE,
    },
};


struct nor_private_data nor_pri_data = {
    .fs_erase_size = SPINOR_FS_ERASE_32K,
    .uk_quad = SPINOR_QUAD_MODE,
};

#endif

#ifdef CONFIG_JZ_SFCNAND
#define SPINAND_UBOOT_OFFSET    0x0
#define SPINAND_UBOOT_SIZE  0x100000
#define SPINAND_KERNEL_OFFSET   (SPINAND_UBOOT_OFFSET + SPINAND_UBOOT_SIZE)
#define SPINAND_KERNEL_SIZE 0x800000
#define SPINAND_ROOTFS_OFFSET   (SPINAND_KERNEL_OFFSET + SPINAND_KERNEL_SIZE)
#define SPINAND_ROOTFS_SIZE 0x2800000
#define SPINAND_DATA_OFFSET (SPINAND_ROOTFS_OFFSET + SPINAND_ROOTFS_SIZE)
#define SPINAND_DATA_SIZE   0x0
#define SPINAND_MASK_FLAGS  0x0

struct jz_sfcnand_partition spinand_partition[] = {
    {
        .name = "uboot",
        .offset = SPINAND_UBOOT_OFFSET,
        .size = SPINAND_UBOOT_SIZE,
        .mask_flags = SPINAND_MASK_FLAGS,
    },

    {
        .name = "kernel",
        .offset = SPINAND_KERNEL_OFFSET,
        .size = SPINAND_KERNEL_SIZE,
        .mask_flags = SPINAND_MASK_FLAGS,
    },

    {
        .name = "rootfs",
        .offset = SPINAND_ROOTFS_OFFSET,
        .size = SPINAND_ROOTFS_SIZE,
        .mask_flags = SPINAND_MASK_FLAGS,
    },

    {
        .name = "data",
        .offset = SPINAND_DATA_OFFSET,
        .size = SPINAND_DATA_SIZE,
        .mask_flags = SPINAND_MASK_FLAGS,
    },
};
#endif

#ifdef CONFIG_JZ_SFC_FLASH_POWER_CTRL
struct flash_power_ctrl power_ctrl = {
    .power_pin = GPIO_FLASH_POWER,
    .power_en_level = GPIO_FLASH_POWER_EN_LEVEL,
    .power_on_delay_ms = FLASH_POWER_ON_DELAY,
};
#endif /* CONFIG_JZ_SFC_FLASH_POWER_CTRL */

struct jz_sfc_info sfc_info_cfg = {
    .use_board_info = 0,

#if defined(CONFIG_JZ_SFC_NOR)
    .flash_param = &spi_nor_info_table[SPINOR_INFO_NUM],    //please check nor params before use
    .flash_partition = sfc_nor_partition,
    .num_partition = ARRAY_SIZE(sfc_nor_partition),
    .other_args = &nor_pri_data,
#elif defined(CONFIG_JZ_SFCNAND)
    .flash_param = NULL,
    .flash_partition = spinand_partition,
    .num_partition = ARRAY_SIZE(spinand_partition),
    .other_args = NULL,
#endif

#ifdef CONFIG_JZ_SFC_FLASH_POWER_CTRL
    .flash_power_ctrl = &power_ctrl,
#endif
};


