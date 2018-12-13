
#ifndef __JZ_VPU_V13_H__
#define __JZ_VPU_V13_H__

#include "jzm_jpeg_enc.h"

#define REG_VPU_GLBC      0x00000
#define VPU_INTE_ACFGERR     (0x1<<20)
#define VPU_INTE_TLBERR      (0x1<<18)
#define VPU_INTE_BSERR       (0x1<<17)
#define VPU_INTE_ENDF        (0x1<<16)

#define REG_VPU_STAT      0x00034
#define VPU_STAT_ENDF    (0x1<<0)
#define VPU_STAT_BPF     (0x1<<1)
#define VPU_STAT_ACFGERR (0x1<<2)
#define VPU_STAT_TIMEOUT (0x1<<3)
#define VPU_STAT_JPGEND  (0x1<<4)
#define VPU_STAT_BSERR   (0x1<<7)
#define VPU_STAT_TLBERR  (0x1F<<10)
#define VPU_STAT_SLDERR  (0x1<<16)

#define REG_VPU_JPGC_STAT 0xE0008
#define JPGC_STAT_ENDF   (0x1<<31)

#define REG_VPU_SDE_STAT  0x90000
#define SDE_STAT_BSEND   (0x1<<1)

#define REG_VPU_DBLK_STAT 0x70070
#define DBLK_STAT_DOEND  (0x1<<0)

#define REG_VPU_AUX_STAT  0xA0010
#define AUX_STAT_MIRQP   (0x1<<0)

enum {
	WAIT_COMPLETE = 0,
	LOCK,
	UNLOCK,
	FLUSH_CACHE,
};

struct flush_cache_info {
	unsigned int	addr;
	unsigned int	len;
#define WBACK		DMA_TO_DEVICE
#define INV		DMA_FROM_DEVICE
#define WBACK_INV	DMA_BIDIRECTIONAL
	unsigned int	dir;
};

struct vpu_dmmu_map_info {
	unsigned int	addr;
	unsigned int	len;
};

typedef struct {
	unsigned char *buf;
	unsigned char *BitStreamBuf;
#ifdef CHECK_RESULT
	uint8_t *soft_buf;
	uint8_t *soft_bts;
#endif
	unsigned int des_va;	/* descriptor virtual address */
	unsigned int des_pa;	/* descriptor physical address */
	int width;
	int height;
	int ql_sel;
	int bslen;
} YUYV_INFO;

#define uint8_t unsigned char
#define uint32_t unsigned int

/* JPEG encode quantization table select level */
typedef enum {
	LOW_QUALITY,
	MEDIUMS_QUALITY,
	HIGH_QUALITY,
} QUANT_QUALITY;

typedef struct _JPEGE_SliceInfo {
	unsigned int des_va;	/* descriptor virtual address */
	unsigned int des_pa;	/* descriptor physical address */
	uint8_t ncol;		/* number of color/components of a MCU minus one */
	uint8_t rsm;		/* Re-sync-marker enable */
	uint8_t *bsa;		/* bitstream buffer address  */
	uint8_t nrsm;		/* Re-Sync-Marker gap number */
	uint32_t nmcu;		/* number of MCU minus one */
	uint32_t raw[3];	/* {rawy, rawu, rawv} or {rawy, rawc, N/C} */
	uint32_t stride[2];    /* {stride_y, stride_c}, only used in raster raw */
	uint32_t mb_width;
	uint32_t mb_height;
	uint8_t raw_format;

	/* Quantization level select,0-2 level */
	QUANT_QUALITY ql_sel;
	uint8_t huffenc_sel;           /* Huffman ENC Table select */
}JPEGE_SliceInfo;

#define DESC_SIZE_MAX 2500

#define vpu_readl(vpu, offset)		__raw_readl((vpu)->iomem + offset)
#define vpu_writel(vpu, offset, value)	__raw_writel((value), (vpu)->iomem + offset)

#define REG_VPU_STATUS ( *(volatile unsigned int*)0xb3200034 )
#define REG_VPU_LOCK ( *(volatile unsigned int*)0xb329004c )
#define REG_VPUCDR ( *(volatile unsigned int*)0xb0000030 )
#define REG_CPM_VPU_SWRST ( *(volatile unsigned int*)0xb00000c4 )
#define CPM_VPU_SR           (0x1<<31)
#define CPM_VPU_STP          (0x1<<30)
#define CPM_VPU_ACK          (0x1<<29)

#define MAX_LOCK_DEPTH		999
#define CMD_VPU_CACHE 100
#define CMD_VPU_PHY   101
#define CMD_VPU_DMA_NOTLB 102
#define CMD_VPU_DMA_TLB 103
#define CMD_VPU_CLEAN_WAIT_FLAG 104
#define CMD_VPU_RESET 105
#define CMD_VPU_SET_REG 106
#define CMD_VPU_DMMU_MAP 107
#define CMD_VPU_DMMU_UNMAP 108
#define CMD_VPU_DMMU_UNMAP_ALL 109
#define CMD_WAIT_COMPLETE 110
/* we add them to wait for vpu end before suspend */
#define VPU_NEED_WAIT_END_FLAG 0x80000000
#define VPU_WAIT_OK 0x40000000
#define VPU_END 0x1

#define JPGE_TILE_MODE  0
#define JPGE_NV12_MODE  8
#define JPGE_NV21_MODE  12

#endif	/* __JZ_VPU_V13_H__ */
