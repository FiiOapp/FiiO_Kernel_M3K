#ifndef __JZ_X2D_REG_H__
#define __JZ_X2D_REG_H__

#define REG_X2D_GLB_CTRL		0xf000

#define REG_X2D_GLB_STATUS              0xF004
#define REG_X2D_GLB_TRIG                0xf008
#define REG_X2D_TLB_BASE                0xf00c
#define REG_X2D_DHA                     0xf010
#define REG_X2D_WDOG_CNT                0xf018

#define REG_X2D_LAY_GCTRL               0xE000
#define REG_X2D_DST_BASE                0xE004
#define REG_X2D_DST_CTRL_STR            0xE008
#define REG_X2D_DST_GS                  0xE00c
#define REG_X2D_DST_MSK_ARGB            0xE010
#define REG_X2D_DST_FMT                 0xE014

#define REG_X2D_LAY0_CTRL               0x0000
#define REG_X2D_LAY0_Y_ADDR             0x0004
#define REG_X2D_LAY0_U_ADDR             0x0008
#define REG_X2D_LAY0_V_ADDR             0x000c
#define REG_X2D_LAY0_IN_FM_GS           0x0010
#define REG_X2D_LAY0_STRIDE             0x0014
#define REG_X2D_LAY0_OUT_GS             0x0018
#define REG_X2D_LAY0_OOSFT              0x001c
#define REG_X2D_LAY0_RSZ_COEF           0x0020
#define REG_X2D_LAY0_BK_ARGB            0x0024

#define REG_X2D_LAY1_CTRL               0x1000
#define REG_X2D_LAY1_Y_ADDR		0x1004
#define REG_X2D_LAY1_U_ADDR             0x1008
#define REG_X2D_LAY1_V_ADDR             0x100c
#define REG_X2D_LAY1_IN_FM_GS           0x1010
#define REG_X2D_LAY1_STRIDE             0x1014
#define REG_X2D_LAY1_OUT_GS             0x1018
#define REG_X2D_LAY1_OOSFT              0x101c
#define REG_X2D_LAY1_RSZ_COEF           0x1020
#define REG_X2D_LAY1_BK_ARGB            0x1024

#define REG_X2D_LAY2_CTRL               0x2000
#define REG_X2D_LAY2_Y_ADDR             0x2004
#define REG_X2D_LAY2_U_ADDR             0x2008
#define REG_X2D_LAY2_V_ADDR             0x200c
#define REG_X2D_LAY2_IN_FM_GS           0x2010
#define REG_X2D_LAY2_STRIDE             0x2014
#define REG_X2D_LAY2_OUT_GS             0x2018
#define REG_X2D_LAY2_OOSFT              0x201c
#define REG_X2D_LAY2_RSZ_COEF           0x2020
#define REG_X2D_LAY2_BK_ARGB            0x2024

#define REG_X2D_LAY3_CTRL               0x3000
#define REG_X2D_LAY3_Y_ADDR             0x3004
#define REG_X2D_LAY3_U_ADDR             0x3008
#define REG_X2D_LAY3_V_ADDR             0x300c
#define REG_X2D_LAY3_IN_FM_GS           0x3010
#define REG_X2D_LAY3_STRIDE             0x3014
#define REG_X2D_LAY3_OUT_GS             0x3018
#define REG_X2D_LAY3_OOSFT              0x301c
#define REG_X2D_LAY3_RSZ_COEF           0x3020
#define REG_X2D_LAY3_BK_ARGB            0x3024

#define X2D_WTDOG_ERR 			0x20
#define X2D_BUSY 			0xf

#define BIT_X2D_SHARP_LEVEL		16
#define BIT_X2D_BANK_SELC		12
#define BIT_X2D_LAY3_TLB_EN		8
#define BIT_X2D_LAY2_TLB_EN		7
#define BIT_X2D_LAY1_TLB_EN		6
#define BIT_X2D_LAY0_TLB_EN		5
#define BIT_X2D_CMD_TLB_EN		4
#define BIT_X2D_DMA_MOD_EN		3
#define BIT_X2D_WDOG_EN			2
#define BIT_X2D_DST_TLB_EN		1
#define BIT_X2D_IRQ_EN			0

#define BIT_X2D_ID			16
#define BIT_X2D_WTCDOG_ERR		5
#define BIT_X2D_IRQ_FLAG		4
#define BIT_X2D_GLB_STATE		0

#define BIT_X2D_IRQ_CLR			3
#define BIT_X2D_RST			2
#define BIT_X2D_STOP_COMP		1
#define BIT_X2D_START_COMP		0

#define BIT_X2D_TILE_MOD		16
#define BIT_X2D_LAYER_NUM		0

#define BIT_X2D_DST_GLB_ALPHA_VAL	24
#define BIT_X2D_DST_MSK_EN		23
#define BIT_X2D_DST_PREM_EN		22
#define BIT_X2D_DST_GLB_ALPHA_EN	21
#define BIT_X2D_DST_BG_EN		20
#define BIT_X2D_DST_STRIDE		0

#define BIT_X2D_DST_GEO_WIDTH		0
#define BIT_X2D_DST_GEO_HEIGHT		16

#define BIT_X2D_DST_ALPHA_POS		8
#define BIT_X2D_DST_RGB_FORMAT		4
#define BIT_X2D_DST_RGB_ORDER		0

#define BIT_X2D_LAY_OSD_MOD		28
#define BIT_X2D_LAY_ALPHA_POS		27
#define BIT_X2D_LAY_RGB_ORDER		24
#define BIT_X2D_LAY_V_MIRROR		19
#define BIT_X2D_LAY_H_MIRROR		18
#define BIT_X2D_LAY_ROTATE		16
#define BIT_X2D_LAY_GLB_ALPHA_VAL	8
#define BIT_X2D_LAY_MSK_EN		6
#define BIT_X2D_LAY_CSCM_EN		5
#define BIT_X2D_LAY_PREM_EN		4
#define BIT_X2D_LAY_GLB_ALPHA_EN	3
#define BIT_X2D_LAY_INPUT_FORMAT	0

#define BIT_X2D_LAY_WIDTH		0
#define BIT_X2D_LAY_HEIGHT		16

#define BIT_X2D_LAY_Y_STRIDE 		0
#define BIT_X2D_LAY_UV_STRIDE		16

#define BIT_X2D_LAY_VSCALE_RATIO	16
#define BIT_X2D_LAY_HSCALE_RATIO	0

#define BIT_X2D_LAY_BG_AVAL
#define BIT_X2D_LAY_BG_RVAL
#define BIT_X2D_LAY_BG_GVAL
#define BIT_X2D_LAY_BG_BVAL

//static   unsigned long reg_read( int offset);
//static   void reg_write(int offset, unsigned long val);

#define X2D_ID					0xE003

enum X2D_IN_FMT {
	Alpha_RGB888 = 0,
	RGB555,
	RGB565,
	YUV420_separate,
	Tile_YUV420,
	NV12,
	NV21,
};

#define __x2d_read_devid()  	((0xffff <<BIT_X2D_ID & (reg_read(jz_x2d,REG_X2D_GLB_STATUS)) )>>BIT_X2D_ID)
//#define __x2d_if_wthdog_err() (1<< BIT_X2D_WDOG_EN & (reg_read(jz_x2d,REG_X2D_GLB_STATUS)))
//#define __x2d_if_irq_happpen() ((1<< BIT_X2D_IRQ_FLAG) & (reg_read(jz_x2d,REG_X2D_GLB_STATUS)))
//#define __x2d_if_free()		   !( 0xf << BIT_X2D_GLB_STATE & (reg_read(jz_x2d,REG_X2D_GLB_STATUS)))

#define __x2d_setup_default_vaddr()	do{unsigned int reg = 0;\
		reg |= 2<<BIT_X2D_SHARP_LEVEL;\
		reg |= 3<<BIT_X2D_BANK_SELC;\
		reg |= 1<<BIT_X2D_LAY0_TLB_EN;\
		reg |= 1<<BIT_X2D_LAY1_TLB_EN;\
		reg |= 1<<BIT_X2D_LAY2_TLB_EN;\
		reg |= 1<<BIT_X2D_LAY3_TLB_EN;\
		reg |= 1<<BIT_X2D_DST_TLB_EN;\
		reg_write(jz_x2d,REG_X2D_GLB_CTRL,reg);\
}while(0)

#define __x2d_setup_default()	do{unsigned int reg = 0;\
		reg |= 2<<BIT_X2D_SHARP_LEVEL;\
		reg |= 3<<BIT_X2D_BANK_SELC;\
		reg_write(jz_x2d,REG_X2D_GLB_CTRL,reg);\
}while(0)

//reg |= 1<<BIT_X2D_CMD_TLB_EN;


#define __x2d_enable_wthdog()  bit_set(jz_x2d,REG_X2D_GLB_CTRL ,BIT_X2D_WDOG_EN)
#define __x2d_disable_wthdog()  bit_clr(jz_x2d,REG_X2D_GLB_CTRL ,BIT_X2D_WDOG_EN)
#define __x2d_enable_irq()		bit_set(jz_x2d,REG_X2D_GLB_CTRL ,BIT_X2D_IRQ_EN)
#define __x2d_disable_irq()  bit_clr(jz_x2d,REG_X2D_GLB_CTRL ,BIT_X2D_IRQ_EN)
#define __x2d_enable_dma()		bit_set(jz_x2d,REG_X2D_GLB_CTRL ,BIT_X2D_DMA_MOD_EN)
#define __x2d_disable_dma()  bit_clr(jz_x2d,REG_X2D_GLB_CTRL ,BIT_X2D_DMA_MOD_EN)

#define __x2d_set_lay_tlb(num) bit_set(jz_x2d,REG_X2D_GLB_CTRL ,(num+5))
#define __x2d_clr_lay_tlb(num) bit_clr(jz_x2d,REG_X2D_GLB_CTRL ,(num+5))

#define __x2d_set_dst_tlb()	bit_set(jz_x2d,REG_X2D_GLB_CTRL ,BIT_X2D_DST_TLB_EN)
#define __x2d_clr_dst_tlb()	bit_clr(jz_x2d,REG_X2D_GLB_CTRL ,BIT_X2D_DST_TLB_EN)

#define __x2d_start_trig()  bit_set(jz_x2d,REG_X2D_GLB_TRIG ,BIT_X2D_START_COMP)
#define __x2d_stop_trig()  bit_set(jz_x2d,REG_X2D_GLB_TRIG ,BIT_X2D_STOP_COMP)
#define __x2d_reset_trig()  bit_set(jz_x2d,REG_X2D_GLB_TRIG ,BIT_X2D_RST)
#define __x2d_clear_irq()  bit_set(jz_x2d,REG_X2D_GLB_TRIG ,BIT_X2D_IRQ_CLR)


#endif
