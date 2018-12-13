#ifndef __JZ_X2D_H__
#define __JZ_X2D_H__

/*
 * IOCTL_XXX commands
 */
#define X2D_IOCTL_MAGIC 'X'

#define IOCTL_X2D_SET_CONFIG			_IOW(X2D_IOCTL_MAGIC, 0x01, struct jz_x2d_config)
#define IOCTL_X2D_START_COMPOSE		    _IO(X2D_IOCTL_MAGIC, 0x02)
#define IOCTL_X2D_GET_SYSINFO		    _IOR(X2D_IOCTL_MAGIC, 0x03, struct jz_x2d_config)
#define IOCTL_X2D_STOP					_IO(X2D_IOCTL_MAGIC, 0x04)
#define IOCTL_X2D_MAP_GRAPHIC_BUF		_IO(X2D_IOCTL_MAGIC, 0x05)
#define IOCTL_X2D_FREE_GRAPHIC_BUF		_IO(X2D_IOCTL_MAGIC, 0x06)


#define X2D_RGBORDER_RGB 			 0
#define X2D_RGBORDER_BGR			 1
#define X2D_RGBORDER_GRB			 2
#define X2D_RGBORDER_BRG			 3
#define X2D_RGBORDER_RBG			 4
#define X2D_RGBORDER_GBR			 5

#define X2D_ALPHA_POSLOW			 1
#define X2D_ALPHA_POSHIGH			 0

#define X2D_H_MIRROR				 4
#define X2D_V_MIRROR				 8
#define X2D_ROTATE_0				 0
#define	X2D_ROTATE_90 				 1
#define X2D_ROTATE_180				 2
#define X2D_ROTATE_270				 3

#define X2D_INFORMAT_ARGB888		0
#define X2D_INFORMAT_RGB555			1
#define X2D_INFORMAT_RGB565			2
#define X2D_INFORMAT_YUV420SP		3
#define X2D_INFORMAT_TILE420		4
#define X2D_INFORMAT_NV12			5
#define X2D_INFORMAT_NV21			6

#define NEW_DST_FORMAT_BIT (1<<31)

#define X2D_OUTFORMAT_ARGB888		0
#define X2D_OUTFORMAT_XARGB888		1
#define X2D_OUTFORMAT_RGB565		2
#define X2D_OUTFORMAT_RGB555		3

#define X2D_OSD_MOD_CLEAR			3
#define X2D_OSD_MOD_SOURCE			1
#define X2D_OSD_MOD_DST				2
#define X2D_OSD_MOD_SRC_OVER		0
#define X2D_OSD_MOD_DST_OVER		4
#define X2D_OSD_MOD_SRC_IN			5
#define X2D_OSD_MOD_DST_IN			6
#define X2D_OSD_MOD_SRC_OUT			7
#define X2D_OSD_MOD_DST_OUT			8
#define X2D_OSD_MOD_SRC_ATOP		9
#define X2D_OSD_MOD_DST_ATOP		0xa
#define X2D_OSD_MOD_XOR				0xb

/*Add by bcjia at 2013-9-23
 *if you add physics addr for x2d
 *please define X2D_USE_PHY_ADDR and modify
 *struct src_layer and  struct jz_x2d_config
 *in hwcomposer
*/
//#define X2D_USE_PHY_ADDR

enum jz_x2d_state {
	x2d_state_idle,
	x2d_state_calc,
	x2d_state_complete,
	x2d_state_error,
	x2d_state_suspend,
};

enum jz_x2d_errcode {
	error_none,
	error_calc,
	error_wthdog,
	error_TLB,
};

/* Order Cannot be Changed! */
typedef struct x2d_lay_info {
	uint8_t lay_ctrl;
	uint8_t lay_galpha;
	uint8_t rom_ctrl;     //rotate and mirror control
	uint8_t RGBM;

	uint32_t y_addr;

	uint32_t u_addr;
	uint32_t v_addr;

	uint16_t swidth;
	uint16_t sheight;
	uint16_t ystr;
	uint16_t uvstr;

	uint16_t owidth;
	uint16_t oheight;
	uint16_t oxoffset;
	uint16_t oyoffset;

	uint16_t rsz_hcoef;
	uint16_t rsz_vcoef;
	uint32_t bk_argb;
}x2d_lay_info, *x2d_lay_info_p;

/* Order Cannot be Changed! */
typedef struct x2d_chain_info {
	uint16_t   overlay_num;
	uint16_t   dst_tile_en;
	uint32_t   dst_addr;
	uint32_t   dst_ctrl_str;
	uint16_t   dst_width;
	uint16_t   dst_height;
	uint32_t   dst_argb;
	uint32_t   dst_fmt;
	x2d_lay_info x2d_lays[4];
} x2d_chain_info, *x2d_chain_info_p;

struct src_layer {
	int format;
	int transform;  //such as rotate or mirror
	int global_alpha_val;
	int argb_order;
	int osd_mode;
	int preRGB_en;
	int glb_alpha_en;
	int mask_en;
	int color_cov_en;

	/* input && output size */
	int in_width;			//LAY0_SGS
	int in_height;
	int in_w_offset;
	int in_h_offset;
	int out_width;			//LAY0_OGS
	int out_height;
	int out_w_offset;		//LAY0_OOSF
	int out_h_offset;

	int v_scale_ratio;
	int h_scale_ratio;
	int msk_val;

	/* yuv virtual address */
	int addr;
	int u_addr;
	int v_addr;

#ifdef X2D_USE_PHY_ADDR
	/* yuv physics address */
	int addr_p;
	int u_addr_p;
	int v_addr_p;
#endif
	int y_stride;
	int v_stride;
};

struct jz_x2d_config{
	/* global val */
	int watchdog_cnt;
	unsigned int tlb_base;

	/* dst val */
	int dst_address;		//DST_BASE (virtual addr)
#ifdef X2D_USE_PHY_ADDR
	int dst_address_p;		//DST_BASE (physics addr)
#endif
	int dst_alpha_val;		//DST__CTRL_STR - alpha
	int dst_stride;			//DST__CTRL_STR -DST_STR
	int dst_mask_val;		//DST_MASK_ARGB
	int dst_width;			//DST_GS
	int dst_height;			//DST_GS
	int dst_bcground;		//DST_MASK_ARGB
	int dst_format;			//DST_FMT -RGB_FMT
	int dst_back_en;		//DST_CTRL_STR -back_en
	int dst_preRGB_en;		//DST_CTRL_STR -preM_en
	int dst_glb_alpha_en;	//DST_CTRL_STR -Glba_en
	int dst_mask_en;	    //DST_CTRL_STR -Msk_en
	//int dst_backpic_alpha_en;

	/* src layers */
	int layer_num;
	struct src_layer lay[4];
};

struct x2d_proc_info {
	pid_t pid;
	unsigned int tlb_base;
	int *record_addr_list;
	int record_addr_num;

	struct file *x2d_filp;
	struct jz_x2d_config configs;
	struct list_head list;
};


struct jz_x2d_platform_data {
	void (*power_on)(void);
	void (*power_off)(void);

};



#endif // __JZ_CIM_H__

