/*
 * Copyright (c) 2016 Ingenic Semiconductor Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __JZDRM_CRTC_H__
#define __JZDRM_CRTC_H__

#define MAX_DESC_NUM 4
#define NUM_FRAME_BUFFERS 1
#define PIXEL_ALIGN 4

#define JZFB_SET_VSYNCINT      _IOW('F', 0x210, int)

#define to_jzdrm_crtc(x) container_of(x, struct jzdrm_crtc, base)

struct jzdrm_display_size {
	u32 fg0_line_size;
	u32 fg0_frm_size;
	u32 panel_line_size;
	u32 height_width;
};

enum jzdrm_format_order {
	FORMAT_X8R8G8B8 = 1,
	FORMAT_X8B8G8R8,
};

/**
 * @next: physical address of next frame descriptor
 * @databuf: physical address of buffer
 * @id: frame ID
 * @cmd: DMA command and buffer length(in word)
 * @offsize: DMA off size, in word
 * @page_width: DMA page width, in word
 * @cpos: smart LCD mode is commands' number, other is bpp,
 * premulti and position of foreground 0, 1
 * @desc_size: alpha and size of foreground 0, 1
 */
struct jzdrm_framedesc {
	uint32_t next;
	uint32_t databuf;
	uint32_t id;
	uint32_t cmd;
	uint32_t offsize;
	uint32_t page_width;
	uint32_t cpos;
	uint32_t desc_size;
} __packed;

/**
 * @fg: foreground 0 or foreground 1
 * @bpp: foreground bpp
 * @x: foreground start position x
 * @y: foreground start position y
 * @w: foreground width
 * @h: foreground height
 */
struct jzdrm_fg_t {
	u32 fg;
	u32 bpp;
	u32 x;
	u32 y;
	u32 w;
	u32 h;
};

/**
 *@decompress: enable decompress function, used by FG0
 *@block: enable 16x16 block function
 *@fg0: fg0 info
 *@fg1: fg1 info
 */
struct jzdrm_osd_t {
	int block;
	struct jzdrm_fg_t fg0;
	struct jzdrm_fg_t fg1;
};

struct jzdrm_crtc {
	struct drm_crtc base;

	int is_lcd_en;		/* 0, disable  1, enable */

	struct drm_display_mode *mode;

	const struct jzdrm_panel_info *crtc;
	uint32_t dirty;

	struct jzfb_platform_data *pdata;

	struct drm_pending_vblank_event *event;
	int dpms;
	wait_queue_head_t frame_done_wq;
	bool frame_done;

	/* fb currently set to scanout 0/1: */
	struct drm_framebuffer *scanout[2];

	/* for deferred fb unref's: */
	struct drm_flip_work unref_work;

	enum jzdrm_format_order fmt_order;	/* frame buffer pixel format order */
	struct jzdrm_osd_t osd;	/* osd's config information */

	struct jzdrm_framedesc *framedesc[MAX_DESC_NUM];
	 struct jzdrm_framedesc *fg1_framedesc;   /* FG 1 dma descriptor */
	size_t vidmem_size;
	/* DMA descriptors */
	int frm_size;
	int desc_num;

	dma_addr_t vidmem_phys;
	void *desc_cmd_vidmem;
	dma_addr_t desc_cmd_phys;
	dma_addr_t framedesc_phys;

	void __iomem *mmio;

	wait_queue_head_t vsync_wq;
	ktime_t vsync_timestamp;
	unsigned int vsync_skip_map;	/* 10 bits width */
	int vsync_skip_ratio;

#define TIMESTAMP_CAP	16
	struct {
		volatile int wp; /* write position */
		int rp;	/* read position */
		u64 value[TIMESTAMP_CAP];
	} timestamp;

#ifdef CONFIG_JZ_MIPI_DSI_MODE1
	struct dsi_device *dsi;
#endif
};
#endif /* __jzdrm_CRTC_H__ */

