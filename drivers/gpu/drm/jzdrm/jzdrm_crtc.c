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

#include "drm_flip_work.h"
#include "jzdrm_drv.h"
#include "regs.h"
#include "jzdrm_crtc.h"

#ifdef CONFIG_JZ_MIPI_DSI
#define CONFIG_SLCDC_CONTINUA
#endif

#ifdef CONFIG_JZ_MIPI_DSI_MODE1
#include "./jz_mipi_dsi/jz_mipi_dsih_hal.h"
#include "./jz_mipi_dsi/jz_mipi_dsi_regs.h"
#include "./jz_mipi_dsi/jz_mipi_dsi_lowlevel.h"
#include <mach/jz_dsim.h>
extern struct dsi_device * jzdsi_init(struct jzdsi_data *pdata);
extern void jzdsi_remove(struct dsi_device *dsi);
extern void dump_dsi_reg(struct dsi_device *dsi);
int jz_mipi_dsi_set_client(struct dsi_device *dsi, int power);
#endif

#define print_dbg(f, arg...) printk(KERN_INFO __FILE__ "-- %s,%d: " f "\n", __func__, __LINE__, ## arg)

void dump_lcdc_registers(struct drm_crtc *crtc)
{
    int i;
    long unsigned int tmp;
    struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
    struct drm_device *dev = crtc->dev;
    int desc_num = jzdrm_crtc->desc_num;

    printk("jzdrm_crtc->mmio:\t0x%08x\n", (unsigned int)(jzdrm_crtc->mmio));
    printk("LCDC_CFG:(0x%08x)\t0x%08lx\n",LCDC_CFG, jzdrm_read(dev, LCDC_CFG));
    printk("LCDC_CTRL:(0x%08x)\t0x%08lx\n", LCDC_CTRL,jzdrm_read(dev, LCDC_CTRL));
    printk("LCDC_STATE:(0x%08x)\t0x%08lx\n",LCDC_STATE, jzdrm_read(dev, LCDC_STATE));
    printk("LCDC_OSDC:(0x%08x)\t0x%08lx\n",LCDC_OSDC, jzdrm_read(dev, LCDC_OSDC));
    printk("LCDC_OSDCTRL:(0x%08x)\t0x%08lx\n",LCDC_OSDCTRL, jzdrm_read(dev, LCDC_OSDCTRL));
    printk("LCDC_OSDS:(0x%08x)\t0x%08lx\n",LCDC_OSDS, jzdrm_read(dev, LCDC_OSDS));
    printk("LCDC_BGC0:(0x%08x)\t0x%08lx\n",LCDC_BGC0, jzdrm_read(dev, LCDC_BGC0));
    printk("LCDC_BGC1:(0x%08x)\t0x%08lx\n",LCDC_BGC1, jzdrm_read(dev, LCDC_BGC1));
    printk("LCDC_KEY0:(0x%08x)\t0x%08lx\n",LCDC_KEY0, jzdrm_read(dev, LCDC_KEY0));
    printk("LCDC_KEY1:(0x%08x)\t0x%08lx\n", LCDC_KEY1,jzdrm_read(dev, LCDC_KEY1));
    printk("LCDC_ALPHA:(0x%08x)\t0x%08lx\n",LCDC_ALPHA, jzdrm_read(dev, LCDC_ALPHA));
    printk("LCDC_IPUR:(0x%08x)\t0x%08lx\n",LCDC_IPUR, jzdrm_read(dev, LCDC_IPUR));
    printk("==================================\n");
    tmp = jzdrm_read(dev, LCDC_VAT);
    printk("LCDC_VAT:(0x%08x)\t0x%08lx, HT = %ld, VT = %ld\n",LCDC_VAT, tmp,
            (tmp & LCDC_VAT_HT_MASK) >> LCDC_VAT_HT_BIT,
            (tmp & LCDC_VAT_VT_MASK) >> LCDC_VAT_VT_BIT);
    tmp = jzdrm_read(dev, LCDC_DAH);
    printk("LCDC_DAH:(0x%08x)\t0x%08lx, HDS = %ld, HDE = %ld\n", LCDC_DAH,tmp,
            (tmp & LCDC_DAH_HDS_MASK) >> LCDC_DAH_HDS_BIT,
            (tmp & LCDC_DAH_HDE_MASK) >> LCDC_DAH_HDE_BIT);
    tmp = jzdrm_read(dev, LCDC_DAV);
    printk("LCDC_DAV:(0x%08x)\t0x%08lx, VDS = %ld, VDE = %ld\n", LCDC_DAV,tmp,
            (tmp & LCDC_DAV_VDS_MASK) >> LCDC_DAV_VDS_BIT,
            (tmp & LCDC_DAV_VDE_MASK) >> LCDC_DAV_VDE_BIT);
    tmp = jzdrm_read(dev, LCDC_HSYNC);
    printk("LCDC_HSYNC:(0x%08x)\t0x%08lx, HPS = %ld, HPE = %ld\n", LCDC_HSYNC,tmp,
            (tmp & LCDC_HSYNC_HPS_MASK) >> LCDC_HSYNC_HPS_BIT,
            (tmp & LCDC_HSYNC_HPE_MASK) >> LCDC_HSYNC_HPE_BIT);
    tmp = jzdrm_read(dev, LCDC_VSYNC);
    printk("LCDC_VSYNC:(0x%08x)\t0x%08lx, VPS = %ld, VPE = %ld\n", LCDC_VSYNC,tmp,
            (tmp & LCDC_VSYNC_VPS_MASK) >> LCDC_VSYNC_VPS_BIT,
            (tmp & LCDC_VSYNC_VPE_MASK) >> LCDC_VSYNC_VPE_BIT);
    printk("==================================\n");
    printk("LCDC_XYP0:(0x%08x)\t0x%08lx\n",LCDC_XYP0, jzdrm_read(dev, LCDC_XYP0));
    printk("LCDC_XYP1:(0x%08x)\t0x%08lx\n",LCDC_XYP1, jzdrm_read(dev, LCDC_XYP1));
    printk("LCDC_SIZE0:(0x%08x)\t0x%08lx\n",LCDC_SIZE0, jzdrm_read(dev, LCDC_SIZE0));
    printk("LCDC_SIZE1:(0x%08x)\t0x%08lx\n",LCDC_SIZE1, jzdrm_read(dev, LCDC_SIZE1));
    printk("LCDC_RGBC:(0x%08x) \t0x%08lx\n",LCDC_RGBC, jzdrm_read(dev, LCDC_RGBC));
    printk("LCDC_PS:(0x%08x)\t0x%08lx\n",LCDC_PS, jzdrm_read(dev, LCDC_PS));
    printk("LCDC_CLS:(0x%08x)\t0x%08lx\n",LCDC_CLS, jzdrm_read(dev, LCDC_CLS));
    printk("LCDC_SPL:(0x%08x)\t0x%08lx\n",LCDC_SPL, jzdrm_read(dev, LCDC_SPL));
    printk("LCDC_REV:(0x%08x)\t0x%08lx\n",LCDC_REV, jzdrm_read(dev, LCDC_REV));
    printk("LCDC_IID:(0x%08x)\t0x%08lx\n",LCDC_IID, jzdrm_read(dev, LCDC_IID));
    printk("==================================\n");
    printk("LCDC_DA0:(0x%08x)\t0x%08lx\n",LCDC_DA0, jzdrm_read(dev, LCDC_DA0));
    printk("LCDC_SA0:(0x%08x)\t0x%08lx\n",LCDC_SA0, jzdrm_read(dev, LCDC_SA0));
    printk("LCDC_FID0:(0x%08x)\t0x%08lx\n",LCDC_FID0, jzdrm_read(dev, LCDC_FID0));
    printk("LCDC_CMD0:(0x%08x)\t0x%08lx\n",LCDC_CMD0, jzdrm_read(dev, LCDC_CMD0));
    printk("LCDC_OFFS0:(0x%08x)\t0x%08lx\n",LCDC_OFFS0, jzdrm_read(dev, LCDC_OFFS0));
    printk("LCDC_PW0:(0x%08x)\t0x%08lx\n", LCDC_PW0,jzdrm_read(dev, LCDC_PW0));
    printk("LCDC_CNUM0:(0x%08x)\t0x%08lx\n",LCDC_CNUM0, jzdrm_read(dev, LCDC_CNUM0));
    printk("LCDC_DESSIZE0:(0x%08x)\t0x%08lx\n",LCDC_DESSIZE0,
            jzdrm_read(dev, LCDC_DESSIZE0));
    printk("==================================\n");
    printk("LCDC_DA1:(0x%08x)\t0x%08lx\n",LCDC_DA1, jzdrm_read(dev, LCDC_DA1));
    printk("LCDC_SA1:(0x%08x)\t0x%08lx\n",LCDC_SA1, jzdrm_read(dev, LCDC_SA1));
    printk("LCDC_FID1:(0x%08x)\t0x%08lx\n",LCDC_FID1, jzdrm_read(dev, LCDC_FID1));
    printk("LCDC_CMD1:(0x%08x)\t0x%08lx\n", LCDC_CMD1,jzdrm_read(dev, LCDC_CMD1));
    printk("LCDC_OFFS1:(0x%08x)\t0x%08lx\n",LCDC_OFFS1, jzdrm_read(dev, LCDC_OFFS1));
    printk("LCDC_PW1:(0x%08x)\t0x%08lx\n",LCDC_PW1, jzdrm_read(dev, LCDC_PW1));
    printk("LCDC_CNUM1:(0x%08x)\t0x%08lx\n",LCDC_CNUM1, jzdrm_read(dev, LCDC_CNUM1));
    printk("LCDC_DESSIZE1:(0x%08x)\t0x%08lx\n",LCDC_DESSIZE1,
            jzdrm_read(dev, LCDC_DESSIZE1));
    printk("==================================\n");
    printk("LCDC_PCFG:(0x%08x)\t0x%08lx\n",LCDC_PCFG, jzdrm_read(dev, LCDC_PCFG));
    printk("==================================\n");
    printk("SLCDC_CFG:(0x%08x) \t0x%08lx\n",SLCDC_CFG, jzdrm_read(dev, SLCDC_CFG));
    printk("SLCDC_CTRL:(0x%08x) \t0x%08lx\n",SLCDC_CTRL, jzdrm_read(dev, SLCDC_CTRL));
    printk("SLCDC_STATE:(0x%08x) \t0x%08lx\n",SLCDC_STATE, jzdrm_read(dev, SLCDC_STATE));
    printk("SLCDC_DATA:(0x%08x) \t0x%08lx\n",SLCDC_DATA, jzdrm_read(dev, SLCDC_DATA));
    printk("SLCDC_CFG_NEW:(0x%08x) \t0x%08lx\n",SLCDC_CFG_NEW,
            jzdrm_read(dev, SLCDC_CFG_NEW));
    printk("SLCDC_WTIME:(0x%08x) \t0x%08lx\n",SLCDC_WTIME, jzdrm_read(dev, SLCDC_WTIME));
    printk("SLCDC_TAS:(0x%08x) \t0x%08lx\n",SLCDC_TAS, jzdrm_read(dev, SLCDC_TAS));
    printk("==================================\n");
    printk("reg:0x10000020 value=0x%08x  (24bit) Clock Gate Register0\n",
            *(unsigned int *)0xb0000020);
    printk("reg:0x100000e4 value=0x%08x  (5bit_lcdc 21bit_lcdcs) Power Gate Register: \n",
            *(unsigned int *)0xb00000e4);
    printk("reg:0x100000b8 value=0x%08x  (10bit) SRAM Power Control Register0 \n",
            *(unsigned int *)0xb00000b8);
    printk("reg:0x10000064 value=0x%08x  Lcd pixclock \n",
            *(unsigned int *)0xb0000064);
    printk("==================================\n");
    printk("PCINT:\t0x%08x\n", *(unsigned int *)0xb0010210);
    printk("PCMASK:\t0x%08x\n",*(unsigned int *)0xb0010220);
    printk("PCPAT1:\t0x%08x\n",*(unsigned int *)0xb0010230);
    printk("PCPAT0:\t0x%08x\n",*(unsigned int *)0xb0010240);
        printk("==================================\n");
    for (i = 0; i < desc_num; i++) {
        if (!jzdrm_crtc->framedesc)
            break;
        printk("==================================\n");
        if (i != desc_num - 1) {
            printk("jzdrm_crtc->framedesc[%d]: %p\n", i,
                    jzdrm_crtc->framedesc[i]);
            printk("DMA 0 descriptor value in memory\n");
        } else {
            printk("jzdrm_crtc->fg1_framedesc: %p\n",
                    &jzdrm_crtc->framedesc[i]);
            printk("DMA 1 descriptor value in memory\n");
        }
        printk("framedesc[%d]->next: \t0x%08x\n", i,
                jzdrm_crtc->framedesc[i]->next);
        printk("framedesc[%d]->databuf:  \t0x%08x\n", i,
                jzdrm_crtc->framedesc[i]->databuf);
        printk("framedesc[%d]->id: \t0x%08x\n", i,
                jzdrm_crtc->framedesc[i]->id);
        printk("framedesc[%d]->cmd:\t0x%08x\n", i,
                jzdrm_crtc->framedesc[i]->cmd);
        printk("framedesc[%d]->offsize:\t0x%08x\n", i,
                jzdrm_crtc->framedesc[i]->offsize);
        printk("framedesc[%d]->page_width:\t0x%08x\n", i,
                jzdrm_crtc->framedesc[i]->page_width);
        printk("framedesc[%d]->cpos:\t0x%08x\n", i,
                jzdrm_crtc->framedesc[i]->cpos);
        printk("framedesc[%d]->desc_size:\t0x%08x\n", i,
                jzdrm_crtc->framedesc[i]->desc_size);
    }

    return;
}

static void jzdrm_clk_enable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct jzdrm_drm_private *priv = dev->dev_private;
	if(priv->is_clk_en){
		return;
	}
	clk_prepare_enable(priv->pwc_lcd);
	clk_prepare_enable(priv->clk);
	clk_prepare_enable(priv->disp_clk);
	priv->is_clk_en = 1;
}

static void jzdrm_clk_disable(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct jzdrm_drm_private *priv = dev->dev_private;
	if(!priv->is_clk_en){
		return;
	}
	priv->is_clk_en = 0;
	clk_disable(priv->disp_clk);
	clk_disable(priv->clk);
	clk_disable(priv->pwc_lcd);
}

static void start(struct drm_crtc *crtc)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	uint32_t ctrl;

	if (jzdrm_crtc->is_lcd_en) {
		return;
	}

	jzdrm_write(dev, LCDC_STATE, 0);
	jzdrm_write(dev, LCDC_OSDS, 0);
	ctrl = jzdrm_read(dev, LCDC_CTRL);
	ctrl |= LCDC_CTRL_ENA;
	ctrl &= ~LCDC_CTRL_DIS;
	jzdrm_write(dev, LCDC_CTRL, ctrl);
	jzdrm_crtc->is_lcd_en = 1;

}

static void stop(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	int count = 15;
	uint32_t ctrl;

	if (!jzdrm_crtc->is_lcd_en) {
		return;
	}

	if (jzdrm_crtc->pdata->lcd_type != LCD_TYPE_SLCD) {
		if(jzdrm_read(dev, LCDC_CTRL) & LCDC_CTRL_ENA) {
			ctrl = jzdrm_read(dev, LCDC_CTRL);
			ctrl |= LCDC_CTRL_DIS;
			jzdrm_write(dev, LCDC_CTRL, ctrl);
			while (!(jzdrm_read(dev, LCDC_STATE) & LCDC_STATE_LDD)
				&& count--) {
				usleep_range(1000, 2000);
			}
			if (count >= 0) {
				ctrl = jzdrm_read(dev, LCDC_STATE);
				ctrl &= ~LCDC_STATE_LDD;
				jzdrm_write(dev, LCDC_STATE, ctrl);
			} else {
			DRM_DEBUG_DRIVER("LCDC normal disable state wrong");
			}
		}
	}else{
		/* SLCD and TVE only support quick disable */
		ctrl = jzdrm_read(dev, LCDC_CTRL);
		ctrl &= ~LCDC_CTRL_ENA;
		jzdrm_write(dev, LCDC_CTRL, ctrl);

		ctrl = jzdrm_read(dev, SLCDC_CTRL);
		ctrl &= ~SLCDC_CTRL_DMA_EN;
		jzdrm_write(dev, SLCDC_CTRL, ctrl);
	}

	jzdrm_crtc->is_lcd_en = 0;
}

static void unref_worker(struct drm_flip_work *work, void *val)
{
	struct jzdrm_crtc *jzdrm_crtc =
		container_of(work, struct jzdrm_crtc, unref_work);
	struct drm_device *dev = jzdrm_crtc->base.dev;

	mutex_lock(&dev->mode_config.mutex);
	drm_framebuffer_unreference(val);
	mutex_unlock(&dev->mode_config.mutex);
}

static int jzdrm_get_controller_bpp(struct drm_crtc *crtc)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	switch (jzdrm_crtc->pdata->bpp) {
	case 18:
	case 24:
		return 32;
	case 15:
		return 16;
	default:
		return jzdrm_crtc->pdata->bpp;
	}
}

static int jzdrm_calculate_size(struct drm_crtc *crtc,
				struct drm_display_mode *mode,
				struct jzdrm_display_size *size)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;

	/*
	 * The rules of f0, f1's position:
	 * f0.x + f0.w <= panel.w;
	 * f0.y + f0.h <= panel.h;
	 */
	if ((jzdrm_crtc->osd.fg0.x + jzdrm_crtc->osd.fg0.w > mode->hdisplay) |
	    (jzdrm_crtc->osd.fg0.y + jzdrm_crtc->osd.fg0.h > mode->vdisplay) |
	    (jzdrm_crtc->osd.fg0.x >= mode->hdisplay) |
	    (jzdrm_crtc->osd.fg0.y >= mode->vdisplay)) {
		dev_err(dev->dev, "Invalid foreground size or position");
		return -EINVAL;
	}

	/* lcd display area */
	size->fg0_line_size = jzdrm_crtc->osd.fg0.w * jzdrm_crtc->osd.fg0.bpp >> 3;
	/* word aligned and in word */
	size->fg0_line_size = ALIGN(size->fg0_line_size, 4) >> 2;
	size->fg0_frm_size = size->fg0_line_size * jzdrm_crtc->osd.fg0.h;

	/* panel PIXEL_ALIGN stride buffer area */
	size->panel_line_size = ALIGN(mode->hdisplay, PIXEL_ALIGN) *
	    (jzdrm_crtc->osd.fg0.bpp >> 3);
	/* word aligned and in word */
	size->panel_line_size = ALIGN(size->panel_line_size, 4) >> 2;
	jzdrm_crtc->frm_size = size->panel_line_size * mode->vdisplay << 2;

	size->height_width = (jzdrm_crtc->osd.fg0.h - 1) << LCDC_DESSIZE_HEIGHT_BIT
	    & LCDC_DESSIZE_HEIGHT_MASK;
	size->height_width |= ((jzdrm_crtc->osd.fg0.w - 1) << LCDC_DESSIZE_WIDTH_BIT
			       & LCDC_DESSIZE_WIDTH_MASK);

	return 0;
}

static void jzdrm_config_tft_lcd_dma(struct drm_crtc *crtc,
			struct jzdrm_display_size *size,
			struct jzdrm_framedesc *framedesc)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);

	framedesc->next = jzdrm_crtc->framedesc_phys;
	framedesc->databuf = jzdrm_crtc->vidmem_phys;
	framedesc->id = 0xda0;

	framedesc->cmd = LCDC_CMD_EOFINT | LCDC_CMD_FRM_EN;
	if (!jzdrm_crtc->osd.block) {
		framedesc->cmd |= size->fg0_frm_size;
		framedesc->offsize =
		    (size->panel_line_size - size->fg0_line_size);
	} else {
		framedesc->cmd |= LCDC_CMD_16X16BLOCK;
		framedesc->cmd |= (jzdrm_crtc->osd.fg0.h & LCDC_CMD_LEN_MASK);
		/* block size */
		/* framedesc->offsize = size->fg0_frm_size; */
	}

	if (framedesc->offsize == 0) {
		framedesc->page_width = 0;
	} else {
		framedesc->page_width = size->fg0_line_size;
	}

	if (1 || (jzdrm_crtc->framedesc[0]->cpos & LCDC_CPOS_ALPHAMD1))
		/* per pixel alpha mode */
		framedesc->cpos = LCDC_CPOS_ALPHAMD1;
	else
		framedesc->cpos = 0;

	switch (jzdrm_crtc->osd.fg0.bpp) {
	case 16:
		framedesc->cpos |= LCDC_CPOS_RGB_RGB565 | LCDC_CPOS_BPP_16;
		break;
	case 30:
		framedesc->cpos |= LCDC_CPOS_BPP_30;
		break;
	default:
		framedesc->cpos |= LCDC_CPOS_BPP_18_24;
		break;
	}

	/* data has not been premultied */
	framedesc->cpos |= LCDC_CPOS_PREMULTI;
	/* coef_sle 0 use 1 */
	framedesc->cpos |= LCDC_CPOS_COEF_SLE_1;
	framedesc->cpos |= (jzdrm_crtc->osd.fg0.y << LCDC_CPOS_YPOS_BIT
			    & LCDC_CPOS_YPOS_MASK);
	framedesc->cpos |= (jzdrm_crtc->osd.fg0.x << LCDC_CPOS_XPOS_BIT
			    & LCDC_CPOS_XPOS_MASK);

	/* fg0 alpha value */
	framedesc->desc_size = 0xff << LCDC_DESSIZE_ALPHA_BIT;
	framedesc->desc_size |= size->height_width;
}

static void jzdrm_config_smart_lcd_dma(struct drm_crtc *crtc,
			  struct jzdrm_display_size *size,
			  struct jzdrm_framedesc *framedesc)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);

	framedesc->next =
	    jzdrm_crtc->framedesc_phys +
	    sizeof(struct jzdrm_framedesc) * (jzdrm_crtc->desc_num - 2);
	framedesc->databuf = jzdrm_crtc->vidmem_phys;
	framedesc->id = 0xda0da0;

	framedesc->cmd = LCDC_CMD_EOFINT | LCDC_CMD_FRM_EN;
	framedesc->cmd |= size->fg0_frm_size;

	if (0 && jzdrm_crtc->framedesc[0]->cpos & LCDC_CPOS_ALPHAMD1)
		/* per pixel alpha mode */
		framedesc->cpos = LCDC_CPOS_ALPHAMD1;
	else
		framedesc->cpos = 0;
	framedesc->offsize = (size->panel_line_size - size->fg0_line_size);
	if (framedesc->offsize == 0) {
		framedesc->page_width = 0;
	} else {
		framedesc->page_width = size->fg0_line_size;
	}

	switch (jzdrm_crtc->osd.fg0.bpp) {
	case 16:
		framedesc->cpos |= LCDC_CPOS_RGB_RGB565 | LCDC_CPOS_BPP_16;
		break;
	case 30:
		framedesc->cpos |= LCDC_CPOS_BPP_30;
		break;
	default:
		framedesc->cpos |= LCDC_CPOS_BPP_18_24;
		break;
	}
	/* data has not been premultied */
	framedesc->cpos |= LCDC_CPOS_PREMULTI;
	/* coef_sle 0 use 1 */
	framedesc->cpos |= LCDC_CPOS_COEF_SLE_1;
	framedesc->cpos |= (jzdrm_crtc->osd.fg0.y << LCDC_CPOS_YPOS_BIT
			    & LCDC_CPOS_YPOS_MASK);
	framedesc->cpos |= (jzdrm_crtc->osd.fg0.x << LCDC_CPOS_XPOS_BIT
			    & LCDC_CPOS_XPOS_MASK);

	/* fg0 alpha value */
	framedesc->desc_size = 0xff << LCDC_DESSIZE_ALPHA_BIT;
	framedesc->desc_size |= size->height_width;

	framedesc[1].next = jzdrm_crtc->framedesc_phys;
	framedesc[1].databuf = 0;
	framedesc[1].id = 0xda0da1;
	framedesc[1].cmd = LCDC_CMD_CMD | LCDC_CMD_FRM_EN | 0;
	framedesc[1].offsize = 0;
	framedesc[1].page_width = 0;
	framedesc[1].cpos = 0;
	framedesc[1].desc_size = 0;

	framedesc[2].next = jzdrm_crtc->framedesc_phys;
	framedesc[2].databuf = jzdrm_crtc->desc_cmd_phys;
	framedesc[2].id = 0xda0da2;
	framedesc[2].offsize = 0;
	framedesc[2].page_width = 0;
	framedesc[2].desc_size = 0;

	/* if connect mipi smart lcd, do not sent command by slcdc, send command by mipi dsi controller. */
#ifdef CONFIG_JZ_MIPI_DSI
	//framedesc[2].databuf = NULL;
	framedesc[2].cmd = LCDC_CMD_CMD | LCDC_CMD_FRM_EN | 0;
	framedesc[2].cpos = 0;
#else	/* CONFIG_JZ_MIPI_DSI */
	/* must to optimize */
	switch (jzdrm_crtc->pdata->smart_config.bus_width) {
		case 8:
			framedesc[2].cmd = LCDC_CMD_CMD | LCDC_CMD_FRM_EN | 1;
			framedesc[2].cpos = 4;
			break;
		case 9:
		case 16:
			framedesc[2].cmd = LCDC_CMD_CMD | LCDC_CMD_FRM_EN | 1;
			framedesc[2].cpos = 2;
			break;
		default:
			framedesc[2].cmd = LCDC_CMD_CMD | LCDC_CMD_FRM_EN | 1;
			framedesc[2].cpos = 1;
			break;
	}
#endif	/* CONFIG_JZ_MIPI_DSI */

}

static void jzdrm_config_fg1_dma(struct drm_crtc *crtc, struct jzdrm_display_size *size)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;

	/*
	 * the descriptor of DMA 1 just init once
	 * and generally no need to use it
	 */
	if (jzdrm_crtc->fg1_framedesc){
		jzdrm_write(dev, LCDC_DA1, jzdrm_crtc->fg1_framedesc->next);
		return;
	}

	jzdrm_crtc->fg1_framedesc = jzdrm_crtc->framedesc[0] + (jzdrm_crtc->desc_num - 1);
	jzdrm_crtc->fg1_framedesc->next =
	    jzdrm_crtc->framedesc_phys +
	    sizeof(struct jzdrm_framedesc) * (jzdrm_crtc->desc_num - 1);

	jzdrm_crtc->fg1_framedesc->databuf = 0;
	jzdrm_crtc->fg1_framedesc->id = 0xda1;
	jzdrm_crtc->fg1_framedesc->cmd = (LCDC_CMD_EOFINT & ~LCDC_CMD_FRM_EN)
	    | size->fg0_frm_size;
	jzdrm_crtc->fg1_framedesc->offsize = 0;
	jzdrm_crtc->fg1_framedesc->page_width = 0;

	/* global alpha mode, data has not been premultied, COEF_SLE is 11 */
	jzdrm_crtc->fg1_framedesc->cpos = LCDC_CPOS_BPP_18_24 | jzdrm_crtc->osd.fg0.y <<
	    LCDC_CPOS_YPOS_BIT | jzdrm_crtc->osd.fg0.x | LCDC_CPOS_PREMULTI
	    | LCDC_CPOS_COEF_SLE_3;

	jzdrm_crtc->fg1_framedesc->desc_size = size->height_width | 0xff <<
	    LCDC_DESSIZE_ALPHA_BIT;

	jzdrm_write(dev, LCDC_DA1, jzdrm_crtc->fg1_framedesc->next);
}

static void jzdrm_config_fg0(struct drm_crtc *crtc, struct drm_display_mode *mode)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct jzdrm_osd_t *osd = &jzdrm_crtc->osd;
	unsigned int rgb_ctrl, cfg, ctrl = 0;

	osd->fg0.fg = 0;
	osd->fg0.bpp = jzdrm_get_controller_bpp(crtc);
	osd->fg0.x = osd->fg0.y = 0;
	osd->fg0.w = mode->hdisplay;
	osd->fg0.h = mode->vdisplay;

	/* OSD mode enable and alpha blending is enabled */
	cfg = LCDC_OSDC_OSDEN | LCDC_OSDC_ALPHAEN;
	cfg |= 1 << 16;		/* once transfer two pixels */

	if (jzdrm_crtc->fmt_order == FORMAT_X8B8G8R8) {
		rgb_ctrl =
		    LCDC_RGBC_RGBFMT | LCDC_RGBC_ODD_BGR | LCDC_RGBC_EVEN_BGR;
	} else {
		/* default: FORMAT_X8R8G8B8 */
		rgb_ctrl =
		    LCDC_RGBC_RGBFMT | LCDC_RGBC_ODD_RGB | LCDC_RGBC_EVEN_RGB;
	}

	jzdrm_write(dev, LCDC_OSDC, cfg);
	jzdrm_write(dev, LCDC_OSDCTRL, ctrl);
	jzdrm_write(dev, LCDC_RGBC, rgb_ctrl);
}

static int jzdrm_prepare_dma_desc(struct drm_crtc *crtc, struct drm_display_mode *mode)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct jzdrm_display_size *display_size;
	struct jzdrm_framedesc *framedesc[MAX_DESC_NUM];
	int i;

	display_size = kzalloc(sizeof(struct jzdrm_display_size), GFP_KERNEL);
	framedesc[0] = kzalloc(sizeof(struct jzdrm_framedesc) *
			       (jzdrm_crtc->desc_num - 1), GFP_KERNEL);
	for (i = 1; i < jzdrm_crtc->desc_num - 1; i++)
		framedesc[i] = framedesc[0] + i;

	jzdrm_calculate_size(crtc, mode, display_size);

	if (jzdrm_crtc->pdata->lcd_type != LCD_TYPE_SLCD) {
		jzdrm_config_tft_lcd_dma(crtc, display_size, framedesc[0]);
	} else {
		jzdrm_config_smart_lcd_dma(crtc, display_size, framedesc[0]);
	}

	for (i = 0; i < jzdrm_crtc->desc_num - 1; i++) {
		jzdrm_crtc->framedesc[i]->next = framedesc[i]->next;
		jzdrm_crtc->framedesc[i]->databuf = framedesc[i]->databuf;
		jzdrm_crtc->framedesc[i]->id = framedesc[i]->id;
		jzdrm_crtc->framedesc[i]->cmd = framedesc[i]->cmd;
		jzdrm_crtc->framedesc[i]->offsize = framedesc[i]->offsize;
		jzdrm_crtc->framedesc[i]->page_width = framedesc[i]->page_width;
		jzdrm_crtc->framedesc[i]->cpos = framedesc[i]->cpos;
		jzdrm_crtc->framedesc[i]->desc_size = framedesc[i]->desc_size;
	}

	if (jzdrm_crtc->pdata->lcd_type != LCD_TYPE_SLCD) {
		jzdrm_write(dev, LCDC_DA0, jzdrm_crtc->framedesc[0]->next);
	} else {
		jzdrm_write(dev, LCDC_DA0, (unsigned int)virt_to_phys((void *)
								     jzdrm_crtc->framedesc[2]));
	}

	jzdrm_config_fg1_dma(crtc, display_size);
	kzfree(framedesc[0]);
	kzfree(display_size);

	return 0;
}

/* Sent a command without data (18-bit bus, 16-bit index) */
static void slcd_send_mcu_command(struct drm_crtc *crtc, unsigned long cmd)
{
	struct drm_device *dev = crtc->dev;
	int count = 10000;
	while ((jzdrm_read(dev, SLCDC_STATE) & SLCDC_STATE_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dev->dev, "SLCDC wait busy state wrong");
	}
	jzdrm_write(dev, SLCDC_DATA, SLCDC_DATA_RS_COMMAND | cmd);
}

static void slcd_send_mcu_data(struct drm_crtc *crtc, unsigned long data)
{
	struct drm_device *dev = crtc->dev;
	int count = 10000;

	while ((jzdrm_read(dev, SLCDC_STATE) & SLCDC_STATE_BUSY) && count--) {
		udelay(10);
	}
	if (count < 0) {
		dev_err(dev->dev, "SLCDC wait busy state wrong");
	}

	jzdrm_write(dev, SLCDC_DATA, SLCDC_DATA_RS_DATA | data);
}

static void jzdrm_slcd_mcu_init(struct drm_crtc *crtc)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct jzfb_platform_data *pdata = jzdrm_crtc->pdata;
	struct drm_device *dev = crtc->dev;
	unsigned int is_lcd_en, i;

	if (pdata->lcd_type != LCD_TYPE_SLCD)
		return;

	is_lcd_en = jzdrm_crtc->is_lcd_en;
	start(crtc);

#ifndef CONFIG_GPIO_SIMULATE
	if (pdata->smart_config.gpio_for_slcd) {
		pdata->smart_config.gpio_for_slcd();
	}
#endif
	/*
	 *set cmd_width and data_width
	 * */
	if (pdata->smart_config.length_data_table
	    && pdata->smart_config.data_table) {
		for (i = 0; i < pdata->smart_config.length_data_table; i++) {
			switch (pdata->smart_config.data_table[i].type) {
			case SMART_CONFIG_DATA:
				slcd_send_mcu_data(
					crtc,
					pdata->smart_config.data_table[i].value);
				break;
			case SMART_CONFIG_CMD:
				slcd_send_mcu_command(
					crtc,
					pdata->smart_config.data_table[i].value);
				break;
			case SMART_CONFIG_UDELAY:
				udelay(pdata->smart_config.data_table[i].value);
				break;
			default:
				dev_err(dev->dev, "Unknow SLCD data type\n");
				break;
			}
		}
		{
			int count = 10000;
			while ((jzdrm_read(dev, SLCDC_STATE) & SLCDC_STATE_BUSY)
			       && count--) {
				udelay(10);
			}
			if (count < 0) {
				dev_err(dev->dev,
					"SLCDC wait busy state wrong");
			}

		}

	}

	if(pdata->bpp / pdata->smart_config.bus_width != 1 ) {
		int tmp = jzdrm_read(dev, SLCDC_CFG_NEW);
		tmp &= ~(SMART_LCD_DWIDTH_MASK); //mask the 8~9bit
		tmp |=  (pdata->bpp / pdata->smart_config.bus_width)  == 2 ? SMART_LCD_NEW_DTIMES_TWICE : SMART_LCD_NEW_DTIMES_THICE;
		jzdrm_write(dev, SLCDC_CFG_NEW, tmp);
		dev_dbg(dev->dev, "the slcd  slcd_cfg_new is %08x\n", tmp);
	}

#ifdef CONFIG_FB_JZ_DEBUG
	/*for register mode test,
	 * you can write test code according to the lcd panel
	 **/
#endif

	/*recovery ori status*/
	if (!is_lcd_en) {
		stop(crtc);
	}

}

int jzdrm_mode_valid(struct drm_crtc *crtc, struct drm_display_mode *mode)
{
	struct jzdrm_drm_private *priv = crtc->dev->dev_private;
	unsigned int bandwidth;
	uint32_t hbp, hfp, hsw, vbp, vfp, vsw;

	/*
	 * check to see if the width is within the range that
	 * the LCD Controller physically supports
	 */
	if (mode->hdisplay > 2048)
		return MODE_VIRTUAL_X;

	/* width must be multiple of 16 */
	if (mode->hdisplay & 0xf)
		return MODE_VIRTUAL_X;

	if (mode->vdisplay > 2048)
		return MODE_VIRTUAL_Y;

	DRM_DEBUG_DRIVER("Processing mode %dx%d@%d with pixel clock %d",
		mode->hdisplay, mode->vdisplay,
		drm_mode_vrefresh(mode), mode->clock);

	hbp = mode->htotal - mode->hsync_end;
	hfp = mode->hsync_start - mode->hdisplay;
	hsw = mode->hsync_end - mode->hsync_start;
	vbp = mode->vtotal - mode->vsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	vsw = mode->vsync_end - mode->vsync_start;
/*
	if ((hbp-1) & ~0x3ff) {
		DRM_DEBUG_DRIVER("Prune: Horizontal Back Porch out of range");
		return MODE_HBLANK_WIDE;
	}

	if ((hfp-1) & ~0x3ff) {
		DRM_DEBUG_DRIVER("Prune: Horizontal Front Porch out of range");
		return MODE_HBLANK_WIDE;
	}

	if ((hsw-1) & ~0x3ff) {
		DRM_DEBUG_DRIVER("Prune: Horizontal Sync Width out of range");
		return MODE_HSYNC_WIDE;
	}

	if (vbp & ~0xff) {
		DRM_DEBUG_DRIVER("Prune: Vertical Back Porch out of range");
		return MODE_VBLANK_WIDE;
	}

	if (vfp & ~0xff) {
		DRM_DEBUG_DRIVER("Prune: Vertical Front Porch out of range");
		return MODE_VBLANK_WIDE;
	}

	if ((vsw-1) & ~0x3f) {
		DRM_DEBUG_DRIVER("Prune: Vertical Sync Width out of range");
		return MODE_VSYNC_WIDE;
	}
*/
	/*
	 * some devices have a maximum allowed pixel clock
	 * configured from the DT
	 */
	if (mode->clock > priv->max_pixelclock) {
		DRM_DEBUG_DRIVER("Prune: pixel clock too high");
		return MODE_CLOCK_HIGH;
	}

	/*
	 * some devices further limit the max horizontal resolution
	 * configured from the DT
	 */
	if (mode->hdisplay > priv->max_width) {
		DRM_DEBUG_DRIVER("Prune: Bad width");
		return MODE_BAD_WIDTH;
	}

	/* filter out modes that would require too much memory bandwidth: */
	bandwidth = mode->hdisplay * mode->vdisplay *
		drm_mode_vrefresh(mode);
	if (bandwidth > priv->max_bandwidth) {
		DRM_DEBUG_DRIVER("Prune: exceeds defined bandwidth limit %d",
				 bandwidth);
		return MODE_BAD;
	}

	return MODE_OK;
}

static void set_scanout(struct drm_crtc *crtc, int n)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct drm_display_mode *mode = jzdrm_crtc->mode;
	struct jzdrm_drm_private *priv = dev->dev_private;
	struct drm_framebuffer *fb = crtc->fb;
	struct drm_gem_cma_object *gem;

	gem = drm_fb_cma_get_gem_obj(fb, 0);
	jzdrm_crtc->vidmem_phys = gem->paddr;

	pm_runtime_get_sync(dev->dev);

	jzdrm_config_fg0(crtc, mode);
	jzdrm_prepare_dma_desc(crtc, jzdrm_crtc->mode);

	if (jzdrm_crtc->scanout[n]) {
		drm_flip_work_queue(&jzdrm_crtc->unref_work,
				    jzdrm_crtc->scanout[n]);
		drm_flip_work_commit(&jzdrm_crtc->unref_work, priv->wq);
	}
	jzdrm_crtc->scanout[n] = crtc->fb;
	drm_framebuffer_reference(jzdrm_crtc->scanout[n]);
	pm_runtime_put_sync(dev->dev);
}

static void jzdrm_vsync(struct jzdrm_crtc* jzdrm_crtc)
{
	if (jzdrm_crtc->pdata->lcd_type != LCD_TYPE_SLCD) {
			/* report vsync to android hwcomposer */
		wmb();
		jzdrm_crtc->timestamp.value[jzdrm_crtc->timestamp.wp] =
			ktime_to_ns(ktime_get());
		jzdrm_crtc->timestamp.wp = (jzdrm_crtc->timestamp.wp + 1) % TIMESTAMP_CAP;
			wake_up_interruptible(&jzdrm_crtc->vsync_wq);
	}
}

static int jzdrm_crtc_ioctl(struct drm_crtc *crtc, struct fb_info *info, unsigned int cmd, unsigned long arg){
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	void __user *argp = (void __user *)arg;
	unsigned int value;
	int  ret, tmp;

	switch(cmd){
	case FBIO_WAITFORVSYNC:
		if (likely(jzdrm_crtc->timestamp.wp == jzdrm_crtc->timestamp.rp)) {
			unlock_fb_info(info);
			interruptible_sleep_on(&jzdrm_crtc->vsync_wq);
			lock_fb_info(info);
		} else {
			printk("<7>send vsync!\n");
		}

		ret = copy_to_user(argp, jzdrm_crtc->timestamp.value + jzdrm_crtc->timestamp.rp,
				sizeof(u64));
		jzdrm_crtc->timestamp.rp = (jzdrm_crtc->timestamp.rp + 1) % TIMESTAMP_CAP;

		if (unlikely(ret))
			return -EFAULT;
		break;
	case JZFB_SET_VSYNCINT:
		if (unlikely(copy_from_user(&value, argp, sizeof(int))))
			return -EFAULT;
		if (value) {

	                /* clear previous EOF flag */
                        tmp = jzdrm_read(dev, LCDC_STATE);

			jzdrm_write(dev, LCDC_STATE, tmp & ~LCDC_STATE_EOF);
			/* enable end of frame interrupt */
			tmp = jzdrm_read(dev, LCDC_CTRL);
			jzdrm_write(dev, LCDC_CTRL, tmp | LCDC_CTRL_EOFM);
		}
		else {
			tmp = jzdrm_read(dev, LCDC_CTRL);
			jzdrm_write(dev, LCDC_CTRL, tmp & (~LCDC_CTRL_EOFM));
		}
                break;
	default:
		break;
	}
	return 0;
}

static void update_scanout(struct drm_crtc *crtc)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;

	if (jzdrm_crtc->dpms == DRM_MODE_DPMS_ON) {
		drm_vblank_get(dev, 0);
		jzdrm_vsync(jzdrm_crtc);
	} else {
		/* not enabled yet, so update registers immediately: */
//		jzdrm_write(dev, LCDC_STATE, 0);
//		set_scanout(crtc, 0);
	}
}

static int jzdrm_lcd_mode_set(struct drm_crtc *crtc, struct drm_display_mode *mode)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct jzfb_platform_data *pdata = jzdrm_crtc->pdata;
	struct fb_videomode *video_modes = pdata->modes;

	uint32_t hbp, hfp, hsw, vbp, vfp, vsw;
	int ret;
	uint32_t pcfg;

	uint16_t hds, vds;
	uint16_t hde, vde;
	uint16_t ht, vt;
	uint32_t cfg, ctrl;

	uint32_t smart_cfg = 0, smart_ctrl = 0;
	uint32_t smart_new_cfg = 0;
	uint32_t smart_wtime = 0, smart_tas = 0;


	ret = jzdrm_mode_valid(crtc, mode);
	if (WARN_ON(ret))
		return ret;

	/* Configure timings: */
	hbp = mode->htotal - mode->hsync_end;
	hfp = mode->hsync_start - mode->hdisplay;
	hsw = mode->hsync_end - mode->hsync_start;
	vbp = mode->vtotal - mode->vsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	vsw = mode->vsync_end - mode->vsync_start;

	hds = hsw + hbp;
	hde = hds + mode->hdisplay;
	ht = hde + hfp;

	vds = vsw + vbp;
	vde = vds + mode->vdisplay;
	vt = vde + vfp;

	cfg = LCDC_CFG_NEWDES | LCDC_CFG_RECOVER;
	cfg |= pdata->lcd_type;
	if (!(video_modes->sync & FB_SYNC_HOR_HIGH_ACT))
		cfg |= LCDC_CFG_HSP;

	if (!(video_modes->sync & FB_SYNC_VERT_HIGH_ACT))
		cfg |= LCDC_CFG_VSP;

	if (pdata->pixclk_falling_edge)
		cfg |= LCDC_CFG_PCP;

	if (pdata->data_enable_active_low)
		cfg |= LCDC_CFG_DEP;
	cfg |= LCDC_CFG_PSM;
	cfg |= LCDC_CFG_CLSM;
	cfg |= LCDC_CFG_SPLM;
	cfg |= LCDC_CFG_REVM;

	ctrl = LCDC_CTRL_BST_64 | LCDC_CTRL_OFUM | LCDC_CTRL_EOFM;
	if (pdata->pinmd)
		ctrl |= LCDC_CTRL_PINMD;

	/* magic number */
	pcfg = 0xC0000000 | (511<<18) | (400<<9) | (256<<0);

	if (pdata->lcd_type == LCD_TYPE_SLCD) {
		smart_cfg = pdata->smart_config.smart_type | SMART_LCD_DWIDTH_24_BIT_ONCE_PARALLEL;

		switch(pdata->smart_config.bus_width){
		case 8:
			smart_cfg |= SMART_LCD_CWIDTH_8_BIT_ONCE;
			smart_new_cfg |= SMART_LCD_NEW_DWIDTH_8_BIT;
			break;
		case 9:
			smart_cfg |= SMART_LCD_CWIDTH_9_BIT_ONCE;
			smart_new_cfg |= SMART_LCD_NEW_DWIDTH_9_BIT;
			break;
		case 16:
			smart_cfg |= SMART_LCD_CWIDTH_16_BIT_ONCE;
			smart_new_cfg |= SMART_LCD_NEW_DWIDTH_16_BIT;
			break;
		case 18:
			smart_cfg |= SMART_LCD_CWIDTH_18_BIT_ONCE;
			smart_new_cfg |= SMART_LCD_NEW_DWIDTH_18_BIT;
			break;
		case 24:
			smart_cfg |= SMART_LCD_CWIDTH_24_BIT_ONCE;
			smart_new_cfg |= SMART_LCD_NEW_DWIDTH_24_BIT;
			break;
		default:
			printk("ERR: please check out your bus width config\n");
			break;
		}

		if (pdata->smart_config.clkply_active_rising)
			smart_cfg |= SLCDC_CFG_CLK_ACTIVE_RISING;
		if (pdata->smart_config.rsply_cmd_high)
			smart_cfg |= SLCDC_CFG_RS_CMD_HIGH;
		if (pdata->smart_config.csply_active_high)
			smart_cfg |= SLCDC_CFG_CS_ACTIVE_HIGH;

		smart_ctrl = SLCDC_CTRL_DMA_MODE;
		//smart_ctrl |= SLCDC_CTRL_GATE_MASK; //for saving power
		smart_ctrl &= ~SLCDC_CTRL_GATE_MASK;

		smart_ctrl |= (SLCDC_CTRL_NEW_MODE | SLCDC_CTRL_NOT_USE_TE); //new slcd mode
		smart_ctrl &= ~SLCDC_CTRL_MIPI_MODE;
		smart_new_cfg |= SMART_LCD_NEW_DTIMES_ONCE;

		if (pdata->smart_config.newcfg_6800_md)
			smart_new_cfg |= SLCDC_NEW_CFG_6800_MD;
		if (pdata->smart_config.datatx_type_serial
		    && pdata->smart_config.cmdtx_type_serial)
			smart_new_cfg |=
			    SLCDC_NEW_CFG_DTYPE_SERIAL |
			    SLCDC_NEW_CFG_CTYPE_SERIAL;
		if (pdata->smart_config.newcfg_cmd_9bit)
			smart_new_cfg |= SLCDC_NEW_CFG_CMD_9BIT;

		smart_wtime = 0;
		smart_tas = 0;
	}

	if (pdata->lcd_type == LCD_TYPE_SLCD) {
#ifdef CONFIG_JZ_MIPI_DSI
		smart_cfg |= 1 << 16;
		smart_new_cfg |= 4 << 13;
		smart_ctrl |= 1 << 7 | 1 << 6;
#endif
		jzdrm_write(dev, SLCDC_CFG, smart_cfg);
		jzdrm_write(dev, SLCDC_CTRL, smart_ctrl);

		jzdrm_write(dev, SLCDC_CFG_NEW, smart_new_cfg);
		jzdrm_write(dev, SLCDC_WTIME, smart_wtime);
		jzdrm_write(dev, SLCDC_TAS, smart_tas);

	}
	jzdrm_write(dev, LCDC_VAT, (ht << 16) | vt);
	jzdrm_write(dev, LCDC_DAH, (hds << 16) | hde);
	jzdrm_write(dev, LCDC_DAV, (vds << 16) | vde);

	jzdrm_write(dev, LCDC_HSYNC, hsw);
	jzdrm_write(dev, LCDC_VSYNC, vsw);

	jzdrm_write(dev, LCDC_CFG, cfg);
	ctrl |= jzdrm_read(dev, LCDC_CTRL);
	jzdrm_write(dev, LCDC_CTRL, ctrl);
	jzdrm_write(dev, LCDC_PCFG, pcfg);

	//jzfb_config_image_enh(info);

	if (pdata->lcd_type == LCD_TYPE_SLCD) {
		jzdrm_slcd_mcu_init(crtc);

#ifdef CONFIG_SLCDC_CONTINUA
		smart_ctrl &= ~SLCDC_CTRL_DMA_MODE;
#else
		smart_ctrl |= SLCDC_CTRL_DMA_START;
#endif
		smart_ctrl |= SLCDC_CTRL_DMA_EN;

#ifdef CONFIG_SLCDC_USE_TE
		smart_ctrl &= ~SLCDC_CTRL_NOT_USE_TE;
#endif

		if (pdata->smart_config.newcfg_fmt_conv) {
			smart_new_cfg = jzdrm_read(dev, SLCDC_CFG_NEW);
			smart_new_cfg |= SLCDC_NEW_CFG_FMT_CONV_EN;
			jzdrm_write(dev, SLCDC_CFG_NEW, smart_new_cfg);
		}
		jzdrm_write(dev, SLCDC_CTRL, smart_ctrl);
	}
/*
#ifdef CONFIG_JZ_MIPI_DSI
	else {
		cfg |= 1 << 24;
		jzdrm_write(dev, LCDC_CFG, cfg);
		jzdrm_crtc->dsi->master_ops->video_cfg(jzdrm_crtc->dsi);
	}
#endif
*/
	return 0;
}

static void jzdrm_destroy(struct drm_crtc *crtc)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);

	WARN_ON(jzdrm_crtc->dpms == DRM_MODE_DPMS_ON);

	drm_crtc_cleanup(crtc);
	drm_flip_work_cleanup(&jzdrm_crtc->unref_work);
#ifdef CONFIG_JZ_MIPI_DSI_MODE1
	jzdsi_remove(jzdrm_crtc->dsi);
#endif

	kfree(jzdrm_crtc);
}

static int jzdrm_page_flip(struct drm_crtc *crtc,
		struct drm_framebuffer *fb,
		struct drm_pending_vblank_event *event)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;

	if (jzdrm_crtc->event) {
		dev_err(dev->dev, "already pending page flip!\n");
		return -EBUSY;
	}

	crtc->fb = fb;
	jzdrm_crtc->event = event;
	update_scanout(crtc);

	return 0;
}

void jzdrm_update_clk(struct drm_crtc *crtc)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct jzdrm_drm_private *priv = dev->dev_private;
	struct jzfb_platform_data *pdata = jzdrm_crtc->pdata;
	unsigned int lcd_clk;
	unsigned long rate;
	int ret;

	pm_runtime_get_sync(dev->dev);

	/* in raster mode, minimum divisor is 2: */
	rate = crtc->mode.clock*1000;

#ifndef CONFIG_JZ_MIPI_DSI
	/* smart lcd WR freq = (lcd pixel clock)/2 */
	if (pdata->lcd_type == LCD_TYPE_SLCD) {
		rate *= 2;
	}
#endif
	ret = clk_set_rate(priv->disp_clk, rate);
	if (ret) {
		dev_err(dev->dev, "failed to set display clock rate to: %d\n",
				crtc->mode.clock);
		goto out;
	}

	lcd_clk = clk_get_rate(priv->clk);

	DRM_DEBUG_DRIVER("lcd_clk=%u, mode clock=%d", lcd_clk,
			  crtc->mode.clock);
	DRM_DEBUG_DRIVER("fck=%lu, dpll_disp_ck=%lu", clk_get_rate(priv->clk),
			 clk_get_rate(priv->disp_clk));

out:
	pm_runtime_put_sync(dev->dev);
}

static void jzdrm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct drm_display_mode *display_mode = &crtc->mode;

	/* we really only care about on or off: */
	if (mode != DRM_MODE_DPMS_ON)
		mode = DRM_MODE_DPMS_OFF;

	if (jzdrm_crtc->dpms == mode)
		return;

	jzdrm_crtc->dpms = mode;

	pm_runtime_get_sync(dev->dev);

	if (mode == DRM_MODE_DPMS_ON) {
		pm_runtime_forbid(dev->dev);
		jzdrm_clk_enable(crtc);
		stop(crtc);
#ifdef CONFIG_JZ_MIPI_DSI_MODE1
		jzdrm_crtc->dsi->master_ops->set_blank_mode(jzdrm_crtc->dsi, FB_BLANK_UNBLANK);
#endif
#ifdef CONFIG_JZ_MIPI_DSI_MODE1
	    if (jzdrm_crtc->dsi->master_ops->ioctl)
		jzdrm_crtc->dsi->master_ops->ioctl(jzdrm_crtc->dsi, CMD_MIPI_DISPLAY_ON);
#endif
		jzdrm_lcd_mode_set(crtc, display_mode);
		set_scanout(crtc, 0);
		jzdrm_update_clk(crtc);
		start(crtc);
	} else {
		jzdrm_crtc->frame_done = false;
#ifdef CONFIG_JZ_MIPI_DSI_MODE1
		jzdrm_crtc->dsi->master_ops->set_blank_mode(jzdrm_crtc->dsi, FB_BLANK_POWERDOWN);
#endif
		stop(crtc);
		jzdrm_clk_disable(crtc);
		pm_runtime_allow(dev->dev);
	}
	pm_runtime_put_sync(dev->dev);
}

static bool jzdrm_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_DRIVER("Mode Fixup not supported by driver yet\n");
	return true;
}

static void jzdrm_prepare(struct drm_crtc *crtc)
{

	//jzdrm_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);
}

static void jzdrm_commit(struct drm_crtc *crtc)
{

	//jzdrm_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
}

static int jzdrm_mode_set(struct drm_crtc *crtc,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode,
		int x, int y,
		struct drm_framebuffer *old_fb)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	int next_frm;
	//int ret;

	jzdrm_crtc->mode = mode;
	pm_runtime_get_sync(dev->dev);
/*
	jzdrm_clk_enable(crtc);
	ret = jzdrm_lcd_mode_set(crtc, mode);
	if (WARN_ON(ret))
		return ret;

	update_scanout(crtc);
	set_scanout(crtc, 0);
	jzdrm_update_clk(crtc);
	jzdrm_clk_disable(crtc);

*/
	next_frm = y / mode->vdisplay;
	if (jzdrm_crtc->pdata->lcd_type != LCD_TYPE_INTERLACED_TV &&
	    jzdrm_crtc->pdata->lcd_type != LCD_TYPE_SLCD) {
		if (!jzdrm_crtc->osd.block) {
			jzdrm_crtc->framedesc[0]->databuf = jzdrm_crtc->vidmem_phys
			    + jzdrm_crtc->frm_size * next_frm;
		} else {
			/* 16x16 block mode */
		}
	} else if (jzdrm_crtc->pdata->lcd_type == LCD_TYPE_SLCD) {
		/*
		struct slcd_te *te;
		te = &jzdrm_crtc->slcd_te;

		jzdrm_crtc->framedesc[0]->databuf = jzdrm_crtc->vidmem_phys
		    + jzdrm_crtc->frm_size * next_frm;
		if (!jzdrm_crtc->is_lcd_en)
			return -EINVAL;
		*/
	}

	pm_runtime_put_sync(dev->dev);

	return 0;
}

static const struct drm_crtc_funcs jzdrm_funcs = {
		.destroy        = jzdrm_destroy,
		.set_config     = drm_crtc_helper_set_config,
		.page_flip      = jzdrm_page_flip,
		.ioctl          = jzdrm_crtc_ioctl,
};

static const struct drm_crtc_helper_funcs jzdrm_helper_funcs = {
		.dpms           = jzdrm_crtc_dpms,
		.mode_fixup     = jzdrm_mode_fixup,
		.prepare        = jzdrm_prepare,
		.commit         = jzdrm_commit,
		.mode_set       = jzdrm_mode_set,
};

int jzdrm_max_width(struct drm_crtc *crtc)
{
	return 2048;
}

irqreturn_t jzdrm_crtc_irq(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	unsigned int state;
	unsigned int tmp;

	state = jzdrm_read(dev, LCDC_STATE);

	if (state & LCDC_STATE_EOF) {
		jzdrm_write(dev, LCDC_STATE, state & ~LCDC_STATE_EOF);
		update_scanout(crtc);
	}

	if (state & LCDC_STATE_OFU) {
		DRM_DEBUG_DRIVER("Out FiFo underrun\n");
		jzdrm_write(dev, LCDC_STATE, state & ~LCDC_STATE_OFU);
		tmp = jzdrm_read(dev, LCDC_CTRL);
		jzdrm_write(dev, LCDC_CTRL, tmp & ~LCDC_CTRL_OFUM);
		update_scanout(crtc);
		start(crtc);
	}

	return IRQ_HANDLED;
}

void jzdrm_crtc_cancel_page_flip(struct drm_crtc *crtc, struct drm_file *file)
{
	struct jzdrm_crtc *jzdrm_crtc = to_jzdrm_crtc(crtc);
	struct drm_pending_vblank_event *event;
	struct drm_device *dev = crtc->dev;
	unsigned long flags;

	/* Destroy the pending vertical blanking event associated with the
	 * pending page flip, if any, and disable vertical blanking interrupts.
	 */
	spin_lock_irqsave(&dev->event_lock, flags);
	event = jzdrm_crtc->event;
	if (event && event->base.file_priv == file) {
		jzdrm_crtc->event = NULL;
		event->base.destroy(&event->base);
		drm_vblank_put(dev, 0);
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

static int jzdrm_alloc_devmem(struct drm_device *dev, struct jzdrm_crtc *jzdrm_crtc)
{
	int i;

	jzdrm_crtc->framedesc[0] =
	    dma_alloc_coherent(dev->dev,
			       sizeof(struct jzdrm_framedesc) * jzdrm_crtc->desc_num,
			       &jzdrm_crtc->framedesc_phys, GFP_KERNEL);
	if (!jzdrm_crtc->framedesc[0])
		return -ENOMEM;
	for (i = 1; i < jzdrm_crtc->desc_num; i++)
		jzdrm_crtc->framedesc[i] = jzdrm_crtc->framedesc[0] + i;

	if (jzdrm_crtc->pdata->lcd_type == LCD_TYPE_SLCD) {
		unsigned long *ptr;
		jzdrm_crtc->desc_cmd_vidmem = dma_alloc_coherent(dev->dev, PAGE_SIZE,
							   &jzdrm_crtc->desc_cmd_phys,
							   GFP_KERNEL);
		if (!jzdrm_crtc->desc_cmd_vidmem)
			return -ENOMEM;
		ptr = (unsigned long *)jzdrm_crtc->desc_cmd_vidmem;
		for (i = 0; i < jzdrm_crtc->pdata->smart_config.length_cmd; i++) {
			ptr[i] = jzdrm_crtc->pdata->smart_config.write_gram_cmd[i];
		}
	}

	return 0;
}

struct drm_crtc *jzdrm_crtc_create(struct drm_device *dev)
{
	struct jzdrm_drm_private *priv = dev->dev_private;
	struct platform_device *pdev = dev->platformdev;
	struct jzfb_platform_data *pdata = pdev->dev.platform_data;
	struct jzdrm_crtc *jzdrm_crtc;
	struct drm_crtc *crtc;
	int ret;

	jzdrm_crtc = kzalloc(sizeof(*jzdrm_crtc), GFP_KERNEL);
	if (!jzdrm_crtc) {
		dev_err(jzdrm_crtc->base.dev->dev, "allocation failed\n");
		return NULL;
	}

	crtc = &jzdrm_crtc->base;
	jzdrm_crtc->pdata = pdata;
	jzdrm_crtc->mmio = priv->mmio;
	jzdrm_crtc->dpms = DRM_MODE_DPMS_OFF;
	jzdrm_crtc->is_lcd_en = 1;

        jzdrm_crtc->fmt_order = FORMAT_X8R8G8B8;

	if (pdata->lcd_type != LCD_TYPE_INTERLACED_TV &&
	    pdata->lcd_type != LCD_TYPE_SLCD) {
		jzdrm_crtc->desc_num = MAX_DESC_NUM - 2;
	} else {
		jzdrm_crtc->desc_num = MAX_DESC_NUM;
	}

	ret = jzdrm_alloc_devmem(dev, jzdrm_crtc);
	if (ret) {
		dev_err(dev->dev, "desc allocation failed\n");
		return NULL;
	}

	init_waitqueue_head(&jzdrm_crtc->frame_done_wq);

	ret = drm_flip_work_init(&jzdrm_crtc->unref_work, 16,
			"unref", unref_worker);
	if (ret) {
		dev_err(dev->dev, "could not allocate unref FIFO\n");
		goto fail;
	}

	init_waitqueue_head(&jzdrm_crtc->vsync_wq);
	jzdrm_crtc->timestamp.rp = 0;
	jzdrm_crtc->timestamp.wp = 0;

	ret = drm_crtc_init(dev, crtc, &jzdrm_funcs);
	if (ret < 0)
		goto fail;

	drm_crtc_helper_add(crtc, &jzdrm_helper_funcs);

#ifdef CONFIG_JZ_MIPI_DSI_MODE1
	jzdrm_crtc->dsi = jzdsi_init(pdata->dsi_pdata);
	if (!jzdrm_crtc->dsi) {
		goto fail;
	}
#endif

	return crtc;

fail:
	jzdrm_destroy(crtc);
	return NULL;
}
