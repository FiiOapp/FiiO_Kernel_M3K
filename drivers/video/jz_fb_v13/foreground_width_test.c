
#define FOREGROUND_WIDTH_TEST

static unsigned int *buffer_for_partial;
static struct jzfb_framedesc *buffer_partial_desc;

static void jzfb_display_v_color_bar_test(struct fb_info *info, int w, int h)
{
	int i, j;
	int bpp;
	unsigned short *p16;
	unsigned int *p32;
	struct jzfb *jzfb = info->par;
	struct fb_videomode *mode = jzfb->pdata->modes;

	if (!mode) {
		dev_err(jzfb->dev, "%s, video mode is NULL\n", __func__);
		return;
	}
	if (!jzfb->vidmem_phys) {
		dev_err(jzfb->dev, "Not allocate frame buffer yet\n");
		return;
	}
	if (!jzfb->vidmem)
		jzfb->vidmem = (void *)(jzfb->vidmem_phys+0x80000000);
	p16 = (unsigned short *)jzfb->vidmem;
	p32 = (unsigned int *)jzfb->vidmem;
	bpp = jzfb->osd.fg0.bpp;

	dev_info(info->dev,
		 "LCD V COLOR BAR w,h,bpp(%d,%d,%d) jzfb->vidmem=%p\n", w, h,
		 bpp, jzfb->vidmem);

	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			short c16;
			int c32 = 0;
			switch ((j / 10) % 4) {
			case 0:
				c16 = 0xF800;
				c32 = 0xFFFF0000;
				break;
			case 1:
				c16 = 0x07C0;
				c32 = 0xFF00FF00;
				break;
			case 2:
				c16 = 0x001F;
				c32 = 0xFF0000FF;
				break;
			default:
				c16 = 0xFFFF;
				c32 = 0xFFFFFFFF;
				break;
			}
			switch (bpp) {
			case 18:
			case 24:
			case 32:
				*p32++ = c32;
				break;
			default:
				*p16++ = c16;
			}
		}
		if (w % PIXEL_ALIGN) {
			switch (bpp) {
			case 18:
			case 24:
			case 32:
				p32 += (ALIGN(mode->xres, PIXEL_ALIGN) - w);
				break;
			default:
				p16 += (ALIGN(mode->xres, PIXEL_ALIGN) - w);
				break;
			}
		}
	}
}

static void regw(uint32_t reg, char logic, uint32_t value)
{
	unsigned int tmp = 0;

        tmp = reg_read(jzfb, reg);
        if (logic == 'a') {
                tmp &= ~(value);
        } else {
                tmp |= value;
        }
        reg_write(jzfb, reg, tmp);
}

static void dma_start(struct jzfb *jzfb)
{
	unsigned int tmp = 0;

        reg_write(jzfb, LCDC_DA0, (uint32_t)jzfb->framedesc[2] - 0xa0000000);

        tmp = reg_read(jzfb, SLCDC_CTRL);
        tmp |= SLCDC_CTRL_DMA_START | SLCDC_CTRL_DMA_MODE;
        reg_write(jzfb, SLCDC_CTRL, tmp);

        jzfb_enable(jzfb->fb);
        msleep(200);
        jzfb_disable(jzfb->fb);
}

static void slcd_send_partial_prm(struct jzfb *jzfb, uint16_t start_x, uint16_t start_y,
		       uint16_t dis_width, uint16_t dis_height)
{
	uint32_t tmp;

	tmp = reg_read(jzfb, SLCDC_CFG_NEW);
        tmp &= ~SLCDC_NEW_CFG_FMT_CONV_EN;
        reg_write(jzfb, SLCDC_CFG_NEW, tmp);

        tmp = reg_read(jzfb, SLCDC_CTRL);
        tmp &= ~SLCDC_CTRL_DMA_EN;
        reg_write(jzfb, SLCDC_CTRL, tmp);

        tmp = reg_read(jzfb, SLCDC_CFG_NEW);
        tmp &= ~(SLCDC_NEW_CFG_DTIME_MASK);
        tmp |= SLCDC_NEW_CFG_DTIME_ONCE;
        reg_write(jzfb, SLCDC_CFG_NEW, tmp);

        /* tmp = reg_read(jzfb, SLCDC_CTRL); */
        /* tmp |= SLCDC_CTRL_FAST_MODE; */
        /* reg_write(jzfb, SLCDC_CTRL, tmp); */

        jzfb_enable(jzfb->fb);

        slcd_send_mcu_command(jzfb, 0x2a);
	slcd_send_mcu_data(jzfb, 0xff & (start_x >> 8));
	slcd_send_mcu_data(jzfb, 0xff & start_x);
	slcd_send_mcu_data(jzfb, 0xff & ((start_x + dis_width - 1) >> 8));
	slcd_send_mcu_data(jzfb, 0xff & (start_x + dis_width - 1));

	slcd_send_mcu_command(jzfb, 0x2b);
	slcd_send_mcu_data(jzfb, 0xff & (start_y >> 8));
	slcd_send_mcu_data(jzfb, 0xff & start_y);
	slcd_send_mcu_data(jzfb, 0xff & ((start_y + dis_height - 1) >> 8));
	slcd_send_mcu_data(jzfb, 0xff & (start_y + dis_height - 1));

	slcd_send_mcu_command(jzfb, 0x2c);
	while ((reg_read(jzfb, SLCDC_STATE) & SLCDC_STATE_BUSY));

        jzfb_disable(jzfb->fb);

	tmp = reg_read(jzfb, SLCDC_CFG_NEW);
        tmp |= SLCDC_NEW_CFG_FMT_CONV_EN;
        reg_write(jzfb, SLCDC_CFG_NEW, tmp);

        tmp = reg_read(jzfb, SLCDC_CFG_NEW);
        tmp &= ~(SLCDC_NEW_CFG_DTIME_MASK);
        tmp |= SLCDC_NEW_CFG_DTIME_TWICE;
        reg_write(jzfb, SLCDC_CFG_NEW, tmp);

        tmp = reg_read(jzfb, SLCDC_CTRL);
        tmp |= SLCDC_CTRL_DMA_EN;
        reg_write(jzfb, SLCDC_CTRL, tmp);

        reg_write(jzfb, LCDC_VAT, (dis_width << 16) | dis_height);
        reg_write(jzfb, LCDC_DAH, dis_width);
        reg_write(jzfb, LCDC_DAV, dis_height);
}

static void clear_screen(struct jzfb *jzfb)
{
        slcd_send_partial_prm(jzfb, 0, 0, 240, 240);

        jzfb->framedesc[2]->next = (uint32_t)jzfb->framedesc[0] - 0xa0000000;

        /* jzfb->framedesc[0]->databuf = jzfb->vidmem_phys + jzfb->frm_size * 0; */
        /* dma_start(jzfb); */

        jzfb->framedesc[0]->databuf = jzfb->vidmem_phys + jzfb->frm_size * 1;
        dma_start(jzfb);

        /* jzfb->framedesc[0]->databuf = jzfb->vidmem_phys + jzfb->frm_size * 2; */
        /* dma_start(jzfb); */

        /* jzfb->framedesc[0]->databuf = jzfb->vidmem_phys + jzfb->frm_size * 3; */
        /* dma_start(jzfb); */

        /* dump_lcdc_registers(jzfb); */
}

static ssize_t test_partial_refresh(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
        /*
         *  本测试方法：
         *  1.将默认申请的3个buffer，分别填充 “彩条 黑 蓝”
         *  2.这里只使用到了一个彩条buffer和一个黑色buffer，黑色buffer用于清空屏幕内容，彩条用于局刷测试。
         *  3.要求: 根据测试buffer的bpp格式，强制jzfb_get_controller_bpp返回对应的格式。
         *  4.通过sys节点，传入4个主要测试参数：dst_x, dst_y, width, height，前两个是绘制到屏幕的位置，后两个是图像大小（彩条是根据实际需要进行绘制的）。
         *
         *  测试结果：
         *  1.基本确认和burst没有多少关系，buffer宽度保证word对齐应该就可以了，也可以尝试Dword。
         *  2.buffer为16bpp时，无法设置奇数宽度。32bpp没有这个问题。有待确认是否为硬件限制。
         *
         *  待验证：
         *  1.在buffer中抠取一块进行局刷没有测试。
         * */

	struct jzfb *jzfb = dev_get_drvdata(dev);
        int src_x, src_y, dst_x, dst_y, width, height;
	unsigned int tmp = 0;
        static char init = 1;
        /* uint16_t *p16; */
        uint32_t *p16;
        unsigned int a, b, c;
        int _w;

        /* printk("test_partial_refresh start!\n"); */
        sscanf(buf, "%d %d %d %d %d %d %d %d %d", &src_x, &src_y, &dst_x, &dst_y, &width, &height, &a, &b, &c);
        /* printk("%d %d %d %d %d %d\n", src_x, src_y, dst_x, dst_y, width, height); */

        jzfb_disable(jzfb->fb);

        if (init) {
		reg_write(jzfb, LCDC_BGC0, 0x000000);
                tmp = reg_read(jzfb, LCDC_CTRL);
                reg_write(jzfb, LCDC_CTRL, tmp | LCDC_CTRL_EOFM);

                buffer_partial_desc = jzfb->framedesc[3];
                buffer_partial_desc->next = (uint32_t)jzfb->framedesc[2] - 0xa0000000;
                buffer_partial_desc->id = 0xda0da3;
                printk("create partial buffer desc at %p\n", buffer_partial_desc);

                buffer_for_partial = jzfb->vidmem + jzfb->frm_size * 0;
                printk("buffer_for_partial address is %p\n", buffer_for_partial);

                for (p16 = (uint32_t *)jzfb->vidmem; p16 < (uint32_t *)(jzfb->vidmem + jzfb->frm_size * 1); p16++)
                        *p16 = 0xf800;
                for (p16 = (uint32_t *)(jzfb->vidmem + jzfb->frm_size * 1); p16 < (uint32_t *)(jzfb->vidmem + jzfb->frm_size * 2); p16++ )
                        *p16 = 0;
                        /* *p16 = 0x07e0; */
                for (p16 = (uint32_t *)(jzfb->vidmem + jzfb->frm_size * 2); p16 < (uint32_t *)(jzfb->vidmem + jzfb->frm_size * 3); p16++ )
                        *p16 = 0x001f;

                init = 0;
                printk("init\n");
        }
        _w = ALIGN(width, 4);
        jzfb_display_v_color_bar_test(jzfb->fb, _w, height);

        clear_screen(jzfb);

        slcd_send_partial_prm(jzfb, dst_x, dst_y, width, height);

        buffer_partial_desc->desc_size = (height-1) << 12 | (width - 1);

#if 1
        /* buffer_for_partial = xxx */
        jzfb->framedesc[2]->next = (uint32_t)buffer_partial_desc - 0xa0000000;
        buffer_partial_desc->databuf = (uint32_t)buffer_for_partial - 0xa0000000;

        if (jzfb->osd.fg0.bpp == 16) {
                buffer_partial_desc->offsize = (_w - width) / 2;
                buffer_partial_desc->page_width = width / 2;
                buffer_partial_desc->cpos = LCDC_CPOS_BPP_16;
                buffer_partial_desc->cmd = LCDC_CMD_EOFINT | LCDC_CMD_FRM_EN | height * width / 2;
        } else {
                buffer_partial_desc->offsize = _w - width;
                buffer_partial_desc->page_width = width;
                buffer_partial_desc->cpos = LCDC_CPOS_BPP_18_24;
                buffer_partial_desc->cmd = LCDC_CMD_EOFINT | LCDC_CMD_FRM_EN | height * width;
        }

        regw(LCDC_CTRL, 'a', LCDC_CTRL_BST_MASK);
        regw(LCDC_CTRL, 'o', LCDC_CTRL_BST_64);

#endif

        dma_start(jzfb);
        /* dump_lcdc_registers(jzfb); */

        return n;
}

