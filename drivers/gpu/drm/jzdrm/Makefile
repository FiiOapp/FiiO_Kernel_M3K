ccflags-y := -Iinclude/drm
ifeq (, $(findstring -W,$(EXTRA_CFLAGS)))
endif

jz_drm-y := \
	jzdrm_crtc.o \
	jzdrm_connector.o \
	jzdrm_drv.o

jz_drm-$(CONFIG_LCD_X163) += jz_panel_msg/mipi_dsi_panel/auo_x163.o \
			     jz_mipi_dsi/mipi_dsi_if.o

jz_drm-$(CONFIG_LCD_BYD_8991FTGF) += jz_panel_msg/rgb_panel/BM347WV-F-8991FTGF.o \
				     jz_rgb/lcd_rgb_if.o

jz_drm-$(CONFIG_JZ_MIPI_DSI)      += jz_mipi_dsi/jz_mipi_dsi.o \
				     jz_mipi_dsi/jz_mipi_dsi_lowlevel.o \
				     jz_mipi_dsi/jz_mipi_dsih_hal.o

obj-$(CONFIG_DRM_JZDRM)	+= jz_drm.o
