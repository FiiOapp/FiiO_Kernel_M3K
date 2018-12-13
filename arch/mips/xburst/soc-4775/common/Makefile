

## WARNING! DO NOT change this lists order because power sleep order depends on it!! ##

obj-y += setup.o
obj-y += reset.o
obj-y += timer.o
obj-y += ddr.o
#maintimer.o cpu0timer.o cpu1timer.o
obj-y += serial.o
obj-y += tcu.o
obj-y += jzcpm_pwc.o

obj-y += clk.o

obj-y += gpio.o
# only for debug
#obj-y += pmmon.o
obj-y += irq.o
obj-y += cpm_usb.o

obj-y += platform.o
obj-y += regs_save_restore.o

obj-y += pm_p0.o


obj-$(CONFIG_CPU_FREQ) += cpufreq.o

obj-$(CONFIG_DWC_LM_INTERFACE)  += lm.o lm-device.o

obj-y += ahbm.o

ifeq ($(CONFIG_MTD_NAND_JZ4775), y)
obj-y += gpemc.o
obj-y += bch.o
endif
