TOPDIR = .

CHIP_TYPE ?= x1000
current_dir = $(PWD)

OBJS := $(TOPDIR)/src/start.o		\
	$(TOPDIR)/src/interface_wrap.o		\
	$(TOPDIR)/src/interface.o		\
	$(TOPDIR)/src/wakeup_handler.o		\
	$(TOPDIR)/src/irq.o		\
	$(TOPDIR)/src/cpm_ops.o		\
	$(TOPDIR)/src/pm_ops.o		\
	$(TOPDIR)/src/dmic_ops.o		\
	$(TOPDIR)/src/dma_ops.o		\
	$(TOPDIR)/src/tcu_timer.o		\
	$(TOPDIR)/src/rtc_ops.o		\
	$(TOPDIR)/src/jz_dma.o		\
	$(TOPDIR)/src/uart.o		\
	$(TOPDIR)/src/gpio.o		\
	$(TOPDIR)/src/print.o		\
	$(TOPDIR)/src/mem_buffer.o		\
	$(TOPDIR)/src/trigger_value_adjust.o		\
	$(TOPDIR)/src/recognization.o		\
	$(TOPDIR)/src/string.o		\


#	$(TOPDIR)/lib/libivw3_6.a


LD_FLAGS=-static -n -nostdlib -EL -m elf32ltsmip -T u-boot.lds --gc-sections -pie -Bstatic -Ttext 0 -fno-comment

CFLAGS := -fPIC -EL -O2 -mips32 -fvisibility=hidden -msoft-float
CFLAGS +=  -I$(current_dir)/include -I$(current_dir)/chips/$(CHIP_TYPE)/include
CFLAGS +=  -DDEBUG


PRJ=voice_module
BIN=$(PRJ).bin
HEX=$(PRJ).hex
DUMP=$(PRJ).asm
ELF=$(PRJ).elf
OBJCFLAGS = --gap-fill=0xff


all:$(HEX) $(DUMP) $(ELF)

$(HEX):$(BIN)
	@hexdump -v -e '"0x" 1/4 "%08x" "," "\n"' $< > $@
$(BIN):$(PRJ)
	mips-linux-gnu-objcopy --gap-fill=0xff -O binary $< $@
$(DUMP):$(PRJ)
	mips-linux-gnu-objdump -D $< > $@

$(ELF):$(PRJ)
	mips-linux-gnu-readelf -a $< > $@

$(PRJ):$(OBJS)
	mips-linux-gnu-ld -o $(PRJ) $(LD_FLAGS) $^ -Map $(PRJ).map

$(TOPDIR)/lib/libivw3_6.a:$(TOPDIR)/lib/libivw3_6.hex
	cp $(TOPDIR)/lib/libivw3_6.hex $(TOPDIR)/lib/libivw3_6.a

%.o:%.S
	mips-linux-gnu-gcc  $(CFLAGS)  -o $@ -c $^
%.o:%.c
	mips-linux-gnu-gcc  $(CFLAGS)  -o $@ -c $^


clean:
	rm $(BIN) $(PRJ) $(OBJS) $(PRJ).* -rf
