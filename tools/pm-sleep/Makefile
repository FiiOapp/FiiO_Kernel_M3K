
CHIP_TYPE ?= x1000
current_dir = $(PWD)

OBJS=start.o pm_sleep.o uart.o gpio.o #i2c-gpio.o

LD_FLAGS=-static -n -nostdlib -EL -m elf32ltsmip -T u-boot.lds --gc-sections -pie -Bstatic -Ttext 0 -fno-comment

CFLAGS := -fPIC -Os -O2 -mips32 -fvisibility=hidden
CFLAGS +=  -I$(current_dir)/include -I$(current_dir)/chips/$(CHIP_TYPE)/include
CFLAGS +=  -DDEBUG


PRJ=core_sleep
BIN=$(PRJ).bin
HEX=$(PRJ).hex
OBJCFLAGS = --gap-fill=0xff


all:$(HEX)

$(HEX):$(BIN)
	@hexdump -v -e '"0x" 1/4 "%08x" "," "\n"' $< > $@
$(BIN):$(PRJ)
	mips-linux-gnu-objcopy --gap-fill=0xff -O binary $< $@

$(PRJ):$(OBJS)
	mips-linux-gnu-ld -o $(PRJ) $(LD_FLAGS) $^ -Map $(PRJ).map

start.o:start.S
	mips-linux-gnu-gcc  $(CFLAGS)  -o $@ -c $^
%o:%c
	mips-linux-gnu-gcc  $(CFLAGS)  -o $@ -c $^
clean:
	rm $(BIN) $(PRJ) $(OBJS) $(PRJ).map $(HEX) -rf
