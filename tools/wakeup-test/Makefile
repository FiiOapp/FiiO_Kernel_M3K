
SRCFILES=wakeup.c

all: clean wakeup

wakeup: $(SRCFILES)
	mips-linux-gnu-gcc -o $@ $(SRCFILES)
	mips-linux-gnu-strip $@
clean:
	rm -rf wakeup
