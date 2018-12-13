# Makefile for USB tools
CROSS_COMPILE = mips-linux-gnu-
CC = $(CROSS_COMPILE)gcc
STRIP = $(CROSS_COMPILE)strip
PTHREAD_LIBS = -lpthread
WARNINGS = -Wall -Wextra
CFLAGS = $(WARNINGS) -g $(PTHREAD_LIBS) -I../include --static

all: sec_test aes_test
sec_test: sec_test.c
	$(CC) $(CFLAGS) -o sec_test sec_test.c
	$(STRIP) sec_test
aes_test:aes_test.c
	$(CC) $(CFLAGS) -o aes_test aes_test.c
	$(STRIP) aes_test

clean:
	$(RM) sec_test aes_test
