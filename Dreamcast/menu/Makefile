# KallistiOS ##version##
#
# DreamOS menu Makefile
# (c)2023 Kaili Hill
#

all: rm-elf dreammenu.elf

include $(KOS_BASE)/Makefile.rules

# OBJS = dreammenu.o
SRCS=\
	$(wildcard *.c)\
	$(wildcard utils/*.c)
OBJS=$(SRCS:.c=.o)

clean:
	-rm -f dreammenu.elf $(OBJS)

rm-elf:
	-rm -f dreammenu.elf

dreammenu.elf: $(OBJS) 
	$(KOS_CC) $(KOS_CFLAGS) $(KOS_LDFLAGS) -o dreammenu.elf $(KOS_START) $(OBJS) $(DATAOBJS) $(OBJEXTRA) $(KOS_LIBS)

run: dreammenu.elf
	$(KOS_LOADER) dreammenu.elf

dist:
	-rm -f $(OBJS)
	$(KOS_STRIP) dreammenu.elf
