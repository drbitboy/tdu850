TOPDIR := /usr/src/linux
HPATH  := $(TOPDIR)/include
ARCH   := $(shell uname -m | sed -e s/i.86/i386/ -e s/sun4u/sparc64/)

all:	tdu.o tunetdu tdutest modules
	cp -f tdu.o modules/
	chmod 755 tunetdu tdutest
	@echo "Now 'make [-n] devices' as root."
	@echo "Then install module as root e.g. "
	@echo "  insmod tdu.o parport=0 reset=1"
	@echo "and test with"
	@echo "  tunetdu /dev/tdu0 -S"

include $(TOPDIR)/.config
include $(TOPDIR)/arch/$(ARCH)/Makefile
include $(TOPDIR)/Rules.make

MODFLAGS = -DMODULE 
ifdef CONFIG_MODULES
ifdef CONFIG_MODVERSIONS
MODFLAGS += -DMODVERSIONS -include $(HPATH)/linux/modversions.h
endif
EXTRA_CFLAGS := $(MODFLAGS) -Wall -O2 -D__KERNEL__ -I$(HPATH)
EXTRA_CFLAGS := $(EXTRA_CFLAGS) -I.   # until tdu.h moved to /usr/src/linux
endif

modules:
	if [ ! -d modules ]; then $(RM) -Rf modules ; mkdir $@ ; fi

tunetdu:	tunetdu.c tdu.h
	$(CC) -Wall -O2 -I$(HPATH) $@.c -o $@

tdutest: tdutest.c tdu.h
	$(CC) -Wall -O2 -I$(HPATH) $@.c -o $@

devices:
	-rm -f /dev/tdu[012]
	mknod -m 0666 /dev/tdu0 c 60 0  # parport 0, a device
#	mknod -m 0666 /dev/tdu1 c 60 1  # parport 0, another device
#	mknod -m 0666 /dev/tdu2 c 60 2  # parport 0, another device still

ddd: ddd_image_io.o dddTest ddd2tdu
	chmod 755 dddTest ddd2tdu

ddd_image_io.o: ddd_image_io.c ddd_image_io.h
	$(CC) -c -Wall -O2 -I. ddd_image_io.c

dddTest:  dddTest.c ddd_image_io.o
	$(CC) -Wall -O2 -I. $^ -o $@

ddd2tdu:  ddd2tdu.c ddd_image_io.o
	$(CC) -Wall -O2 -I. $^ -o $@

cleanddd:
	-rm -f *ddd*.o dddTest ddd2tdu

clean: cleanddd
	-rm -f *.o *~ tunetdu tdutest /dev/tdu[012] .tdu.o.flags

maketars:
	( cd .. ; tar zcf tduwithddd.tar.gz tdu )
	tar cf ../ddd.tar ddd* ; $(RM) ddd*
	( cd .. ; tar zcf tdu.tar.gz tdu )
	tar xf ../ddd.tar

