MODNAME := minion

obj-m	:= $(MODNAME).o

minion-objs :=  i2c-altera.o minion_top.o dma.o

# Work out the kernel version
ifeq (,$(KERNELRELEASE))
KVERS ?= $(shell uname -r)
else
KVERS ?= $(KERNELRELEASE)
endif

PREFIX    ?= /usr/local
KERNELDIR ?= /lib/modules/$(KVERS)
PWD       := $(shell pwd)
RM        := rm -f

# source file groups
SHARED_HEADERS := minion_ioctl.h
KERNEL_SRCS    := minion_top.h dma.h minion_reg.h $(minion-objs:.o=.c) thermal_interface.h
DISTFILES := Makefile $(SHARED_HEADERS) $(KERNEL_SRCS)

ifeq ("$(KVERS)", "4.9.140-minit")
WANT_64BIT_DMA=0
else
WANT_64BIT_DMA=1
endif

all: modules

$(MODNAME).ko: $(KERNEL_SRCS) $(SHARED_HEADERS)
	$(MAKE) modules

modules:
	KCPPFLAGS="-DMINION_WANT_64BIT_DMA=$(WANT_64BIT_DMA)" $(MAKE) -C $(KERNELDIR)/build M=$(PWD)

install-modules: modules
	install -d "$(DESTDIR)$(KERNELDIR)/misc"
	install -m644 $(MODNAME).ko "$(DESTDIR)$(KERNELDIR)/misc"
ifndef DESTDIR
	depmod -a $(KVERS)
endif

install-dev:
	install -d "$(DESTDIR)$(PREFIX)/include"
	install -m644 $(SHARED_HEADERS) "$(DESTDIR)$(PREFIX)/include"

install: install-modules install-dev

uninstall: modules
	$(RM) "$(DESTDIR)$(KERNELDIR)/misc/$(MODNAME).ko"
	for shared_header in $(SHARED_HEADERS); do $(RM) "$(DESTDIR)$(PREFIX)/include/$$shared_header"; done
ifndef DESTDIR
	depmod -a $(KVERS)
endif

clean:
	$(MAKE) -C $(KERNELDIR)/build M=$(PWD) clean
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers *.order

dist:
	install -d $(distdir)
	cp $(DISTFILES) $(distdir)

.PHONY: all modules install uninstall clean install-modules install-dev
