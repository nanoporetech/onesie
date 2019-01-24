SHELL=/bin/bash
.PHONY: merged_filesystems
.PHONY: all
.DEFAULT_GOAL: all

all: merged_filesystems .configured_qemu
	$(MAKE) -C qemu $@

clean:
	rm -f .configured_qemu
	$(MAKE) -C qemu $@

MODIFICATION_FILES = $(shell find modifications/ -type f)
merged_filesystems: $(MODIFICATION_FILES)
	echo $^
	for file in $^; do ln -f -s $$(readlink -f $${file}) qemu/$${file#modifications/} ; done

.configured_qemu:
	if [ ! -x $@ ] ; then \
	cd qemu ; \
	./configure \
            --target-list=i386-softmmu,x86_64-softmmu,x86_64-linux-user,i386-linux-user \
            --enable-sdl && \
	touch ../.configured_qemu ; \
	cd .. ; \
	fi


