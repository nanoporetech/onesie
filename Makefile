# default to distributing as source and relying on DKMS to build
PACKAGE_BASE_NAME := ont-minion1c-driver

# DKMS sets KERNELRELEASE
ifeq (,$(KERNELRELEASE))
KVERS ?= $(shell uname -r)
else
KVERS ?= $(KERNELRELEASE)
endif
KERNELDIR ?= /lib/modules/$(KVERS)
PWD       := $(shell pwd)
RM        := rm -f
ARCH      ?= $(shell uname -p | sed 's/i386/x86_64/')

# correct architecture name
ifeq ($(ARCH), x86_64)
	DEB_ARCH = amd64
else ifeq ($(ARCH), aarch64)
	DEB_ARCH = arm64
else
	DEB_ARCH  ?= arm64
endif


# Extract a version string from the driver source. It must be the only thing
# between quotes
VERSION        := $(shell grep ONT_DRIVER_VERSION driver/minion_top.h | sed -e 's/^.*"\([^"]*\)"$$/\1/')
DISTRIBUTION   ?= $(shell . /etc/os-release && echo $$VERSION_CODENAME)
ifeq ($(DISTRIBUTION),)
VERSION_SUFFIX ?=
else
VERSION_SUFFIX ?= ~$(DISTRIBUTION)
endif

# Recommended minimum firmware version, used for setting the ont-minion1c-fpga
# dependency
FIRMWARE_VERSION := 2.4.1

# Each time the driver changes, ie whenever VERSION is different, this should be set back to 1
DEBIAN_REVISION := 1

all: utils driver test

driver:
	$(MAKE) -C driver

utils:
	$(MAKE) -C utils

test:
	#$(MAKE) -C test
	@echo "No tests on this branch yet!"

install:
	$(MAKE) -C driver $@
	$(MAKE) -C udev $@
	$(MAKE) -C utils $@

clean:
	$(MAKE) -C driver $@
	$(MAKE) -C utils $@
	#$(MAKE) -C test $@
	$(RM) -r package
	$(RM) -r $(PACKAGE_BASE_NAME)-$(VERSION)
	$(RM) $(PACKAGE_BASE_NAME)-*.deb
	$(RM) $(PACKAGE_BASE_NAME)_*.dsc
	$(RM) $(PACKAGE_BASE_NAME)_*.tar.gz

.PHONY: dist-deb
dist-deb:
	debian/debian_packaging.sh \
		--version $(VERSION)\
		--debian_revision $(DEBIAN_REVISION)\
		--version-suffix $(VERSION_SUFFIX)\
		--kernel $(KVERS)\
		--firmware $(FIRMWARE_VERSION)\
		--architecture $(DEB_ARCH)\
		--package-base-name $(PACKAGE_BASE_NAME)\
		$(if $(COMPILED_DRIVER_PACKAGE),--compiled-module)


.PHONY: all install driver utils clean
