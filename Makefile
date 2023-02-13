# default to distributing as source and relying on DKMS to build
COMPILED_DRIVER_PACKAGE ?= 0
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

export

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
.ONESHELL: # The rules execute in a single shell (as opposed to one shell per line.)
dist-deb:
	set -e
	# assmeble all the files under package
	rm -rf package
	mkdir -p package/debian
	# make the .deb control file and change kernel verison number
	cp debian/control package/debian/control
	if [ $(COMPILED_DRIVER_PACKAGE) -eq 1 ]; then
		sed -e "s/_KVERS_/$(KVERS)/g;s/_VERSION_/$(VERSION)/g;s/_FIRMWARE-VERSION_/$(FIRMWARE_VERSION)/g" debian/control.modules.in >> package/debian/control
		sed -e "s/_KVERS_/$(KVERS)/g" debian/postinst.modules.in > package/debian/ont-minion1c-driver-$(KVERS).postinst
		sed -e "s/_KVERS_/$(KVERS)/g" debian/postrm.modules.in > package/debian/ont-minion1c-driver-$(KVERS).postrm
	fi
	sed -i -e "s/_ARCH_/$(DEB_ARCH)/g;s/_VERSION_/$(VERSION)/g;s/_FIRMWARE-VERSION_/$(FIRMWARE_VERSION)/g" package/debian/control
	# debhelper version-9, changelog is just version number
	echo 9 > package/debian/compat
	echo "VERSION_SUFFIX=$(VERSION_SUFFIX)"
	echo "$(PACKAGE_BASE_NAME) ($(VERSION)-$(DEBIAN_REVISION)$(VERSION_SUFFIX)) unstable; urgency=low" > package/debian/changelog

	# copy the source and packaging information, make the source package
	mkdir $(PACKAGE_BASE_NAME)-$(VERSION)
	cp -r package/debian $(PACKAGE_BASE_NAME)-$(VERSION)
	cp -r driver utils Makefile $(PACKAGE_BASE_NAME)-$(VERSION)
	dpkg-source -b $(PACKAGE_BASE_NAME)-$(VERSION)
	$(RM) -r $(PACKAGE_BASE_NAME)-$(VERSION)

	# cleanup
	(cd package && fakeroot dh_prep)
	# add utils
	$(MAKE) -C utils DESTDIR=$(PWD)/package/debian/ont-minion1c-driver-utils install

	# if this is a binary then make and add driver object file.
	if [ $(COMPILED_DRIVER_PACKAGE) -eq 1 ]; then
		$(MAKE) -C driver DESTDIR=$(PWD)/package/debian/$(PACKAGE_BASE_NAME)-$(KVERS) PREFIX=/usr install-modules
	fi
	# add driver-includes, add source files for DKMS
	$(MAKE) -C driver DESTDIR=$(PWD)/package/debian/$(PACKAGE_BASE_NAME)-dev PREFIX=/usr install-dev
	$(MAKE) -C driver distdir=$(PWD)/package/debian/$(PACKAGE_BASE_NAME)-dkms/usr/src/$(PACKAGE_BASE_NAME)-$(VERSION) dist
	$(MAKE) -C udev DESTDIR=$(PWD)/package/debian/$(PACKAGE_BASE_NAME)-udev install
	$(MAKE) -C utils DESTDIR=$(PWD)/package/debian/$(PACKAGE_BASE_NAME)-utils PREFIX=/usr install
	# change the DKMS version to match the driver
	sed -e "s/_VERSION_/$(VERSION)/g" debian/dkms.conf.in > package/debian/$(PACKAGE_BASE_NAME)-dkms.dkms

	cd package
	# generate .deb files to install binary driver
	if [ $(COMPILED_DRIVER_PACKAGE) -eq 1 ]; then
		fakeroot dh_installmodules
	fi
	# generate .deb filse for the rest
	fakeroot dh_strip
	fakeroot dh_dkms -p $(PACKAGE_BASE_NAME)-dkms
	fakeroot dh_makeshlibs
	fakeroot dh_shlibdeps
	fakeroot dh_installdeb
	fakeroot dh_gencontrol
	fakeroot dh_md5sums
	fakeroot dh_builddeb
	cd ..
	$(RM) -r package

.PHONY: all install driver utils clean
