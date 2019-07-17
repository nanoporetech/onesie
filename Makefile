# default to distributing as source and relying on DKMS to build
COMPILED_DRIVER_PACKAGE ?= 0
PACKAGE_BASE_NAME := ont-minion1c-driver
DISTRIBUTION := $(shell lsb_release -cs)

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


# Extract a version string from the driver source it mus be the only thing
# between quotes
VERSION   := $(shell grep ONT_DRIVER_VERSION driver/minion_top.h | sed -e 's/^.*"\([^"]*\)"$$/\1/')
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
	$(MAKE) -C utils $@

clean:
	$(MAKE) -C driver $@
	$(MAKE) -C utils $@
	#$(MAKE) -C test $@
	$(RM) -r package
	$(RM) $(PACKAGE_BASE_NAME)-dev_$(VERSION)-1~$(DISTRIBUTION)_all.deb
	$(RM) $(PACKAGE_BASE_NAME)-dkms_$(VERSION)-1~$(DISTRIBUTION)_all.deb
	$(RM) $(PACKAGE_BASE_NAME)-utils_$(VERSION)-1~$(DISTRIBUTION)_$(DEB_ARCH).deb
	$(RM) $(PACKAGE_BASE_NAME)-$(KVERS)_$(VERSION)-1~$(DISTRIBUTION)_$(DEB_ARCH).deb
	$(RM) $(PACKAGE_BASE_NAME)_$(VERSION)-1~$(DISTRIBUTION).dsc
	$(RM) $(PACKAGE_BASE_NAME)_$(VERSION)-1~$(DISTRIBUTION).tar.gz

dist-deb:
	# assmeble all the files under package
	rm -rf package
	mkdir -p package/debian
	# make the .deb control file and change kernel verison number
	cp debian/control package/debian/control
	if [ $(COMPILED_DRIVER_PACKAGE) -eq 1 ]; then\
		sed -e "s/_KVERS_/$(KVERS)/g;s/_VERSION_/$(VERSION)/g" debian/control.modules.in >> package/debian/control;\
		sed -e "s/_KVERS_/$(KVERS)/g" debian/postinst.modules.in > package/debian/ont-minion1c-driver-$(KVERS).postinst;\
		sed -e "s/_KVERS_/$(KVERS)/g" debian/postrm.modules.in > package/debian/ont-minion1c-driver-$(KVERS).postrm;\
	fi
	sed -i -e "s/_ARCH_/$(DEB_ARCH)/g;s/_VERSION_/$(VERSION)/g" package/debian/control
	# debhelper version-9, changelog is just version number
	echo 9 > package/debian/compat
	echo "$(PACKAGE_BASE_NAME) ($(VERSION)-1~$(shell lsb_release -cs)) unstable; urgency=low" > package/debian/changelog

	# copy the source and packaging information, make the source package
	mkdir $(PACKAGE_BASE_NAME)-$(VERSION)
	cp -r package/debian $(PACKAGE_BASE_NAME)-$(VERSION)
	cp -r driver utils Makefile $(PACKAGE_BASE_NAME)-$(VERSION)
	dpkg-source -b $(PACKAGE_BASE_NAME)-$(VERSION)
	$(RM) -r $(PACKAGE_BASE_NAME)-$(VERSION)

	# cleanup
	cd package && fakeroot dh_prep
	# add utils
	$(MAKE) -C utils DESTDIR=$(PWD)/package/debian/ont-minion1c-driver-utils install

	# if this is a binary then make and add driver object file.
	if [ $(COMPILED_DRIVER_PACKAGE) -eq 1 ]; then $(MAKE) -C driver DESTDIR=$(PWD)/package/debian/$(PACKAGE_BASE_NAME)-$(KVERS) PREFIX=/usr install-modules; fi
	# add driver-includes and udev rules, add source files for DKMS
	$(MAKE) -C driver DESTDIR=$(PWD)/package/debian/$(PACKAGE_BASE_NAME)-dev PREFIX=/usr install-dev
	$(MAKE) -C driver distdir=$(PWD)/package/debian/$(PACKAGE_BASE_NAME)-dkms/usr/src/$(PACKAGE_BASE_NAME)-$(VERSION) dist
	$(MAKE) -C utils DESTDIR=$(PWD)/package/debian/$(PACKAGE_BASE_NAME)-utils PREFIX=/usr install
	# change the DKMS version to match the driver
	sed -e "s/_VERSION_/$(VERSION)/g" debian/dkms.conf.in > package/debian/$(PACKAGE_BASE_NAME)-dkms.dkms
	# generate .deb files to install binary driver
	if [ $(COMPILED_DRIVER_PACKAGE) -eq 1 ]; then cd package && fakeroot dh_installmodules; fi
	# generate .deb filse for the rest
	cd package && fakeroot dh_dkms -p $(PACKAGE_BASE_NAME)-dkms
	cd package && fakeroot dh_installdeb
	cd package && fakeroot dh_gencontrol
	cd package && fakeroot dh_builddeb
	$(RM) -r package

.PHONY: all install driver utils clean
