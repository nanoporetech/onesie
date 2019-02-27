# unless told otherwise, files will be installed in this directory
PREFIX ?= /opt/ONT/minit1c-driver-utils
RM        := rm -f

# list of all the programs to build
UTILS := minit-reg minit-hsrx minit-shift minit-eeprom minit-dma

IOCTL := ../driver/ont_minit_ioctl.h

CXXFLAGS += -std=c++11 -I ../driver

all: $(UTILS)

install: $(UTILS)
	install -d "$(DESTDIR)$(PREFIX)/bin"
	install -m755 $(UTILS) "$(DESTDIR)$(PREFIX)/bin"

uninstall:
	for util in $(UTILS); do $(RM) "$(DESTDIR)$(PREFIX)/bin/$$util"; done

%: %.cpp $(IOCTL)
	g++ $(CXXFLAGS) -o $@ $@.cpp

clean:
	rm -f $(UTILS)
	
PHONY: all install clean uninstall
