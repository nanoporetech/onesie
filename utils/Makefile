# unless told otherwise, files will be installed in this directory
PREFIX ?= /opt/ONT/minion1c-driver-utils
RM        := rm -f

# list of all the programs to build
UTILS := minion-reg minion-hsrx minion-shift minion-eeprom minion-temp minion-dma dma-audit

IOCTL := ../driver/minion_ioctl.h

CXXFLAGS += -std=c++11 -I ../driver -pthread -Wl,--no-as-needed

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
