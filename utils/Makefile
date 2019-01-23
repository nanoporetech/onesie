# unless told otherwise, files will be installed in this directory
PREFIX ?= /opt/ONT/minit
RM        := rm -f

# list of all the programs to build
UTILS := minit-reg

IOCTL := ../driver/ont_minit_ioctl.h

CXXFLAGS += -std=c++11 -I ../driver

all: $(UTILS)

install: $(UTILS)
	install -d "$(DESTDIR)$(PREFIX)/bin"
	install -m755 $(UTILS) "$(DESTDIR)$(PREFIX)/bin"

uninstall:
	for util in $(UTILS); do $(RM) "$(DESTDIR)$(PREFIX)/bin/$$util"; done

minit-reg: minit-reg.cpp $(IOCTL)
	g++ $< $(CXXFLAGS) -o $@

clean:
	rm -f $(UTILS)
	
PHONY: all install clean uninstall
