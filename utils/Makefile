# unless told otherwise, files will be installed in this directory
PREFIX ?= /opt/ONT/minion1c-driver-utils
DOC_PREFIX ?= /usr/share/man/man1
RM := rm -f

# list of all the programs to build
UTILS := minion-reg minion-hsrx minion-shift minion-eeprom minion-temp minion-dma dma-audit shift-decode

#MANUALS := $(UTILS:=.1.gz)
MANUALS := dma-audit.1.gz minion-dma.1.gz

IOCTL := ../driver/minion_ioctl.h

CXXFLAGS += -std=c++14 -O2 -I ../driver -pthread -Wl,--no-as-needed -D_FORTIFY_SOURCE=2

all: $(UTILS) $(MANUALS)

install: $(UTILS) $(MANUALS)
	install -d "$(DESTDIR)$(PREFIX)/bin"
	install -m755 $(UTILS) "$(DESTDIR)$(PREFIX)/bin"
	install -d "$(DESTDIR)$(DOC_PREFIX)"
	install -m644 $(MANUALS) "$(DESTDIR)$(DOC_PREFIX)"

uninstall:
	for util in $(UTILS); do $(RM) "$(DESTDIR)$(PREFIX)/bin/$$util"; done
	for manual in $(MANUALS); do $(RM) "$(DESTDIR)$(PREFIX)/bin/$$manual"; done

$(UTILS): %: %.cpp $(IOCTL)
	g++ $(CXXFLAGS) -o $@ $@.cpp

# Fix-up the version then convert from markdown to man page format
%.1: %.md
	sed -e "s/@VERSION@/$(VERSION)/g" $< | pandoc --standalone --to man --output $@

# Compress the man pages
$(MANUALS): %.1.gz: %.1
	# --no-name needed to not save the timestamp
	gzip --best --no-name --keep $<

clean:
	rm -f $(UTILS) $(MANUALS)
	
PHONY: all install clean uninstall
