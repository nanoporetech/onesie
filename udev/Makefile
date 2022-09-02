UDEV_RULE      := 10-minion-perms.rules

all:

install:
	install -d "$(DESTDIR)/lib/udev/rules.d"
	install -m644 $(UDEV_RULE) "$(DESTDIR)/lib/udev/rules.d"

uninstall:
	$(RM) "$(DESTDIR)/lib/udev/rules.d/$(UDEV_RULE)"

clean:

.PHONY: all install uninstall clean
