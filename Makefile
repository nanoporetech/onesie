
all clean:
	$(MAKE) -C driver $@
	$(MAKE) -C utils $@
	$(MAKE) -C test/emulation $@
