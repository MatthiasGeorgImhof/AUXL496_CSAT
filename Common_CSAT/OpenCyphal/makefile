# Master Makefile to run sub-Makefiles in isolated environments

MAKEFILES := makefile._4111spyglass makefile.reg makefile.uavcan

.PHONY: all clean

TRG := target

all:
	@for mf in $(MAKEFILES); do \
	    echo "Running $$mf..."; \
	    env -i HOME="$(HOME)" PATH="$(PATH)" $(MAKE) -f $$mf --no-print-directory; \
	done

clean:
	@for mf in $(MAKEFILES); do \
	    echo "Cleaning $$mf..."; \
	    env -i HOME="$(HOME)" PATH="$(PATH)" $(MAKE) -f $$mf clean --no-print-directory; \
	done
	rm -rf $(TRG)/nunavut
