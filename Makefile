#MAKEFLAGS += -j

ifeq ($(MAKE),)
	MAKE=make
endif

all: .configured
	cd .build && $(MAKE) --no-print-directory

config: .build
	cd .build && ccmake ..
	touch .configured

.configured: .build
	cd .build && cmake ..
	touch .configured

.build:
	mkdir -p .build

clean: .build
	-cd .build && $(MAKE) clean --no-print-directory
	-rm -rf .build
	rm -f .configured

