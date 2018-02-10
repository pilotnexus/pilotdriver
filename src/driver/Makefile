obj-m += pilot.o
pilot-objs := module.o queue.o commands.o
KDIR ?= /lib/modules/$(shell uname -r)

PREFIX = /home/mdk/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
KERNEL_HEADER_ROOT := /home/mdk/rpi


KERNEL_DIR_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* | grep linux-rpi)
KERNEL_VER_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* | grep linux-rpi | awk '{ sub(/.*linux-rpi-/,""); print }')

default:
	@echo "building kernel module"
	make -C $(KDIR) M=$(PWD) clean
	make -C $(KDIR) M=$(PWD) modules
help:
	@echo ""
	@echo "cross compile:"
	@for x in $(KERNEL_VER_LIST); do printf '  make xc_%s  %.*s  build kernel module %s\n' $$x $$(expr 32 - $$( echo $$x | wc -c ) ) "................................." $$x; done
	@echo ""
	
# -------------------------------------------------------------------------------------------

xc: $(addprefix xc_,$(KERNEL_VER_LIST))

define xc_template
xc_$(1):
ifeq ($(DEBUG), 1)
	@echo "BUILDING DEBUG VERSION"
	@echo "#define DEBUG" > $(PWD)/debug.h
else ifeq ($(DEBUG), 2)
	@echo "BUILDING DEBUG LEVEL 2 VERSION"
	@echo "#define DEBUG\n#define DEBUGALL" > $(PWD)/debug.h
else
	@echo "/* #define DEBUG */" > $(PWD)/debug.h
endif
	make -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) clean
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) modules
endef

$(foreach xc,$(KERNEL_VER_LIST),$(eval $(call xc_template,$(xc))))

# -------------------------------------------------------------------------------------------
