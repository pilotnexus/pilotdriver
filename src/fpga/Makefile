obj-m += pilot_fpga.o

pilot_fpga-objs := module.o flash.o

BASE_DRV_SRC = $(PWD)/../driver
PREFIX = $(PWD)/../../build/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
ifneq ($(shell $(PREFIX)gcc --version >/dev/null 2>&1; echo $$?),0)
	PREFIX = $(PWD)/../../build/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
endif
KERNEL_HEADER_ROOT := $(PWD)/../../build/rpi
KERNEL_DIR_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* 2> /dev/null | grep linux-rpi)
KERNEL_VER_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* 2> /dev/null | grep linux-rpi | awk '{ sub(/.*linux-rpi-/,""); print }')

help:
	@echo ""
	@for x in $(KERNEL_VER_LIST); do printf '  make xc_%s  %.*s  build kernel module %s\n' $$x $$(expr 32 - $$( echo $$x | wc -c ) ) "................................." $$x; done
	@echo ""
	
# -------------------------------------------------------------------------------------------



local:
		@echo "BUILDING FPGA DRIVER"
		make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules EXTRA_CFLAGS="-g -DDEBUG -DDEBUGALL"





xc: $(addprefix xc_,$(KERNEL_VER_LIST))

define xc_template
xc_$(1):
ifeq ($(DEBUG), 1)
	@echo "BUILDING DEBUG VERSION"
	make -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) clean
	cp $(BASE_DRV_SRC)/Module.symvers $(PWD)
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) modules EXTRA_CFLAGS="-g -DDEBUG"
else ifeq ($(DEBUG), 2)
	@echo "BUILDING DEBUG LEVEL 2 VERSION"
	make -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) clean
	cp $(BASE_DRV_SRC)/Module.symvers $(PWD)
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) modules EXTRA_CFLAGS="-g -DDEBUG -DDEBUGALL"
else
	make -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) clean
	cp $(BASE_DRV_SRC)/Module.symvers $(PWD)
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) modules
endif
	
endef

$(foreach xc,$(KERNEL_VER_LIST),$(eval $(call xc_template,$(xc))))

# -------------------------------------------------------------------------------------------
