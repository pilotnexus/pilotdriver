obj-m += pilot_slcd.o

pilot_slcd-objs := module.o bmp.o

SRC = /home/mdk/pilot/slcd
BASE_DRV_SRC = /home/mdk/pilot/driver

PREFIX = /home/mdk/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
KERNEL_HEADER_ROOT := /home/mdk/rpi
KERNEL_DIR_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* | grep linux-rpi)
KERNEL_VER_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* | grep linux-rpi | awk '{ sub(/.*linux-rpi-/,""); print }')

help:
	@echo ""
	@for x in $(KERNEL_VER_LIST); do printf '  make mod_%s  %.*s  build kernel module %s\n' $$x $$(expr 32 - $$( echo $$x | wc -c ) ) "................................." $$x; done
	@echo ""
	
# -------------------------------------------------------------------------------------------

mod: $(addprefix mod_,$(KERNEL_VER_LIST))

define mod_template
mod_$(1):
ifeq ($(DEBUG), 1)
	@echo "BUILDING DEBUG VERSION"
	@echo "#define DEBUG" > $(SRC)/../driver/debug.h
else
	@echo "/* #define DEBUG */" > $(SRC)/../driver/debug.h
endif
	make -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(BASE_DRV_SRC) clean
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(BASE_DRV_SRC) modules

	make -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(SRC) clean
	cp $(BASE_DRV_SRC)/Module.symvers $(SRC)
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(SRC) modules
	
endef

$(foreach mod,$(KERNEL_VER_LIST),$(eval $(call mod_template,$(mod))))

# -------------------------------------------------------------------------------------------
