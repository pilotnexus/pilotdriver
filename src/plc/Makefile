obj-m += pilot_plc.o
pilot_plc-objs := module.o

include $(PWD)/../Makefile.common

help:
	@echo ""
	@for x in $(KERNEL_VER_LIST); do printf '  make xc_%s  %.*s  build kernel module %s\n' $$x $$(expr 32 - $$( echo $$x | wc -c ) ) "................................." $$x; done
	@echo ""
	
# -------------------------------------------------------------------------------------------

local:
	@echo "BUILDING LOCAL PLC DRIVER ${BUILD_MSG}" 
	cp $(BASE_DRV_SRC)/Module.symvers $(PWD)
	$(KCPPFLAGS) make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules KBUILD_EXTRA_SYMBOLS=`pwd`/../driver/Module.symvers

xc: $(addprefix xc_,$(KERNEL_VER_LIST))

define xc_template
xc_$(1):
	@echo "BUILDING PLC DRIVER ${BUILD_MSG}" 
	make -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) clean
	cp $(BASE_DRV_SRC)/Module.symvers $(PWD)
	$(KCPPFLAGS) make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) modules
endef

$(foreach xc,$(KERNEL_VER_LIST),$(eval $(call xc_template,$(xc))))

# -------------------------------------------------------------------------------------------
