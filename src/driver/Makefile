obj-m += pilot.o
pilot-objs := module.o queue.o commands.o

include $(PWD)/../Makefile.common

help:
	@echo ""
	@for x in $(KERNEL_VER_LIST); do printf '  make xc_%s  %.*s  build kernel module %s\n' $$x $$(expr 32 - $$( echo $$x | wc -c ) ) "................................." $$x; done
	@echo ""
	
# -------------------------------------------------------------------------------------------


local:
	@echo "BUILDING LOCAL BASE DRIVER ${BUILD_MSG}" 
	$(KCPPFLAGS) make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules 

xc: $(addprefix xc_,$(KERNEL_VER_LIST))

define xc_template
xc_$(1):
	@echo "BUILDING BASE DRIVER ${BUILD_MSG}"
	make -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) clean
	$(KCPPFLAGS) make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(filter %$(1),$(KERNEL_DIR_LIST)) M=$(PWD) modules
endef

$(foreach xc,$(KERNEL_VER_LIST),$(eval $(call xc_template,$(xc))))

# -------------------------------------------------------------------------------------------
