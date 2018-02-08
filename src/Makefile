BASE_DIR = /home/mdk/pilot
DRIVER_LIST = driver io plc rtc slcd tty fpga
PREFIX = /home/mdk/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
KERNEL_HEADER_ROOT := /home/mdk/rpi
KERNEL_DIR_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* | grep linux-rpi)
KERNEL_VER_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* | grep linux-rpi | awk '{ sub(/.*linux-rpi-/,""); print }')

PACKAGEDIR:="$(BASE_DIR)/package"
MODULESDIR:="$(PACKAGEDIR)/debian/lib/modules"
DEBDIR:="$(BASE_DIR)/deb"

help:
	@echo ""
	@for x in $(KERNEL_VER_LIST); do printf '  make mod_%s  %.*s  build kernel module %s\n' $$x $$(expr 32 - $$( echo $$x | wc -c ) ) "................................." $$x; done
	@echo "  make all  ................................  build all kernel modules"
	@echo ""
	
# -------------------------------------------------------------------------------------------
mod: $(addprefix mod_,$(KERNEL_VER_LIST))

define mod_template
mod_$(1):
	- mkdir $(BASE_DIR)/bin
	- rm -R $(BASE_DIR)/bin/$(1)
	mkdir $(BASE_DIR)/bin/$(1)
	- $(foreach DRV,$(DRIVER_LIST),make -C $(BASE_DIR)/$(DRV) mod_$(1) && cp $(BASE_DIR)/$(DRV)/*.ko $(BASE_DIR)/bin/$(1);)
	@if [ `ls -1 $(BASE_DIR)/bin/$(1) | wc -l` -ne $(words $(DRIVER_LIST)) ]; then rm -R $(BASE_DIR)/bin/$(1); \
	else \
	echo "create package"; \
	rm -rf $(MODULESDIR)/*; \
	mkdir $(MODULESDIR)/`echo $(1) | cut -f1 -d'+'`+; \
	cp $(BASE_DIR)/bin/$(1)/* $(MODULESDIR)/`echo $(1) | cut -f1 -d'+'`+; \
	sed "s/\[PACKAGENAME\]/pilot-$(1)/" $(PACKAGEDIR)/control.template > $(PACKAGEDIR)/debian/DEBIAN/control; \
	dpkg -b $(PACKAGEDIR)/debian $(DEBDIR)/pilot-$(1).deb; \
	fi
endef

$(foreach mod,$(KERNEL_VER_LIST),$(eval $(call mod_template,$(mod))))

# -------------------------------------------------------------------------------------------

all:
	$(foreach VER,$(KERNEL_VER_LIST), make mod_$(VER);)
	