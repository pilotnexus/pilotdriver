OUT_DIR = $(PWD)/..
DRIVER_LIST = driver io plc rtc slcd tty fpga
DRIVER_NAMES_LIST = pilot pilot_io pilot_plc pilot_rtc pilot_slcd pilot_tty pilot_fpga

PREFIX = $(PWD)/../build/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
ifneq ($(shell $(PREFIX)gcc --version >/dev/null 2>&1; echo $$?),0)
	PREFIX = $(PWD)/../build/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
endif

KERNEL_HEADER_ROOT := $(PWD)/../build/rpi

KERNEL_DIR_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* | grep linux-rpi)
KERNEL_VER_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* | grep linux-rpi | awk '{ sub(/.*linux-rpi-/,""); print }')

PACKAGEDIR:="$(OUT_DIR)/build/package"
MODULESDIR:="$(PACKAGEDIR)/debian/lib/modules"
OVERLAYSDIR:="$(PACKAGEDIR)/debian/boot/overlays"
DEBDIR:="$(OUT_DIR)/deb"
VERSION:=$(shell cat version)

help:
	@echo "Cross-compile for version ${VERSION}"
	@echo ""
	@for x in $(KERNEL_VER_LIST); do printf '  make xc_%s  %.*s  build kernel module %s\n' $$x $$(expr 32 - $$( echo $$x | wc -c ) ) "................................." $$x; done
	@echo "  make all  ................................  build all kernel modules"
	@echo ""
	
# -------------------------------------------------------------------------------------------
xc: $(addprefix xc_,$(KERNEL_VER_LIST))

define xc_template
xc_$(1):
	- mkdir $(OUT_DIR)/bin
	- rm -R $(OUT_DIR)/bin/$(1)
	mkdir $(OUT_DIR)/bin/$(1)
	- $(foreach DRV,$(DRIVER_LIST),cd $(PWD)/$(DRV) && make xc_$(1) && cp $(PWD)/$(DRV)/*.ko $(OUT_DIR)/bin/$(1);)
	@if [ `ls -1 $(OUT_DIR)/bin/$(1) | wc -l` -ne $(words $(DRIVER_NAMES_LIST)) ]; then \
	echo "not all driver files compiled. please check $(OUT_DIR)/bin/$(1) for missing kernel drivers"; \
	else \
	echo "create package"; \
	rm -rf $(MODULESDIR)/*; \
	mkdir -p $(MODULESDIR)/`echo $(1) | cut -f1 -d'+'`+; \
	cp $(OUT_DIR)/bin/$(1)/* $(MODULESDIR)/`echo $(1) | cut -f1 -d'+'`+; \
	mkdir -p $(OVERLAYSDIR); \
	cp pilot.dtbo $(OVERLAYSDIR); \
	sed "s/\[PACKAGENAME\]/pilot-$(1)/" $(PACKAGEDIR)/control.template | sed "s/\[VERSION\]/$(VERSION)/" > $(PACKAGEDIR)/debian/DEBIAN/control; \
	fakeroot dpkg -b $(PACKAGEDIR)/debian $(DEBDIR)/pilot-$(1).deb; \
	fi
endef

$(foreach xc,$(KERNEL_VER_LIST),$(eval $(call xc_template,$(xc))))

# -------------------------------------------------------------------------------------------

all:
	$(foreach VER,$(KERNEL_VER_LIST), make xc_$(VER);)