OUT_DIR = $(PWD)
DRIVER_LIST = driver io plc rtc slcd tty fpga
DRIVER_NAMES_LIST = pilot pilot_io pilot_plc pilot_rtc pilot_slcd pilot_tty pilot_fpga

PREFIX = arm-linux-gnueabihf-

KERNEL_HEADER_ROOT := $(PWD)/build/rpi

KERNEL_DIR_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* 2> /dev/null | grep linux-rpi)
KERNEL_VER_LIST := $(shell ls -1vd $(KERNEL_HEADER_ROOT)/* 2> /dev/null | grep linux-rpi | awk '{ sub(/.*linux-rpi-/,""); print }')

PACKAGEDIR:="$(OUT_DIR)/build/package"
MODULESDIR:="$(PACKAGEDIR)/debian/lib/modules"
OVERLAYSDIR:="$(PACKAGEDIR)/debian/boot/overlays"
KERNEL_SOURCE_DIR:="/opt/kernel-source"
DEB:="deb"
DEBDIR:="$(OUT_DIR)/$(DEB)"
VERSION:=$(shell cat version)

bold=$(shell tput bold)
normal=$(shell tput sgr0)

kern_release := $(shell uname -r)
kern_version := $(shell uname -a | awk '/\#/ { print substr($$4,2) }')
arch := $(shell uname -m)
fullkernel := $(kern_release)-$(arch)
# fullkernel := $(shell if echo $(kern_release) | grep -q "+$$"; then echo $(kern_release) | sed 's/+/-$(kern_version)/'; else echo $(kern_release)-$(kern_version); fi)

localname=pilot-$(kern_release)
fullname=pilot-$(fullkernel)

coral_packagedir:="$(OUT_DIR)/build/$(localname)"
coral_modulesdir:="$(coral_packagedir)/debian/lib/modules/$(kern_release)/"
coral_overlaysdir:="$(coral_packagedir)/debian/boot"

reverse = $(if $(1),$(call reverse,$(wordlist 2,$(words $(1)),$(1)))) $(firstword $(1))

local:
	echo "Building Pilot Kernel Drivers for $(fullkernel)"
	- rm -rf $(OUT_DIR)/bin/$(fullkernel)/*
	- mkdir -p $(OUT_DIR)/bin/$(fullkernel)
	@for x in $(DRIVER_LIST); do cd src/$$x && make local && cp $(PWD)/src/$$x/*.ko $(OUT_DIR)/bin/$(fullkernel)/ && cd ../..; done

load:
	$(eval UNLOAD_DRIVER_NAMES_LIST = $(call reverse,$(DRIVER_NAMES_LIST)))
	@sudo pkill -f pilotd
	@for x in $(UNLOAD_DRIVER_NAMES_LIST); do sudo rmmod "$$x" 2>&1 >/dev/null; done
	@for x in $(DRIVER_NAMES_LIST); do sudo insmod "$(OUT_DIR)/bin/$(fullkernel)/$$x.ko"; done
	@- cp /etc/pilot/variables /proc/pilot/plc/varconfig

prepare:
	sudo apt install -y git bc bison flex libssl-dev libncurses5-dev make raspberrypi-kernel-headers

package:
	echo "create package"; \
	rm -rf $(MODULESDIR)/*; \
	mkdir -p $(MODULESDIR)/$(kern_release); \
	cp $(OUT_DIR)/bin/$(fullkernel)/* $(MODULESDIR)/$(kern_release); \
	mkdir -p $(OVERLAYSDIR); \
	cp rpi_files/pilot.dtbo $(OVERLAYSDIR); \
	sed "s/\[PACKAGENAME\]/pilot-$(fullkernel)/" $(PACKAGEDIR)/control.template | sed "s/\[VERSION\]/$(VERSION)/" > $(PACKAGEDIR)/debian/DEBIAN/control; \
	mkdir -p $(DEBDIR); \
	fakeroot dpkg -b $(PACKAGEDIR)/debian $(DEBDIR)/pilot-$(fullkernel).deb; \

install:
	dpkg -i $(DEB)/pilot-$(fullkernel).deb; \

help:
	@echo "Cross-compile for version ${VERSION}"
	@echo ""
	@for x in $(KERNEL_VER_LIST); do printf '  make xc_%s  %.*s  build kernel module %s\n' $$x $$(expr 32 - $$( echo $$x | wc -c ) ) "................................." $$x; done
	@echo "  make all  ................................  build all kernel modules"
	@echo ""
	@echo "Or type $(bold)'make coral'$(normal) to build for the google coral board"
	@echo ""

coral:
	@echo "$(bold)Building drivers for Google Coral board: $(localname)$(normal)"
	- mkdir $(OUT_DIR)/bin
	- rm -rf $(OUT_DIR)/bin/*
	- $(foreach DRV,$(DRIVER_LIST),cd $(PWD)/$(DRV) && make local && cp $(PWD)/$(DRV)/*.ko $(OUT_DIR)/bin/;)
	
	@if [ `ls -1 $(OUT_DIR)/bin | wc -l` -ne $(words $(DRIVER_NAMES_LIST)) ]; then \
	echo "$(bold)not all driver files compiled. please check $(OUT_DIR)/bin/ for missing kernel drivers$(normal)"; \
	else \
	cd coral_files;\
	./setup.sh; \
	fi

# -------------------------------------------------------------------------------------------
xc: $(addprefix xc_,$(KERNEL_VER_LIST))

define xc_template
xc_$(1):
	- mkdir $(OUT_DIR)/bin
	- rm -R $(OUT_DIR)/bin/$(1)
	mkdir $(OUT_DIR)/bin/$(1)
	- $(foreach DRV,$(DRIVER_LIST),cd $(PWD)/src/$(DRV) && make xc_$(1) && cp $(PWD)/src/$(DRV)/*.ko $(OUT_DIR)/bin/$(1);)
	@if [ `ls -1 $(OUT_DIR)/bin/$(1) | wc -l` -ne $(words $(DRIVER_NAMES_LIST)) ]; then \
	echo "not all driver files compiled. please check $(OUT_DIR)/bin/$(1) for missing kernel drivers"; \
	else \
	echo "create package"; \
	rm -rf $(MODULESDIR)/*; \
	mkdir -p $(MODULESDIR)/`echo $(1); \
	cp $(OUT_DIR)/bin/$(1)/* $(MODULESDIR)/`echo $(1); \
	mkdir -p $(OVERLAYSDIR); \
	cp rpi_files/pilot.dtbo $(OVERLAYSDIR); \
	sed "s/\[PACKAGENAME\]/pilot-$(1)/" $(PACKAGEDIR)/control.template | sed "s/\[VERSION\]/$(VERSION)/" > $(PACKAGEDIR)/debian/DEBIAN/control; \
	fakeroot dpkg -b $(PACKAGEDIR)/debian $(DEBDIR)/pilot-$(1).deb; \
	fi
endef

$(foreach xc,$(KERNEL_VER_LIST),$(eval $(call xc_template,$(xc))))

# -------------------------------------------------------------------------------------------

all:
	$(foreach VER,$(KERNEL_VER_LIST), make xc_$(VER);)
