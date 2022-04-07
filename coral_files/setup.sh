#!/bin/bash


OUT_DIR="../.."
DRIVER_NAMES_LIST="pilot pilot_io pilot_plc pilot_rtc pilot_slcd pilot_tty pilot_fpga"

DEBDIR="$OUT_DIR/deb"
PACKAGEDIR="$OUT_DIR/build/package"
VERSION=$(cat ../version)



bold=$(tput bold)
normal=$(tput sgr0)

kernel=$(uname -r)
localname=pilot-$kernel

coral_packagedir="$OUT_DIR/build/$localname"
coral_modulesdir="$coral_packagedir/debian/lib/modules/$kernel/"
coral_overlaysdir="$coral_packagedir/debian/boot"




echo "This is the setup.sh script found in coral_files"

echo "$bold create package $normal "


rm -rf $coral_packagedir
cp -r  $PACKAGEDIR $coral_packagedir
mkdir -p $coral_modulesdir
echo "Copying kernelmodules"
cp $OUT_DIR/bin/* $coral_modulesdir





echo "Making the devicetree overlay" 
./dtc -I dts -O dtb -o pilot_coral.dtbo -b 0 -@ pilot_coral.dts 
cp pilot_coral.dtbo $coral_overlaysdir

#echo "Adding pilot_coral overlay as boot parameter"
#echo "overlay=pilot_coral" >  $coral_overlaysdir/overlays.txt;


echo "Creating boot.scr to remove the serial output"
mkimage -A arm -T script -O linux -d boot.txt boot.scr

mkdir -p $coral_packagedir/debian/tmp
cp boot.scr $coral_packagedir/debian/tmp


cp postinst $coral_packagedir/debian/DEBIAN/
cp prerm $coral_packagedir/debian/DEBIAN/

echo "Replacing Version in .deb control file"
sed "s/\[PACKAGENAME\]/$localname/" $coral_packagedir/control.template | sed "s/\[VERSION\]/$VERSION/" > $coral_packagedir/debian/DEBIAN/control


echo "All good, creating the .deb"

fakeroot dpkg -b $coral_packagedir/debian $DEBDIR/$localname.deb 
