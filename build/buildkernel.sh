#!/bin/bash

if [ "$#" -ne 3 ]
then
  echo "Build Pilot Driver Modules"
  echo ""
  echo "Usage: buildkernel [x.y.z-v7+{buildnum}] [FIRMWARE_HASH] [config.gz]"
  exit 1
fi

DIR="linux-rpi-$1"

mkdir ~/work/pilot/pilotdriver/build/rpi
cd ~/work/pilot/pilotdriver/build/rpi
if [ -d "$DIR" ]; then
  # Control will enter here if $DIRECTORY exists.
  echo "$DIR already exists..updating"
  #rm -R "$DIR" || { echo 'removing directory ./rpi/$DIR failed' ; exit 1; }
else
  echo "creating directory $DIR"
  mkdir "$DIR" || { echo 'creating directory ./rpi/$DIR failed' ; exit 1; }
fi

HASH=$(/usr/bin/wget https://raw.github.com/raspberrypi/firmware/$2/extra/git_hash -O - 2> NUL)

echo "fetching linux kernel"
cd ~/work/pilot/pilotdriver/build/rpi/linux
git fetch || { echo 'fetch failed' ; exit 1; }

echo "checking out hash $HASH"
git checkout $HASH || { echo 'checkout failed' ; exit 1; }

echo "copying kernel"
rsync -a --info=progress2 ./ ~/work/pilot/pilotdriver/build/rpi/$DIR --exclude .git  || { echo 'copy failed' ; exit 1; }

echo "preparing build"
export CCPREFIX=/home/amd/work/pilot/pilotdriver/build/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
export KERNEL_SRC="~/work/pilot/pilotdriver/build/rpi/$DIR"

cd ~/work/pilot/pilotdriver/build/rpi/$DIR || { echo 'cd failed' ; exit 1; }
make mrproper

cp $3 ./config.gz
chown $USER:$USER ./config.gz
zcat ./config.gz > .config || { echo 'error creating .config' ; exit 1; }

echo "building kernel" 

make ARCH=arm CROSS_COMPILE=${CCPREFIX} oldconfig
make ARCH=arm CROSS_COMPILE=${CCPREFIX}

