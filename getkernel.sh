#!/bin/bash

if ping -c 1 $1 &> /dev/null
then
  echo "Host ok"
else
  echo "Host $1 not reachable"
  exit 2;
fi

DIR=$(sshpass -p raspberry ssh pi@$1 "uname -a" | awk '{ printf tolower($1) "-rpi-" $3} $4 ~ /#/ { print substr($4,2) }') || { echo 'error getting uname' ; exit 1; }

if [ -z "$DIR" ]; then
 echo "Could not remotely get kernel version. Did you add the host key to the known hosts?"
 exit 1;
fi

cd ~/rpi/
if [ -d "$DIR" ]; then
  # Control will enter here if $DIRECTORY exists.
  echo "$DIR already exists..updating"
  #rm -R "$DIR" || { echo 'removing directory ~/rpi/$DIR failed' ; exit 1; }
else
  echo "creating directory $DIR"
  mkdir "$DIR" || { echo 'creating directory ~/rpi/$DIR failed' ; exit 1; }
fi

HASH=$(sshpass -p raspberry ssh pi@$1 "FIRMWARE_HASH=\$(/bin/zgrep '* firmware as of' /usr/share/doc/raspberrypi-bootloader/changelog.Debian.gz | head -1 | awk '{ print \$5 }') && /usr/bin/wget https://raw.github.com/raspberrypi/firmware/\$FIRMWARE_HASH/extra/git_hash -O - 2> NUL")

echo "fetching linux kernel"
cd ~/linux
git fetch || { echo 'fetch failed' ; exit 1; }

echo "checking out hash $HASH"
git checkout $HASH || { echo 'checkout failed' ; exit 1; }

echo "copying kernel"
rsync -a --info=progress2 ./ ~/rpi/$DIR --exclude .git  || { echo 'copy failed' ; exit 1; }

echo "preparing build"
export CCPREFIX=/home/mdk/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
export KERNEL_SRC="~/rpi/$DIR"

cd ~/rpi/$DIR || { echo 'cd failed' ; exit 1; }
make mrproper

sshpass -p raspberry ssh pi@$1 "sudo modprobe configs"
sshpass -p raspberry scp pi@$1:/proc/config.gz ./
zcat config.gz > .config || { echo 'error creating .config' ; exit 1; }

echo "building kernel"

make ARCH=arm CROSS_COMPILE=${CCPREFIX} oldconfig
make ARCH=arm CROSS_COMPILE=${CCPREFIX}

