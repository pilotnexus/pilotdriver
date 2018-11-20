#!/bin/bash

if [ "$#" -ne 3 ]
then
  echo "Build Pilot Driver Modules"
  echo ""
  echo "Usage: getkernel [IP] [user] [password]"
  echo "IP: The IP address of a Raspberry Pi containing the kernel version for which the drivers should be compiled (make sure that it is accessible via SSH)"
  echo "user: SSH Username"
  echo "password: SSH Password"
  exit 1
fi

if ping -c 1 $1 &> /dev/null
then
  echo "Host ok"
else
  echo "Host $1 not reachable"
  exit 2;
fi

DIR=$(sshpass -p $3 ssh -o StrictHostKeyChecking=no -q $2@$1 "uname -a" | awk '{ printf tolower($1) "-rpi-" $3} $4 ~ /#/ { print substr($4,2) }') || { echo 'error getting uname' ; exit 1; }

if [ -z "$DIR" ]; then
 echo "Could not remotely get kernel version. Is host, username and password correct?"
 exit 1;
fi

export CCPREFIX=$PWD/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-

mkdir ./rpi
cd ./rpi
if [ -d "$DIR" ]; then
  # Control will enter here if $DIRECTORY exists.
  echo "$DIR already exists..updating"
  #rm -R "$DIR" || { echo 'removing directory ./rpi/$DIR failed' ; exit 1; }
else
  echo "creating directory $DIR"
  mkdir "$DIR" || { echo 'creating directory ./rpi/$DIR failed' ; exit 1; }
fi

HASH=$(sshpass -p $3 ssh -o StrictHostKeyChecking=no -q $2@$1 "FIRMWARE_HASH=\$(/bin/zgrep '* firmware as of' /usr/share/doc/raspberrypi-bootloader/changelog.Debian.gz | head -1 | awk '{ print \$5 }') && /usr/bin/wget https://raw.github.com/raspberrypi/firmware/\$FIRMWARE_HASH/extra/git_hash -O - 2> NUL")

echo "fetching linux kernel"
cd ./linux
git fetch || { echo 'fetch failed' ; exit 1; }

echo "checking out hash $HASH"
git checkout $HASH || { echo 'checkout failed' ; exit 1; }

echo "copying kernel"
rsync -a --info=progress2 ./ ../$DIR --exclude .git  || { echo 'copy failed' ; exit 1; }

echo "preparing build"
export KERNEL_SRC="$PWD/$DIR"

cd ../$DIR || { echo 'cd failed' ; exit 1; }
make mrproper

sshpass -p $3 ssh -o StrictHostKeyChecking=no -q $2@$1 "sudo modprobe configs"
sshpass -p $3 scp -o StrictHostKeyChecking=no -q $2@$1:/proc/config.gz ./
chown $USER:$USER config.gz
zcat config.gz > .config || { echo 'error creating .config' ; exit 1; }

echo "building kernel"

make ARCH=arm CROSS_COMPILE=${CCPREFIX} oldconfig
make ARCH=arm CROSS_COMPILE=${CCPREFIX}

