#!/bin/bash
#ldcon
THREADS=4

if [ "$#" -lt 2 ]
then
  echo "Build Pilot Driver Modules"
  echo ""
  echo "Usage: getkernel [IP] [user] [password]"
  echo "IP: The IP address of a Raspberry Pi containing the kernel version for which the drivers should be compiled (make sure that it is accessible via SSH)"
  echo "user: SSH Username"
  echo "password: SSH Password (optional if SSH key is set up)"
  exit 1
fi

if ping -c 1 $1 &> /dev/null
then
  echo "Host ok"
else
  echo "Host $1 not reachable"
  exit 2;
fi

# If password is provided, use it. If not, use SSH key-based authentication.
if [ "$#" -eq 3 ]
then
  DIR=$(sshpass -p $3 ssh -o StrictHostKeyChecking=no -q $2@$1 "uname -a" | awk '{ printf tolower($1) "-rpi-" $3} $4 ~ /#/ { print substr($4,2) }') || { echo 'error getting uname' ; exit 1; }
else
  DIR=$(ssh -o StrictHostKeyChecking=no -q $2@$1 "uname -a" | awk '{ printf tolower($1) "-rpi-" $3} $4 ~ /#/ { print substr($4,2) }') || { echo 'error getting uname' ; exit 1; }
fi

if [ -z "$DIR" ]; then
 echo "Could not remotely get kernel version. Is host and username correct?"
 exit 1;
else
  echo "Building for kernel $DIR"
fi

export CCPREFIX=arm-linux-gnueabihf-

mkdir -p ./rpi
cd ./rpi
if [ -d "$DIR" ]; then
  echo "$DIR already exists..updating"
else
  echo "creating directory $DIR"
  mkdir "$DIR" || { echo 'creating directory ./rpi/$DIR failed' ; exit 1; }
fi

# If password is provided, use it. If not, use SSH key-based authentication.
if [ "$#" -eq 3 ]
then
  HASH=$(sshpass -p $3 ssh -o StrictHostKeyChecking=no -q $2@$1 "FIRMWARE_HASH=\$(/bin/zgrep '* firmware as of' /usr/share/doc/raspberrypi-bootloader/changelog.Debian.gz | head -1 | awk '{ print \$5 }') && /usr/bin/wget https://raw.github.com/raspberrypi/firmware/\$FIRMWARE_HASH/extra/git_hash -O - 2> /dev/null")
else
  HASH=$(ssh -o StrictHostKeyChecking=no -q $2@$1 "FIRMWARE_HASH=\$(/bin/zgrep '* firmware as of' /usr/share/doc/raspberrypi-bootloader/changelog.Debian.gz | head -1 | awk '{ print \$5 }') && /usr/bin/wget https://raw.github.com/raspberrypi/firmware/\$FIRMWARE_HASH/extra/git_hash -O - 2> /dev/null")
fi
echo "firmware hash: $HASH"
echo "fetching linux kernel"
if [ ! -d "./linux" ]; then
  echo "Kernel source tree does not exist yet, cloning..."
  git clone https://github.com/raspberrypi/linux.git
fi
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

rm -rf ./config.gz
echo "copying config.gz to $PWD"
# If password is provided, use it. If not, use SSH key-based authentication.
if [ "$#" -eq 3 ]
then
  sshpass -p $3 ssh -o StrictHostKeyChecking=no -q $2@$1 "sudo modprobe configs"
  sshpass -p $3 scp -o StrictHostKeyChecking=no -q $2@$1:/proc/config.gz ./
else
  ssh -o StrictHostKeyChecking=no -q $2@$1 "sudo modprobe configs"
  scp -o StrictHostKeyChecking=no -q $2@$1:/proc/config.gz ./
fi

retVal=$?
if [ $retVal -ne 0 ]; then
    echo "Error getting config.gz"
    exit $retVal
fi

chown $USER:$USER config.gz
zcat config.gz > .config || { echo 'error creating .config' ; exit 1; }

echo "building kernel"
make ARCH=arm CROSS_COMPILE=${CCPREFIX} -j$THREADS defconfig
make ARCH=arm CROSS_COMPILE=${CCPREFIX} -j$THREADS
