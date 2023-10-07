#!/bin/bash
#ldcon
THREADS=4
CROSS_COMPILER_PATH=
AARCH64_CROSS_COMPILER_PATH=/home/amd/work/pilot/pilotdriver/build/tools/gcc-arm-8.3-2019.03-x86_64-aarch64-linux-gnu/bin

if [ "$#" -lt 1 ]
then
  echo "Build Pilot Driver Modules"
  echo ""
  echo "Usage: getkernel [user@IP] [password]"
  echo "user@IP: SSH username and IP address of the Raspberry Pi (make sure it is accessible via SSH)"
  echo "password: SSH Password (optional if SSH key is set up)"
  exit 1
fi

# Extract the username and IP address from the command line argument
USER_HOST=$1
SSH_USER=$(echo $USER_HOST | cut -d '@' -f1)
IP=$(echo $USER_HOST | cut -d '@' -f2)

if ping -c 1 $IP &> /dev/null
then
  echo "Host ok"
else
  echo "Host $IP not reachable"
  exit 2;
fi


# If password is provided, use it. If not, use SSH key-based authentication.
if [ "$#" -eq 2 ]
then
  PASSWORD=$2
  SSH_CMD="sshpass -p $PASSWORD ssh -o StrictHostKeyChecking=no -q $USER_HOST"
else
  SSH_CMD="ssh -o StrictHostKeyChecking=no -q $USER_HOST"
fi

DIR=$($SSH_CMD "uname -a" | awk '{ printf tolower($1) "-rpi-" $3} $4 ~ /#/ { print substr($4,2) }') || { echo 'error getting uname' ; exit 1; }
if [ -z "$DIR" ]; then
 echo "Could not remotely get kernel version. Is host and username correct?"
 exit 1;
else
  echo "Building for kernel $DIR"
fi

# Determine the cross-compiler prefix based on the Raspberry Pi architecture
ARCH=$($SSH_CMD "uname -m")
if [[ $ARCH == "aarch64" ]]; then
  echo "Building for 64-bit architecture"
  CCPREFIX="$AARCH64_CROSS_COMPILER_PATH/aarch64-linux-gnu-"
  BUILD_ARCH=arm64
else
  echo "Building for 32-bit architecture"
  CCPREFIX="$CROSS_COMPILER_PATH/arm-linux-gnueabihf-"
  BUILD_ARCH=arm
fi

mkdir -p ./rpi
cd ./rpi
if [ -d "$DIR" ]; then
  echo "$DIR already exists..updating"
else
  echo "creating directory $DIR"
  mkdir "$DIR" || { echo 'creating directory ./rpi/$DIR failed' ; exit 1; }
fi

HASH=$($SSH_CMD "FIRMWARE_HASH=\$(/bin/zgrep '* firmware as of' /usr/share/doc/raspberrypi-bootloader/changelog.Debian.gz | head -1 | awk '{ print \$5 }') && /usr/bin/wget https://raw.github.com/raspberrypi/firmware/\$FIRMWARE_HASH/extra/git_hash -O - 2> /dev/null")
echo "firmware hash: $HASH"
echo "fetching linux kernel"
if [ ! -d "./linux" ]; then
  echo "Kernel source tree does not exist yet, cloning..."
  git clone https://github.com/raspberrypi/linux.git
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
$SSH_CMD "sudo modprobe configs"
$SSH_CMD "sudo cat /proc/config.gz" > ./config.gz

retVal=$?
if [ $retVal -ne 0 ]; then
    echo "Error getting config.gz"
    exit $retVal
fi

chown $USER:$USER config.gz
zcat config.gz > .config || { echo 'error creating .config' ; exit 1; }

echo "building kernel"
make ARCH=${BUILD_ARCH} CROSS_COMPILE=${CCPREFIX} -j$THREADS oldconfig
make ARCH=${BUILD_ARCH} CROSS_COMPILE=${CCPREFIX} -j$THREADS
