#!/bin/bash

fullfilename=$(curl -sIkL https://downloads.raspberrypi.org/raspbian_latest | sed -r '/Location: /!d;s/.*Location: (.*)$/\1/' | tail -n 1 | sed 's/.*\///' | sed 's/\r$//' )

extension="${fullfilename##*.}"
filename="${fullfilename%.*}"
image="${filename}.img"

#if ["$extension" != "zip"]; then
#  echo "Extension unknown, exiting"
#  exit 1
#fi

if [ ! -d "images" ]; then
  mkdir images
fi

cd ./images
if [ ! -f $image ]; then
    echo "Version ${fullfilename} not found, downloading..."
    wget --content-disposition https://downloads.raspberrypi.org/raspbian_latest
    unzip $fullfilename
    rm $fullfilename
fi

echo "trying to mount..."

partitions=(`fdisk -l $image | grep -w 'img[1-2]' | awk '{print ($2 * 512)}'`)
#echo $partitions | xxd
echo "${partitions[0]}"
echo "second offset:"
echo "${partitions[1]}"

cd ..

if [ ! -d "boot" ]; then
  mkdir boot
fi

if [ ! -d "root" ]; then
  mkdir root
fi

sudo umount ./boot
sudo umount ./root
sudo mount -v -o offset=${partitions[0]},gid=$(id -u $USER),uid=$(id -u $USER) -t vfat ./images/$image ./boot
sudo mount -v -o offset=${partitions[1]} -t ext4 ./images/$image ./root
sudo chown $USER:$USER ./root

FIRMWARE_HASH=$(/bin/zgrep '* firmware as of' ./root/usr/share/doc/raspberrypi-bootloader/changelog.Debian.gz | head -1 | awk '{ print $5 }')
HASH=$(wget https://raw.github.com/raspberrypi/firmware/$FIRMWARE_HASH/extra/git_hash -O - 2> NUL)


