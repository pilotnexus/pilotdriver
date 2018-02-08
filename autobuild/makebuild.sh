#!/bin/bash

HASH=$(zgrep '* firmware as of' ./root/usr/share/doc/raspberrypi-bootloader/changelog.Debian.gz | head -1 | awk '{ print $5 }')
echo "Hash is ${HASH}"
echo "Fetching firmware..."

if [ -d "./firmware" ]; then
  cd firmware
else
  mkdir ./firmware
  cd firmware
  git init
  git remote add origin https://github.com/raspberrypi/firmware.git
fi

git fetch
git checkout $HASH
echo "Done"
cd ..