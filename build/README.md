# Quickstart

build the docker image:
```
docker build -t pilotkernel:rpi . --rm
```

run the docker image:
```
docker run -it -v /home/amd/work/pilot/pilotdriver/build/rpi:/home/user/build/rpi -v /home/amd/work/pilot/pilotdriver/src/:/home/user/src pilotkernel:rpi
```

# files and folders

## check.sh
checks for the lastest raspberry pi image in https://downloads.raspberrypi.org/raspbian_latest. If a new version is available, the script downloads it and mounts the partitions into /boot and /root

## boot/ and root/ directories
used to mount images by check.sh

## images/ directory
holds downloaded images, used by check.sh

## copytftp.sh
enables to directly copy the mounted images to a tftp server (boot) and nfs server (root) for PXE booting

## rpi/ directory
mounted by the pilotkernel:rpi docker image, storage locations for Raspberry Pi kernel headers


