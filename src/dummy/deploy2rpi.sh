# copy the module to the raspberry pi
scp ./pilotdummy.ko root@192.168.2.108:~/

# remove the module and add it again
ssh root@192.168.2.108 "dmesg -C; rmmod ~/pilotdummy.ko; insmod ~/pilotdummy.ko; dmesg;"
