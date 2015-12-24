# raspberry pi ip address
IP="192.168.2.51"

# copy the module to the raspberry pi
scp -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no ./pilotplc.ko pi@${IP}:~/

# remove the module and add it again
ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no pi@${IP} "sudo dmesg -C; sudo rmmod ~/pilotplc.ko; sudo insmod ~/pilotplc.ko; dmesg;"
