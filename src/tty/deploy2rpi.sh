# raspberry pi ip address
IP="192.168.2.119"

# copy the module to the raspberry pi
scp -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no ./pilottty.ko pi@${IP}:~/

# remove the module and add it again
ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no pi@${IP} "sudo dmesg -C; sudo rmmod pilottty; sudo insmod ~/pilottty.ko; dmesg;"
