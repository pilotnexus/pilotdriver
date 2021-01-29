#!/bin/bash

KERNEL=$(sshpass -praspberry ssh -o StrictHostKeyChecking=no pi@$1 "uname -a | sed 's/Linux raspberrypi \([^[[:blank:]]*\) #\([0-9]*\).*/\1\2/'")
echo "building $KERNEL"
cd src
make xc_$KERNEL $2

echo "unloading modules on $1"
sshpass -praspberry ssh -o StrictHostKeyChecking=no pi@$1 "sudo rmmod pilot_fpga; sudo rmmod pilot_io;sudo rmmod pilot_tty;sudo rmmod pilot_slcd;sudo rmmod pilot_plc;sudo rmmod pilot_rtc;sudo rmmod pilot;mkdir -p ~/pilotmodules;rm ~/pilotmodules/*"

echo "copying modules on $1"

sshpass -praspberry scp -o StrictHostKeyChecking=no ./driver/pilot.ko pi@$1:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./io/pilot_io.ko pi@$1:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./tty/pilot_tty.ko pi@$1:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./plc/pilot_plc.ko pi@$1:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./slcd/pilot_slcd.ko pi@$1:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./rtc/pilot_rtc.ko pi@$1:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./fpga/pilot_fpga.ko pi@$1:~/pilotmodules/

echo "loading modules on $1"
sshpass -praspberry ssh -o StrictHostKeyChecking=no pi@$1 "sudo rmmod -f ~/pilotmodules/pilot_io.ko;sudo rmmod -f ~/pilotmodules/pilot_tty.ko;sudo rmmod -f ~/pilotmodules/pilot_slcd.ko;sudo rmmod -f ~/pilotmodules/pilot_plc.ko;sudo rmmod -f ~/pilotmodules/pilot_rtc.ko; sudo  rmmod -f ~/pilotmodules/pilot_fpga.ko;sudo rmmod -f ~/pilotmodules/pilot.ko"
sshpass -praspberry ssh -o StrictHostKeyChecking=no pi@$1 "sudo insmod ~/pilotmodules/pilot.ko;sudo insmod ~/pilotmodules/pilot_io.ko;sudo insmod ~/pilotmodules/pilot_tty.ko;sudo insmod ~/pilotmodules/pilot_slcd.ko;sudo insmod ~/pilotmodules/pilot_plc.ko;sudo insmod ~/pilotmodules/pilot_rtc.ko; sudo  insmod ~/pilotmodules/pilot_fpga.ko"

cd ..
