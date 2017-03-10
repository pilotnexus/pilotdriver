#!/bin/bash

echo "unloading modules from $1"
sshpass -praspberry ssh -o StrictHostKeyChecking=no pi@192.168.2.179 "sudo rmmod pilot_io;sudo rmmod pilot_tty;sudo rmmod pilot_slcd;sudo rmmod pilot_plc;sudo rmmod pilot_rtc;sudo rmmod pilot;mkdir ~/pilotmodules"

sshpass -praspberry scp -o StrictHostKeyChecking=no ./driver/pilot.ko pi@192.168.2.179:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./io/pilot_io.ko pi@192.168.2.179:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./tty/pilot_tty.ko pi@192.168.2.179:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./plc/pilot_plc.ko pi@192.168.2.179:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./slcd/pilot_slcd.ko pi@192.168.2.179:~/pilotmodules/
sshpass -praspberry scp -o StrictHostKeyChecking=no ./rtc/pilot_rtc.ko pi@192.168.2.179:~/pilotmodules/

sshpass -praspberry ssh -o StrictHostKeyChecking=no pi@192.168.2.179 "sudo insmod ~/pilotmodules/pilot.ko;sudo insmod ~/pilotmodules/pilot_io.ko;sudo insmod ~/pilotmodules/pilot_tty.ko;sudo insmod ~/pilotmodules/pilot_slcd.ko;sudo insmod ~/pilotmodules/pilot_plc.ko;sudo insmod ~/pilotmodules/pilot_rtc.ko"
