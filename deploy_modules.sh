#!/bin/bash

if [ "$#" -lt 1 ]
then
  echo "Build and Deploy Pilot Modules"
  echo ""
  echo "Usage: deploy_modules [user@IP] [-p password] [make parameter]"
  echo "user@IP: The username and IP address of a Raspberry Pi (make sure that it is accessible via SSH)"
  echo "-p password: SSH Password (optional if SSH key is set up)"
  echo "make parameter: Parameter for make command (optional)"
  exit 1
fi

user_host=$1
shift

# If password is provided, use it. If not, use SSH key-based authentication.
if [[ $1 == -p* ]]
then
    password=${1#-p}
    shift
    SSH_CMD="sshpass -p$password ssh -o StrictHostKeyChecking=no $user_host"
    SCP_CMD="scp -o StrictHostKeyChecking=no"
else
    SSH_CMD="ssh -o StrictHostKeyChecking=no $user_host"
    SCP_CMD="scp -o StrictHostKeyChecking=no"
fi

KERNEL=$($SSH_CMD "uname -a | awk '{print \$3 substr(\$4, 2)}'")
echo "building $KERNEL"
make xc_$KERNEL $@

echo "unloading modules on $user_host"
$SSH_CMD "sudo rmmod pilot_fpga; sudo rmmod pilot_io;sudo rmmod pilot_tty;sudo rmmod pilot_slcd;sudo rmmod pilot_plc;sudo rmmod pilot_rtc;sudo rmmod pilot;mkdir -p ~/pilotmodules"

echo "copying modules on $user_host"
$SCP_CMD ./src/driver/pilot.ko $user_host:~/pilotmodules/
$SCP_CMD ./src/io/pilot_io.ko $user_host:~/pilotmodules/
$SCP_CMD ./src/tty/pilot_tty.ko $user_host:~/pilotmodules/
$SCP_CMD ./src/plc/pilot_plc.ko $user_host:~/pilotmodules/
$SCP_CMD ./src/slcd/pilot_slcd.ko $user_host:~/pilotmodules/
$SCP_CMD ./src/rtc/pilot_rtc.ko $user_host:~/pilotmodules/
$SCP_CMD ./src/fpga/pilot_fpga.ko $user_host:~/pilotmodules/

echo "loading modules on $user_host"
$SSH_CMD "sudo rmmod -f pilot_io;sudo rmmod -f pilot_tty;sudo rmmod -f pilot_slcd;sudo rmmod -f pilot_plc;sudo rmmod -f pilot_rtc; sudo  rmmod -f pilot_fpga;sudo rmmod -f pilot"
$SSH_CMD "sudo insmod ~/pilotmodules/pilot.ko;sudo insmod ~/pilotmodules/pilot_io.ko;sudo insmod ~/pilotmodules/pilot_tty.ko;sudo insmod ~/pilotmodules/pilot_slcd.ko;sudo insmod ~/pilotmodules/pilot_plc.ko;sudo insmod ~/pilotmodules/pilot_rtc.ko; sudo  insmod ~/pilotmodules/pilot_fpga.ko"
$SSH_CMD "cp /etc/pilot/variables /proc/pilot/plc/varconfig"
