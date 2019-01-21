#!/bin/bash

if (( $# != 6 )); then
    echo "Illegal number of parameters"
    echo "usage: ./copyntfs.sh [copy_target_host] [target_nfs] [ip] [user] [password] [PXE-boot-IP]"
    exit 1
fi

sudo rsync -ratlzx --progress --rsh="/usr/bin/sshpass -p ${5} ssh -o StrictHostKeyChecking=no -l ${4}" boot/* ${4}@${3}:${1}
sudo rsync -ratlzx --progress --rsh="/usr/bin/sshpass -p ${5} ssh -o StrictHostKeyChecking=no -l ${4}" root/* ${4}@${3}:${2}

$(sshpass -p $5 ssh $4@$3 "echo \"dwc_otg.lpm_enable=0 console=tty1 root=/dev/nfs nfsroot=${6}:${2},vers=3 rw ip=dhcp rootwait elevator=deadline\" > ${1}/cmdline.txt ") || { echo 'error modifying cmdline.txt' ; exit 1; }
$(sshpass -p $5 ssh $4@$3 "sed '/^PARTUUID/ d' ${2}/etc/fstab > ${2}/etc/fstabtemp && mv ${2}/etc/fstabtemp ${2}/etc/fstab") || { echo 'error modifying fstab' ; exit 1; }