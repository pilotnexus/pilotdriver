:: copy Makefile, all header, and .c files to the ubuntu build server
scp ./Makefile ./deploy2rpi.sh *.h *.c mdk@192.168.2.246:~/Documents/pilot/plc/

:: start the build ssh
::ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/plc/Module.symvers; cd ~/Documents/pilot/plc; make 3.12.22+691"
::ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/plc/Module.symvers; cd ~/Documents/pilot/plc; make 3.18.7+755"
ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/plc/Module.symvers; cd ~/Documents/pilot/plc; make 3.18.7-v7+755"
