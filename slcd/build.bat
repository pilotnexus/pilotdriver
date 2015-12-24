:: copy Makefile, all header, and .c files to the ubuntu build server
scp ./Makefile *.h *.c deploy2rpi.sh mdk@192.168.2.246:~/Documents/pilot/slcd/

:: start the build
::ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/slcd/Module.symvers; cd ~/Documents/pilot/slcd; make 3.12.22+691"
ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/slcd/Module.symvers; cd ~/Documents/pilot/slcd; make 3.18.7+755"
