:: copy Makefile, all header, and .c files to the ubuntu build server
scp ./Makefile *.h *.c deploy2rpi.sh mdk@192.168.2.246:~/Documents/pilot/tty/

:: start the build
::ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/tty/Module.symvers; cd ~/Documents/pilot/tty; make 3.6.11+538"
::ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/tty/Module.symvers; cd ~/Documents/pilot/tty; make 3.12.22+691"
ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/tty/Module.symvers; 
cd ~/Documents/pilot/tty; make 4.1.7+817"
