:: copy Makefile, all header, and .c files to the ubuntu build server
scp ./Makefile *.h *.c deploy2rpi.sh mdk@192.168.2.246:~/Documents/pilot/dummy/

:: start the build
ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/dummy/Module.symvers; cd ~/Documents/pilot/dummy; make 3.6.11+538"
