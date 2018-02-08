:: copy Makefile, all header, and .c files to the ubuntu build server
scp ./Makefile *.h *.c deploy2rpi.sh mdk@192.168.2.246:~/Documents/pilot/io/

:: start the build
::ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/io/Module.symvers; cd ~/Documents/pilot/io; make 3.6.11+538"
::ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/io/Module.symvers; cd ~/Documents/pilot/io; make 3.12.22+691"
ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/io/Module.symvers; cd ~/Documents/pilot/io; make 3.18.7+755"
