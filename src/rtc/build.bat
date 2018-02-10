:: copy Makefile, all header, and .c files to the ubuntu build server
scp ./Makefile ./deploy2rpi.sh *.h *.c mdk@192.168.2.246:~/Documents/pilot/rtc/

:: start the build ssh
::ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/rtc/Module.symvers; cd ~/Documents/pilot/rtc; make 3.12.22+691"
ssh mdk@192.168.2.246 "cp ~/Documents/pilot/driver/Module.symvers ~/Documents/pilot/rtc/Module.symvers; cd ~/Documents/pilot/rtc; make 3.18.7+755"
