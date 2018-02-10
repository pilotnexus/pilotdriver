:: copy Makefile, all header, and .c files to the ubuntu build server
scp ./Makefile *.h *.c deploy2rpi.sh mdk@192.168.2.246:~/Documents/pilot/driver/

:: start the build
::ssh mdk@192.168.2.246 "cd ~/Documents/pilot/driver; make 3.6.11+538"
::ssh mdk@192.168.2.246 "cd ~/Documents/pilot/driver; make 3.12.22+691"

:: rpi1
::ssh mdk@192.168.2.246 "cd ~/Documents/pilot/driver; make 3.18.7+755"

:: rpi2
ssh mdk@192.168.2.246 "cd ~/Documents/pilot/driver; make 4.1.7+817"
