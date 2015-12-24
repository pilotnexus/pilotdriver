:: Tell ubuntu build server to deploy the module
@scp deploy2rpi.sh mdk@192.168.2.246:~/Documents/pilot/driver/deploy2rpi.sh
@ssh mdk@192.168.2.246 "cd ~/Documents/pilot/driver; ./deploy2rpi.sh"
