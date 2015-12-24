:: Tell ubuntu build server to deploy the module
@scp deploy2rpi.sh mdk@192.168.2.246:~/Documents/pilot/plc/deploy2rpi.sh
@ssh mdk@192.168.2.246 "cd ~/Documents/pilot/plc; chmod +x deploy2rpi.sh; ./deploy2rpi.sh"
