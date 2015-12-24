obj-m += pilotio.o

pilotio-objs := module.o irq.o

SRC = /home/mdk/Documents/pilot/io
LINUX_3_2_27 = /home/mdk/rpi/linux-rpi-3.2.27/
LINUX_3_6_11 = /home/mdk/rpi/linux-rpi-3.6.y/
LINUX_3_6_11_538 = /home/mdk/rpi/linux-rpi-3.6.y-538/
LINUX_3_12_22_691 = /home/mdk/rpi/linux-rpi-3.12.22+691/
LINUX_3_18_7_755 = /home/mdk/rpi/linux-rpi-3.18.7+755/
LINUX_4_1_7_817 = /home/mdk/rpi/linux-rpi-4.1.7+817/
LINUX_4_1_7_V7_817 = /home/mdk/rpi/linux-rpi-4.1.7-v7+817/

PREFIX = /home/mdk/rpi/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-

all:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_2_27) M=$(SRC) modules

3.2.27+:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_2_27) M=$(SRC) modules

3.6.11+:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_6_11) M=$(SRC) modules

3.6.11+538:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_6_11_538) M=$(SRC) modules

3.12.22+691:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_12_22_691) M=$(SRC) modules

3.18.7+755:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_3_18_7_755) M=$(SRC) modules

4.1.7+817:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_4_1_7_817) M=$(SRC) modules

4.1.7-v7+817:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_4_1_7_V7_817) M=$(SRC) modules

clean:
	make -C $(LINUX_3_2_27) M=$(SRC) clean
	make -C $(LINUX_3_6_11) M=$(SRC) clean
	make -C $(LINUX_3_6_11_538) M=$(SRC) clean
	make -C $(LINUX_3_18_7_755) M=$(SRC) clean