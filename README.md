# generate devicetree blob for raspberry pi
dtc -O dtb -o pilot.dtbo -b 0 -@ pilot_pi.dts




To build on the CORAL platform you need to install the kernel sources and variuos tools like this:


sudo apt-get install python3-dev libssl-dev libffi-dev fakeroot u-boot-tools
