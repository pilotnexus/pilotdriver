# Prerequisites

For cross-compilation you need the ARM toolchain installed:
``` bash
sudo apt-get install gcc-arm-linux-gnueabihf # for 32-bit targets
sudo apt-get install gcc-aarch64-linux-gnu   # for 64-bit targets, not yet supported
```

# Local compilation

``` bash
make prepare
make local
make package
```

# Cross-compilation
1.First download the required kernel headers for your target Raspberry Pi
``` bash
 build/getkernel.sh [IP] [username] [password]
```

where [IP] is the IP of the Raspberry Pi you want to build the kernel module for
[username] is the SSH login user
[password] is the SSH login password
(use the space in front of the command to avoid storing the command in your history to avoid having the password stored in plaintext)

2. then run the build
show downloaded kernel header versions:
``` bash
make help
```

run the appropriate version (you can get the version on the Raspberry Pi by running `uname -a`)
for example:
``` bash
make xc_5.10.52-v7l+1441
```

# generate devicetree blob for raspberry pi
dtc -O dtb -o pilot.dtbo -b 0 -@ pilot_pi.dts

To build on the CORAL platform you need to install the kernel sources and variuos tools like this:
``` bash
sudo apt-get install python3-dev libssl-dev libffi-dev fakeroot u-boot-tools
```
