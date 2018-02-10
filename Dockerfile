FROM debian:stretch

RUN dpkg --add-architecture i386 && apt-get update && apt-get install -y apt-utils libc6:i386 libncurses5:i386 libstdc++6:i386 git rsync cmake && useradd -ms /bin/bash user

USER user
WORKDIR /home/user

RUN git clone --depth=1 git://github.com/raspberrypi/tools.git && echo 'export PATH=$PATH:$HOME/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin' >> $HOME/.bashrc

ENTRYPOINT ["/bin/bash"]