FROM debian:stretch

RUN dpkg --add-architecture i386 && apt-get update && apt-get install -y apt-utils libc6:i386 libncurses5:i386 libstdc++6:i386 zlib1g:i386 bc device-tree-compiler sshpass git rsync cmake sudo nano && useradd -ms /bin/bash user && mkdir /home/user/build && chmod a+rwx /home/user/build
COPY ./getkernel.sh /home/user/
COPY ./package /home/user/build/package/

RUN chmod a+rwx /home/user/getkernel.sh && chown -R user:user /home/user/build/package/

USER user
WORKDIR /home/user/build

RUN git clone --depth=1 git://github.com/raspberrypi/tools.git && echo 'export PATH=$PATH:$HOME/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin' >> $HOME/.bashrc

WORKDIR /home/user

ENTRYPOINT ["/bin/bash"]