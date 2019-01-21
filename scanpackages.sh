#!/bin/bash
cd src
sudo sh -c 'make all'
cd ..
dpkg-scanpackages deb /dev/null | gzip -9c > deb/Packages.gz
scp ./deb/Packages.gz amd@amesconservices.cloudapp.net:~/archive/
scp ./deb/*.deb amd@amesconservices.cloudapp.net:~/archive/deb/
