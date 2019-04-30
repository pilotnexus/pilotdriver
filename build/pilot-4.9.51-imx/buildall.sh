#! /bin/sh

# remove all old packages
rm *.deb

# build the packages
./buildpackage.sh 3.2.27+
./buildpackage.sh 3.6.11+
./buildpackage.sh 3.6.11+ 538
./buildpackage.sh 3.12.22+691
./buildpackage.sh 3.18.7+755
./buildpackage.sh 3.18.7-v7+755
./buildpackage.sh 3.18.11+781
./buildpackage.sh 3.18.11-v7+781

# copy the packages to the global package folder
cp *.deb ../../packages