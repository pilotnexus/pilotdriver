# use the first parameter as the version of the package
if [ -z $1 ] ; then
  echo No Version specified as an argument, using version 3.6.11+  
  VERSION="3.6.11+"
else
  VERSION=$1
fi

if [ -z $2 ] ; then
  echo No Subversion specified as an argument, using version as name
  NAME="$VERSION"
else
	NAME=$1$2
fi

# execute make for the main driver module
make -C ../driver ${NAME} # build the main driver module (pilot.ko)
make -C ../tty ${NAME}    # build the tty driver module (pilottty.ko)
make -C ../io ${NAME}     # build the io driver module (pilotio.ko)
make -C ../rtc ${NAME}    # build the rtc driver module (pilotrtc.ko)

MODULESDIR="./debian/lib/modules"
KODIR="${MODULESDIR}/${VERSION}"

# remove old debian/lib/modules folders
rm -rf ${MODULESDIR}

# create the folder for the specified version if it does not exist
if [ ! -d $KODIR ] ; then
  mkdir -p $KODIR
fi

# copy the kernel module binaries to the destination folder
cp ../driver/pilot.ko ${KODIR}/pilot.ko
cp ../tty/pilottty.ko ${KODIR}/pilottty.ko
cp ../io/pilotio.ko ${KODIR}/pilotio.ko
cp ../rtc/pilotrtc.ko ${KODIR}/pilotrtc.ko

# create the package name
PACKAGENAME="pilot${NAME}"

# create the package configuration file based upon the template debian/DEBIAN/control
sed "s/\[PACKAGENAME\]/${PACKAGENAME}/" control.template > debian/DEBIAN/control

# create the package
dpkg -b debian ${PACKAGENAME}.deb
