set -e #错误即退出

sudo cp /apollo/supplement/sources.list /etc/apt/
sudo apt update

function apt_get_update_and_install() {
    # --fix-missing
    apt-get -y update && \
        apt-get -y install --no-install-recommends "$@"
}

sudo apt-get install tcl-dev tk-dev python3-tk libeigen3-dev -y
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgtest-dev libglew-dev libcholmod3 libcxsparse3 -y
apt_get_update_and_install \
    libeigen3-dev \
    libflann-dev \
    libglew-dev \
    libglfw3-dev \
    freeglut3-dev \
    libusb-1.0-0-dev \
    libdouble-conversion-dev \
    libopenni-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    liblz4-dev \
    libfreetype6-dev \
    libpcap-dev \
    libqhull-dev
apt_get_update_and_install \
    libusb-1.0-0 \
    libopenni0 \
    libfreetype6 \
    libtiff5 \
    libdouble-conversion1 \
    libpcap0.8 \
    libqhull7

cd /apollo/third_party/pcl/
FILE=/apollo/third_party/pcl/pcl-pcl-1.11.0
if [ ! -d "$FILE" ]; then
    sudo bash /apollo/docker/build/installers/install_pcl.sh
    sudo chmod 777 -R /apollo/third_party/pcl/pcl-pcl-1.11.0
    rm -rf /apollo/third_party/pcl/pcl-1.11.0.tar.gz
else
    cd pcl-pcl-1.11.0/build/
    make -j8
    sudo make install
fi
sudo rm -rf /opt/apollo/sysroot/include/pcl-1.10
cd /apollo
cp /apollo/supplement/BUILD.tpl /apollo/third_party/pcl/

