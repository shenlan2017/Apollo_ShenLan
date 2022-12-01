sudo cp /apollo/supplement/sources.list /etc/apt/
sudo apt update

sudo apt-get install tcl-dev tk-dev python3-tk libeigen3-dev -y
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgtest-dev libglew-dev libcholmod3 libcxsparse3 -y

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

