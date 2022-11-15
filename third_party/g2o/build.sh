bash /apollo/supplement/update_docker.sh
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-gl-dev libgtest-dev libglew-dev libcholmod3 libcxsparse3 
sudo cp /usr/lib/x86_64-linux-gnu/libcholmod.so /apollo/third_party/g2o/lib/cholmod/
sudo cp /usr/lib/x86_64-linux-gnu/libcholmod.so.3 /apollo/third_party/g2o/lib/cholmod/
sudo cp -r /usr/include/suitesparse/* /apollo/third_party/g2o/lib/cholmod/include/
sudo cp /usr/lib/x86_64-linux-gnu/libcxsparse.so /apollo/third_party/g2o/lib/cxsparse/
cp /apollo/third_party/g2o/config.h /apollo/third_party/g2o/g2o/g2o/
