#! /bin/bash

BAG_PATH=$1

DOCKER_USER="${USER}"
CURRENT_DIR=${PWD}
DEV_CONTAINER="apollo_dev_${USER}"

# 进入docker解析数据并保存
xhost +local:root 1>/dev/null 2>&1
docker exec \
    -u "${DOCKER_USER}" \
    -it "${DEV_CONTAINER}" \
    /bin/bash /apollo/supplement/run_parse_lidar2imu.sh ${BAG_PATH}

xhost -local:root 1>/dev/null 2>&1

# 运行激光里程计
cd ~/calibration/lidar2ins_ros_ws
catkin_make -j8
source devel/setup.bash
roslaunch floam floam_velodyne.launch pcd_path:=${CURRENT_DIR}/${BAG_PATH}/parsed_data/00000/pcd

# 数据对齐
cd ~/calibration/lidar2ins_non_ros_ws/pose_align/
if [ ! -d "/home/t/calibration/lidar2ins_non_ros_ws/pose_align/build/" ]; then
    mkdir build
fi
cd build
cmake ..
make -j4
./pose_align ${CURRENT_DIR}/${BAG_PATH}/parsed_data/00000/pcd/
mv lidar_novatel_extrinsics.yaml \
   ${CURRENT_DIR}/modules/calibration/data/dev_kit_pix_hooke/lidar_params/lidar_novatel_extrinsics.yaml

# 执行可视化
sudo apt-get install -y \
    xterm \
    libglm-dev \
    libglfw3-dev \
    ros-melodic-geodesy \
    ros-melodic-pcl-ros \
    ros-melodic-nmea-msgs \
    ros-melodic-libg2o
xterm -e "roscore" &
cd ~/calibration/lidar2ins_ros_ws 
catkin_make -j8
source devel/setup.bash
rosrun interactive_slam odometry2graph

# 数据后处理
cd ${CURRENT_DIR}
rm -rf ${BAG_PATH}/parsed_data/


