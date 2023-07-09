#! /bin/bash
if [ $# -lt 1 ]; then
  echo "Usage: example_build_all_map.sh [records folder]"
  exit 1
fi

BAG_PATH=$1

DOCKER_USER="${USER}"
CURRENT_DIR=${PWD}
DEV_CONTAINER="apollo_dev_${USER}"

# 进入docker解析数据并保存
xhost +local:root 1>/dev/null 2>&1
docker exec \
    -u "${DOCKER_USER}" \
    -it "${DEV_CONTAINER}" \
    /bin/bash /apollo/supplement/run_parse_lidar2camera.sh ${BAG_PATH}

xhost -local:root 1>/dev/null 2>&1

# 进行参数配置
if [ ! -d "/home/t/calibration/camera2lidar_ws/project/" ]; then
    mkdir -p /home/t/calibration/camera2lidar_ws/project
fi
cp ${BAG_PATH}/parsed_data/00000/pcd/1.pcd /home/t/calibration/camera2lidar_ws/project/
image_file=$(find $1/bag_camera_6mm_front -maxdepth 1 -type f -name "*.jpeg" -print -quit)
mv ${image_file} /home/t/calibration/camera2lidar_ws/project/1.jpeg
cp modules/calibration/data/dev_kit_pix_hooke/camera_params/front_6mm_extrinsics.yaml \
    /home/t/calibration/camera2lidar_ws/project
cp modules/calibration/data/dev_kit_pix_hooke/camera_params/front_6mm_intrinsics.yaml \
    /home/t/calibration/camera2lidar_ws/project

# 执行标定任务
cd /home/t/calibration/camera2lidar_ws/
if [ ! -d "/home/t/calibration/camera2lidar_ws/build/" ]; then
    mkdir build
fi
cd build
cmake ..
make -j4
cd /home/t/calibration/camera2lidar_ws/bin/
./run_lidar2camera \
../project/1.jpeg \
../project/1.pcd \
../project/front_6mm_intrinsics.yaml \
../project/front_6mm_extrinsics.yaml

mv ../bin/front_6mm_extrinsics.yaml \
    ${CURRENT_DIR}/modules/calibration/data/dev_kit_pix_hooke/camera_params/front_6mm_extrinsics.yaml

# 数据后处理，清除中间文件
cd ${CURRENT_DIR}
rm -rf ${BAG_PATH}/bag_camera_6mm_front/
rm -rf ${BAG_PATH}/bag_camera_6mm_front_timestamp.txt
rm -rf ${BAG_PATH}/parsed_data/


