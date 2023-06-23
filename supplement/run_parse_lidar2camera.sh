#! /bin/bash

BAG_PATH=${PWD}/$1

# 解析图像数据
sed -i "2c \  filepath: ${BAG_PATH}" modules/tools/record_parse_save/parser_params.yaml
./bazel-bin/modules/tools/record_parse_save/record_parse_save
test -s ${BAG_PATH}/../bag_camera_6mm_front_timestamp.txt \
    && echo "成功解析图像数据！" || (echo "解析图像数据失败，请检查。" && exit)
mv ${BAG_PATH}/../bag_camera_6mm_front_timestamp.txt ${BAG_PATH}/bag_camera_6mm_front_timestamp.txt
if [ -d "${BAG_PATH}/bag_camera_6mm_front/" ]; then
    rm -rf ${BAG_PATH}/bag_camera_6mm_front/
fi
mv /apollo/data/bag/bag_camera_6mm_front/ ${BAG_PATH}/

# 解析雷达和imu数据
bash scripts/lidar_parse.sh \
${BAG_PATH} \
${BAG_PATH} \
/apollo/modules/calibration/data/dev_kit_pix_hooke/lidar_params/lidar_novatel_extrinsics.yaml \
lidar16
test -s ${BAG_PATH}/parsed_data/00000/pcd/1.pcd \
    && echo "成功解析雷达数据！" || (echo "解析雷达数据失败，请检查。" && exit)



