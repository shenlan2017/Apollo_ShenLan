#! /bin/bash
BAG_PATH=${PWD}/$1

if [ -d "${BAG_PATH}/parsed_data/" ]; then
    rm -rf ${BAG_PATH}/parsed_data/
fi

# 解析雷达和imu数据
bash scripts/lidar_parse.sh \
${BAG_PATH} \
${BAG_PATH} \
/apollo/modules/calibration/data/dev_kit_pix_hooke/lidar_params/lidar_novatel_extrinsics.yaml \
lidar16
test -s ${BAG_PATH}/parsed_data/00000/pcd/1.pcd \
    && echo "成功解析雷达数据！" || (echo "解析雷达数据失败，请检查。" && exit)

