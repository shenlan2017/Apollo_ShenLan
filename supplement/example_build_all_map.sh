#! /bin/bash

if [ $# -lt 3 ]; then
  echo "Usage: example_build_all_map.sh [records folder] [map_name] [zone_id]"
  exit 1
fi

data_path=$1
map_name=$2
zone_id=$3
echo "data_path is : ${data_path}"

bash /apollo/supplement/build_lane_map.sh ${data_path} ${map_name}
echo "Build Lane Map Done."


bash /apollo/supplement/ndt_map_creator.sh \
    ${data_path} \
    /apollo/modules/calibration/data/dev_kit_pix_hooke/lidar_params/lidar_novatel_extrinsics.yaml \
    ${zone_id} \
    /apollo/modules/map/data/${map_name}/ndt_map \
    lidar
echo "Build NDT Map Done."


bash /apollo/supplement/msf_map_creator.sh \
    ${data_path} \
    /apollo/modules/calibration/data/dev_kit_pix_hooke/lidar_params/lidar_novatel_extrinsics.yaml \
    ${zone_id} \
    /apollo/modules/map/data/${map_name}/ \
    lidar
echo "Build MSF Map Done."