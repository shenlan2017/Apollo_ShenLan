/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 21:11:24
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_LIDAR_MEASUREMENT_DATA_H_
#define LIDAR_LOCALIZATION_SENSOR_DATA_LIDAR_MEASUREMENT_DATA_H_

#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/pose_data.h"

namespace lidar_localization {

class LidarMeasurementData {
 public:
  double time_{0.0};

  CloudData point_cloud_;
  IMUData imu_;
  PoseData gnss_odometry_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_SENSOR_DATA_LIDAR_MEASUREMENT_DATA_H_
