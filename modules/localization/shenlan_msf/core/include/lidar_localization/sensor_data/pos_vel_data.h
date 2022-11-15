/*
 * @Description: synced GNSS-odo measurements as PosVelData
 * @Author: Ge Yao
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 21:36:09
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_H_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_H_

#include <string>

#include <Eigen/Dense>

namespace lidar_localization {

class PosVelData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  double time_{0.0};

  Eigen::Vector3f pos_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f vel_ = Eigen::Vector3f::Zero();

};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_H_
