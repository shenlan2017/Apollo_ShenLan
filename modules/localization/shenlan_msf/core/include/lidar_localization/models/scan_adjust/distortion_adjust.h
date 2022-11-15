/*
 * @Description: 点云畸变补偿
 * @Author: Ren Qian
 * @Date: 2020-02-25 14:38:12
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:20:29
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_H_

#include "glog/logging.h"

#include <Eigen/Dense>

#include <pcl/common/transforms.h>

#include "lidar_localization/models/scan_adjust/distortion_adjust.h"
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/velocity_data.h"

namespace lidar_localization {
class DistortionAdjust {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  void SetMotionInfo(const VelocityData& velocity_data,
                     const float scan_period);

  bool AdjustCloud(const CloudData::CloudTypePtr& input_cloud_ptr,
                   CloudData::CloudTypePtr& output_cloud_ptr);

 private:
  Eigen::Matrix3f UpdateMatrix(const float real_time) const {
    const Eigen::Vector3f angle = angular_rate_ * real_time;
    const Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    const Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;
    return t_V.matrix();
  }

  float scan_period_;
  Eigen::Vector3f velocity_;
  Eigen::Vector3f angular_rate_;
};
}  // namespace lidar_localization
#endif
