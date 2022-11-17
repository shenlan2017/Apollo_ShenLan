/*
 * @Description: 存放处理后的IMU姿态以及GNSS位置
 * @Author: Ren Qian
 * @Date: 2020-02-27 23:10:56
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-02 09:57:55
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_H_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_H_
#include <vector>

#include <Eigen/Dense>

#include "lidar_localization/sensor_data/velocity_data.h"

namespace lidar_localization {

class PoseData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  Eigen::Quaternionf GetQuaternion() const {
    Eigen::Quaternionf q;
    q = pose_.block<3, 3>(0, 0);
    return q;
  }

  void GetVelocityData(VelocityData& velocity_data) const {
    velocity_data.time_ = time_;

    velocity_data.linear_velocity_.x = vel_.v.x();
    velocity_data.linear_velocity_.y = vel_.v.y();
    velocity_data.linear_velocity_.z = vel_.v.z();

    velocity_data.angular_velocity_.x = vel_.w.x();
    velocity_data.angular_velocity_.y = vel_.w.y();
    velocity_data.angular_velocity_.z = vel_.w.z();
  }

  static bool SyncData(
      double sync_time,
      std::deque<PoseData, Eigen::aligned_allocator<PoseData>>& UnsyncedData,
      std::deque<PoseData, Eigen::aligned_allocator<PoseData>>& SyncedData);

  static bool SyncData(
      double sync_time,
      std::deque<PoseData, Eigen::aligned_allocator<PoseData>>& UnsyncedData,
      PoseData& SyncedData);

  double time_{0.0};
  Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();

  struct {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    Eigen::Vector3f w = Eigen::Vector3f::Zero();
  } vel_;

  std::vector<double> cov_;
};

}  // namespace lidar_localization

#endif
