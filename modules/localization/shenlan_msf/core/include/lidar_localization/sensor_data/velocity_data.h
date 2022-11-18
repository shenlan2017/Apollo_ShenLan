/*
 * @Description: velocity 数据
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-21 22:54:27
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_H_
#define LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_H_
#include <deque>

#include <Eigen/Dense>

namespace lidar_localization {
class VelocityData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  static bool SyncData(
      double sync_time,
      std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>>&
          UnsyncedData,
      std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>>&
          SyncedData);
  void TransformCoordinate(Eigen::Matrix4f transform_matrix);
  void NED2ENU();

  struct LinearVelocity {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct AngularVelocity {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  double time_{0.0};
  LinearVelocity linear_velocity_;
  AngularVelocity angular_velocity_;
};
}  // namespace lidar_localization
#endif
