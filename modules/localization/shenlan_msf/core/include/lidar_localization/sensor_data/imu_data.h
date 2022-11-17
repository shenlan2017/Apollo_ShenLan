/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:20:21
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_H_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_H_

#include <cmath>
#include <deque>

#include <Eigen/Dense>

namespace lidar_localization {
class IMUData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  class Orientation {
   public:
    void Normlize() {
      double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
      x /= norm;
      y /= norm;
      z /= norm;
      w /= norm;
    }

    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{0.0};
  };

  struct LinearAcceleration {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct AngularVelocity {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct AccelBias {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  struct GyroBias {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  double time_{0.0};

  Orientation orientation_;

  LinearAcceleration linear_acceleration_;
  AngularVelocity angular_velocity_;

  AccelBias accel_bias_;
  GyroBias gyro_bias_;

 public:
  // 把四元数转换成旋转矩阵送出去
  Eigen::Matrix3f GetOrientationMatrix() const {
    Eigen::Quaterniond q(orientation_.w, orientation_.x, orientation_.y,
                         orientation_.z);
    return q.matrix().cast<float>();
  }

  static bool SyncData(
      double sync_time,
      std::deque<IMUData, Eigen::aligned_allocator<IMUData>>& UnsyncedData,
      std::deque<IMUData, Eigen::aligned_allocator<IMUData>>& SyncedData);

  static bool SyncData(
      double sync_time,
      std::deque<IMUData, Eigen::aligned_allocator<IMUData>>& UnsyncedData,
      IMUData& SyncedData);
};
}  // namespace lidar_localization
#endif
