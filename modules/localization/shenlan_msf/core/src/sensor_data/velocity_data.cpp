/*
 * @Description: velocity data
 * @Author: Ren Qian
 * @Date: 2020-02-23 22:20:41
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 15:47:22
 */
#include "lidar_localization/sensor_data/velocity_data.h"

namespace lidar_localization {
bool VelocityData::SyncData(
    double sync_time,
    std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>>&
        UnsyncedData,
    std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>>&
        SyncedData) {
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
  while (UnsyncedData.size() >= 2) {
    if (UnsyncedData.front().time_ > sync_time) return false;
    if (UnsyncedData.at(1).time_ < sync_time) {
      UnsyncedData.pop_front();
      continue;
    }
    if (sync_time - UnsyncedData.front().time_ > 0.2) {
      UnsyncedData.pop_front();
      return false;
    }
    if (UnsyncedData.at(1).time_ - sync_time > 0.2) {
      UnsyncedData.pop_front();
      return false;
    }
    break;
  }
  if (UnsyncedData.size() < 2) return false;

  VelocityData front_data = UnsyncedData.at(0);
  VelocityData back_data = UnsyncedData.at(1);
  VelocityData synced_data;

  const double front_scale =
      (back_data.time_ - sync_time) / (back_data.time_ - front_data.time_);
  const double back_scale =
      (sync_time - front_data.time_) / (back_data.time_ - front_data.time_);
  synced_data.time_ = sync_time;
  synced_data.linear_velocity_.x = front_data.linear_velocity_.x * front_scale +
                                   back_data.linear_velocity_.x * back_scale;
  synced_data.linear_velocity_.y = front_data.linear_velocity_.y * front_scale +
                                   back_data.linear_velocity_.y * back_scale;
  synced_data.linear_velocity_.z = front_data.linear_velocity_.z * front_scale +
                                   back_data.linear_velocity_.z * back_scale;
  synced_data.angular_velocity_.x =
      front_data.angular_velocity_.x * front_scale +
      back_data.angular_velocity_.x * back_scale;
  synced_data.angular_velocity_.y =
      front_data.angular_velocity_.y * front_scale +
      back_data.angular_velocity_.y * back_scale;
  synced_data.angular_velocity_.z =
      front_data.angular_velocity_.z * front_scale +
      back_data.angular_velocity_.z * back_scale;

  SyncedData.push_back(synced_data);

  return true;
}

void VelocityData::TransformCoordinate(Eigen::Matrix4f transform_matrix) {
  Eigen::Matrix4d matrix = transform_matrix.cast<double>();

  Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
  Eigen::Vector3d t = matrix.block<3, 1>(0, 3);

  // get angular & linear velocities in IMU frame:
  Eigen::Vector3d w(angular_velocity_.x, angular_velocity_.y,
                    angular_velocity_.z);
  Eigen::Vector3d v(linear_velocity_.x, linear_velocity_.y, linear_velocity_.z);

  // a. first, add velocity component generated by rotation:
  Eigen::Vector3d delta_v;
  delta_v(0) = w(1) * t(2) - w(2) * t(1);
  delta_v(1) = w(2) * t(0) - w(0) * t(2);
  delta_v(2) = w(0) * t(1) - w(1) * t(0);
  v += delta_v;

  // b. transform velocities in IMU frame to lidar frame:
  w = R.transpose() * w;
  v = R.transpose() * v;

  // finally:
  angular_velocity_.x = w(0);
  angular_velocity_.y = w(1);
  angular_velocity_.z = w(2);
  linear_velocity_.x = v(0);
  linear_velocity_.y = v(1);
  linear_velocity_.z = v(2);
}

void VelocityData::NED2ENU() {
  LinearVelocity linear_velocity_enu;

  linear_velocity_enu.x = +linear_velocity_.y;
  linear_velocity_enu.y = +linear_velocity_.x;
  linear_velocity_enu.z = -linear_velocity_.z;

  linear_velocity_.x = linear_velocity_enu.x;
  linear_velocity_.y = linear_velocity_enu.y;
  linear_velocity_.z = linear_velocity_enu.z;

  AngularVelocity angular_velocity_enu;

  angular_velocity_enu.x = +angular_velocity_.y;
  angular_velocity_enu.y = +angular_velocity_.x;
  angular_velocity_enu.z = -angular_velocity_.z;

  angular_velocity_.x = angular_velocity_enu.x;
  angular_velocity_.y = angular_velocity_enu.y;
  angular_velocity_.z = angular_velocity_enu.z;
}

}  // namespace lidar_localization
