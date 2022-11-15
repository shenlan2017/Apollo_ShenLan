/*
 * @Description: 关键帧之间的相对位姿，用于闭环检测
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:20:50
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_LOOP_POSE_H_
#define LIDAR_LOCALIZATION_SENSOR_DATA_LOOP_POSE_H_

#include <Eigen/Dense>

namespace lidar_localization {
class LoopPose {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  Eigen::Quaternionf GetQuaternion() const {
    Eigen::Quaternionf q;
    q = pose_.block<3, 3>(0, 0);
    return q;
  }

  double time_{0.0};
  unsigned int index0_{0};
  unsigned int index1_{0};
  Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();

};
}  // namespace lidar_localization

#endif
