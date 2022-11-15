/*
 * @Description: LIO key frame
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-02 14:53:38
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_H_
#define LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <sophus/so3.hpp>

#include "lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.h"



namespace lidar_localization {

struct KeyFrame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  static const int kIndexP{0};
  static const int kIndexR{3};
  static const int kIndexV{6};
  static const int kIndexA{9};
  static const int kIndexG{12};

  KeyFrame() {}

  explicit KeyFrame(const int vertex_id, const g2o::PRVAG& prvag) {
    // set time:
    time_ = prvag.time_;
    // set seq. ID:
    index_ = vertex_id;
    // set state:
    pose_.block<3, 1>(0, 3) = prvag.pos_.cast<float>();
    pose_.block<3, 3>(0, 0) = prvag.ori_.matrix().cast<float>();
    vel_.v = prvag.vel_.cast<float>();
    bias_.accel = prvag.b_a_.cast<float>();
    bias_.gyro = prvag.b_g_.cast<float>();
  }

  explicit KeyFrame(const int param_index,
                    const double T,
                    const double* prvag) {
    // set time:
    time_ = T;
    // set seq. ID:
    index_ = param_index;
    // set state:
    Eigen::Map<const Eigen::Vector3d> pos(prvag + kIndexP);
    Eigen::Map<const Eigen::Vector3d> log_ori(prvag + kIndexR);
    Eigen::Map<const Eigen::Vector3d> v(prvag + kIndexV);
    Eigen::Map<const Eigen::Vector3d> b_a(prvag + kIndexA);
    Eigen::Map<const Eigen::Vector3d> b_g(prvag + kIndexG);

    pose_.block<3, 1>(0, 3) = pos.cast<float>();
    pose_.block<3, 3>(0, 0) = Sophus::SO3d::exp(log_ori).matrix().cast<float>();

    vel_.v = v.cast<float>();

    bias_.accel = b_a.cast<float>();
    bias_.gyro = b_g.cast<float>();
  }

  Eigen::Quaternionf GetQuaternion() const {
    Eigen::Quaternionf q;
    q = pose_.block<3, 3>(0, 0);
    return q;
  }

  Eigen::Vector3f GetTranslation() const {
    Eigen::Vector3f t = pose_.block<3, 1>(0, 3);
    return t;
  }

  // key frame timestamp
  double time_{0.0};
  // key frame ID:
  unsigned int index_{0};
  // a. position & orientation:
  Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
  // b. velocity:
  struct {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    Eigen::Vector3f w = Eigen::Vector3f::Zero();
  } vel_;
  // c. bias:
  struct {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // c.1. accelerometer:
    Eigen::Vector3f accel = Eigen::Vector3f::Zero();
    // c.2. gyroscope:
    Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
  } bias_;

  // for GNSS:
  // 0: NONE
  // 55: INS_RTKFLOAT
  // 56: INS_RTKFIXED
  // for LiDAR odometery:
  // 0: good solution
  // 1: degeneracy
  int solution_status_{0};
};

}  // namespace lidar_localization

#endif
