/*
 * @Description: g2o vertex for LIO extended pose
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 21:20:16
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_VERTEX_VERTEX_PRVAG_H_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_VERTEX_VERTEX_PRVAG_H_

#include <g2o/core/base_vertex.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <mutex>
#include <sophus/so3.hpp>

namespace g2o {

struct PRVAG {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int kIndexPos{0};
  static const int kIndexOri{3};
  static const int kIndexVel{6};
  static const int kIndexBa{9};
  static const int kIndexBg{12};

  PRVAG() {}

  explicit PRVAG(const double* data) {
    pos_ = Eigen::Vector3d(
        data[kIndexPos + 0], data[kIndexPos + 1], data[kIndexPos + 2]);
    ori_ = Sophus::SO3d::exp(Eigen::Vector3d(
        data[kIndexOri + 0], data[kIndexOri + 1], data[kIndexOri + 2]));
    vel_ = Eigen::Vector3d(
        data[kIndexVel + 0], data[kIndexVel + 1], data[kIndexVel + 2]);
    b_a_ = Eigen::Vector3d(
        data[kIndexBa + 0], data[kIndexBa + 1], data[kIndexBa + 2]);
    b_g_ = Eigen::Vector3d(
        data[kIndexBg + 0], data[kIndexBg + 1], data[kIndexBg + 2]);
  }

  void write(double* data) const {
    // get orientation in so3:
    auto log_ori = ori_.log();

    for (size_t i = 0; i < 3; ++i) {
      data[kIndexPos + i] = pos_(i);
      data[kIndexOri + i] = log_ori(i);
      data[kIndexVel + i] = vel_(i);
      data[kIndexBa + i] = b_a_(i);
      data[kIndexBg + i] = b_g_(i);
    }
  }

  double time_{0.0};

  Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
  Sophus::SO3d ori_ = Sophus::SO3d();
  Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_a_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_g_ = Eigen::Vector3d::Zero();
};

class VertexPRVAG : public g2o::BaseVertex<15, PRVAG> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  virtual void setToOriginImpl() override { _estimate = PRVAG(); }

  virtual void oplusImpl(const double* update) override {
    _estimate.pos_ += Eigen::Vector3d(update[PRVAG::kIndexPos + 0],
                                      update[PRVAG::kIndexPos + 1],
                                      update[PRVAG::kIndexPos + 2]);
    _estimate.ori_ = _estimate.ori_ * Sophus::SO3d::exp(Eigen::Vector3d(
                                          update[PRVAG::kIndexOri + 0],
                                          update[PRVAG::kIndexOri + 1],
                                          update[PRVAG::kIndexOri + 2]));
    _estimate.vel_ += Eigen::Vector3d(update[PRVAG::kIndexVel + 0],
                                      update[PRVAG::kIndexVel + 1],
                                      update[PRVAG::kIndexVel + 2]);

    Eigen::Vector3d d_b_a_i(update[PRVAG::kIndexBa + 0],
                            update[PRVAG::kIndexBa + 1],
                            update[PRVAG::kIndexBa + 2]);
    Eigen::Vector3d d_b_g_i(update[PRVAG::kIndexBg + 0],
                            update[PRVAG::kIndexBg + 1],
                            update[PRVAG::kIndexBg + 2]);

    _estimate.b_a_ += d_b_a_i;
    _estimate.b_g_ += d_b_g_i;

    updateDeltaBiases(d_b_a_i, d_b_g_i);
  }

  bool isUpdated(void) const { return _is_updated; }

  void updateDeltaBiases(const Eigen::Vector3d& d_b_a_i,
                         const Eigen::Vector3d& d_b_g_i) {
    std::lock_guard<std::mutex> l(_m);

    _is_updated = true;

    d_b_a_i_ += d_b_a_i;
    d_b_g_i_ += d_b_g_i;
  }

  void getDeltaBiases(Eigen::Vector3d& d_b_a_i, Eigen::Vector3d& d_b_g_i) {
    std::lock_guard<std::mutex> l(_m);

    d_b_a_i = d_b_a_i_;
    d_b_g_i = d_b_g_i_;

    // d_b_a_i_ = d_b_g_i_ = Eigen::Vector3d::Zero();

    _is_updated = false;
  }

  virtual bool read(std::istream& in) { return true; }

  virtual bool write(std::ostream& out) const { return true; }

 private:
  std::mutex _m;
  bool _is_updated{false};

  Eigen::Vector3d d_b_a_i_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d d_b_g_i_ = Eigen::Vector3d::Zero();
};

}  // namespace g2o

#endif  // LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_VERTEX_VERTEX_PRVAG_H_
