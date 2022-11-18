/*
 * @Description: g2o edge for LIO IMU pre-integration measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-28 21:20:53
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_IMU_PRE_INTEGRATION_H_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_IMU_PRE_INTEGRATION_H_

#include <iostream>

#include "glog/logging.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.h"

using Vector15d = Eigen::Matrix<double, 15, 1>;

namespace g2o {

class EdgePRVAGIMUPreIntegration
    : public g2o::
          BaseBinaryEdge<15, Vector15d, g2o::VertexPRVAG, g2o::VertexPRVAG> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int kIndexP{0};
  static const int kIndexR{3};
  static const int kIndexV{6};
  static const int kIndexA{9};
  static const int kIndexG{12};

  EdgePRVAGIMUPreIntegration()
      : g2o::BaseBinaryEdge<15,
                            Vector15d,
                            g2o::VertexPRVAG,
                            g2o::VertexPRVAG>() {}

  virtual void computeError() override {
    g2o::VertexPRVAG* v0 = dynamic_cast<g2o::VertexPRVAG*>(_vertices[0]);
    g2o::VertexPRVAG* v1 = dynamic_cast<g2o::VertexPRVAG*>(_vertices[1]);

    const Eigen::Vector3d& pos_i = v0->estimate().pos_;
    const Sophus::SO3d& ori_i = v0->estimate().ori_;
    const Eigen::Vector3d& vel_i = v0->estimate().vel_;
    const Eigen::Vector3d& b_a_i = v0->estimate().b_a_;
    const Eigen::Vector3d& b_g_i = v0->estimate().b_g_;

    const Eigen::Vector3d& pos_j = v1->estimate().pos_;
    const Sophus::SO3d& ori_j = v1->estimate().ori_;
    const Eigen::Vector3d& vel_j = v1->estimate().vel_;
    const Eigen::Vector3d& b_a_j = v1->estimate().b_a_;
    const Eigen::Vector3d& b_g_j = v1->estimate().b_g_;

    // update pre-integration measurement caused by bias change:
    // if (v0->isUpdated()) {
    //   Eigen::Vector3d d_b_a_i, d_b_g_i;

    //   v0->getDeltaBiases(d_b_a_i, d_b_g_i);

    //   updateMeasurement(d_b_a_i, d_b_g_i);
    // }

    // const Eigen::Vector3d& alpha_ij = _measurement.block<3, 1>(kIndexP, 0);
    // const Eigen::Vector3d& theta_ij = _measurement.block<3, 1>(kIndexR, 0);
    // const Eigen::Vector3d& beta_ij = _measurement.block<3, 1>(kIndexV, 0);

    // _error.block<3, 1>(kIndexP, 0) =
    //     ori_i.inverse() * (pos_j - pos_i - (vel_i - 0.50 * g_ * T_) * T_) -
    //     alpha_ij;
    // _error.block<3, 1>(kIndexR, 0) =
    //     (Sophus::SO3d::exp(theta_ij).inverse() * ori_i.inverse() *
    //     ori_j).log();
    // _error.block<3, 1>(kIndexV, 0) =
    //     ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij;
    // _error.block<3, 1>(kIndexA, 0) = b_a_j - b_a_i;
    // _error.block<3, 1>(kIndexG, 0) = b_g_j - b_g_i;

    Eigen::Vector3d d_b_a_i, d_b_g_i;
    v0->getDeltaBiases(d_b_a_i, d_b_g_i);
    updateMeasurement(d_b_a_i, d_b_g_i);

    _error.block<3, 1>(kIndexP, 0) =
        ori_i.inverse() * (pos_j - pos_i - (vel_i - 0.50 * g_ * T_) * T_) -
        alpha_ij_;
    _error.block<3, 1>(kIndexR, 0) =
        (Sophus::SO3d::exp(theta_ij_).inverse() * ori_i.inverse() * ori_j)
            .log();
    _error.block<3, 1>(kIndexV, 0) =
        ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij_;
    _error.block<3, 1>(kIndexA, 0) = b_a_j - b_a_i;
    _error.block<3, 1>(kIndexG, 0) = b_g_j - b_g_i;
  }

  void setT(const double& T) { T_ = T; }

  void setGravitiy(const Eigen::Vector3d& g) { g_ = g; }

  void setJacobian(const Eigen::MatrixXd& J) { J_ = J; }

  virtual void setMeasurement(const Vector15d& m) override { _measurement = m; }

  void updateMeasurement(const Eigen::Vector3d& d_b_a_i,
                         const Eigen::Vector3d& d_b_g_i) {
    // _measurement.block<3, 1>(kIndexP, 0) +=
    //     (J_.block<3, 3>(kIndexP, kIndexA) * d_b_a_i +
    //      J_.block<3, 3>(kIndexP, kIndexG) * d_b_g_i);
    // _measurement.block<3, 1>(kIndexR, 0) =
    //     (Sophus::SO3d::exp(_measurement.block<3, 1>(kIndexR, 0)) *
    //      Sophus::SO3d::exp(J_.block<3, 3>(kIndexR, kIndexG) * d_b_g_i))
    //         .log();
    // _measurement.block<3, 1>(kIndexV, 0) +=
    //     (J_.block<3, 3>(kIndexV, kIndexA) * d_b_a_i +
    //      J_.block<3, 3>(kIndexV, kIndexG) * d_b_g_i);

    alpha_ij_ = _measurement.block<3, 1>(kIndexP, 0) +
                (J_.block<3, 3>(kIndexP, kIndexA) * d_b_a_i +
                 J_.block<3, 3>(kIndexP, kIndexG) * d_b_g_i);
    theta_ij_ = (Sophus::SO3d::exp(_measurement.block<3, 1>(kIndexR, 0)) *
                 Sophus::SO3d::exp(J_.block<3, 3>(kIndexR, kIndexG) * d_b_g_i))
                    .log();
    beta_ij_ = _measurement.block<3, 1>(kIndexV, 0) +
               (J_.block<3, 3>(kIndexV, kIndexA) * d_b_a_i +
                J_.block<3, 3>(kIndexV, kIndexG) * d_b_g_i);
  }

  virtual bool read(std::istream& is) override {
    double T;
    is >> T;

    Eigen::Vector3d g;
    is >> g.x() >> g.y() >> g.z();

    Vector15d v;
    is >> v(kIndexP + 0) >> v(kIndexP + 1) >> v(kIndexP + 2) >>
        v(kIndexR + 0) >> v(kIndexR + 1) >> v(kIndexR + 2) >> v(kIndexV + 0) >>
        v(kIndexV + 1) >> v(kIndexV + 2) >> v(kIndexA + 0) >> v(kIndexA + 1) >>
        v(kIndexA + 2) >> v(kIndexG + 0) >> v(kIndexG + 1) >> v(kIndexG + 2);

    setT(T);
    setGravitiy(g);
    setMeasurement(v);

    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        // update cross-diagonal element:
        if (i != j) {
          information()(j, i) = information()(i, j);
        }
      }
    }

    return true;
  }

  virtual bool write(std::ostream& os) const override {
    Vector15d v = _measurement;

    os << T_ << " ";

    os << g_.x() << " " << g_.y() << " " << g_.z() << " ";

    os << v(kIndexP + 0) << " " << v(kIndexP + 1) << " " << v(kIndexP + 2)
       << " " << v(kIndexR + 0) << " " << v(kIndexR + 1) << " "
       << v(kIndexR + 2) << " " << v(kIndexV + 0) << " " << v(kIndexA + 1)
       << " " << v(kIndexA + 2) << " " << v(kIndexA + 0) << " "
       << v(kIndexV + 1) << " " << v(kIndexV + 2) << " " << v(kIndexG + 0)
       << " " << v(kIndexG + 1) << " " << v(kIndexG + 2) << " ";

    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);

    return os.good();
  }

 private:
  double T_{0.0};

  Eigen::Vector3d g_ = Eigen::Vector3d::Zero();

  Eigen::MatrixXd J_;

  Eigen::Vector3d alpha_ij_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d theta_ij_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d beta_ij_ = Eigen::Vector3d::Zero();
};

}  // namespace g2o

#endif  // LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_IMU_PRE_INTEGRATION_H_
