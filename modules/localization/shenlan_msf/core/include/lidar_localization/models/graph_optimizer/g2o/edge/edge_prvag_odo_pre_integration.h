/*
 * @Description: g2o edge for LIO odo pre-integration measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 20:20:45
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_ODO_PRE_INTEGRATION_H_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_ODO_PRE_INTEGRATION_H_

#include <iostream>

#include "glog/logging.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.h"

namespace g2o {

namespace EdgePRVAGOdoPreIntegration {

static const int kDim = 6;

typedef Eigen::Matrix<double, kDim, 1> Measurement;

class EdgePRVAGOdoPreIntegration
    : public g2o::BaseBinaryEdge<kDim,
                                 Measurement,
                                 g2o::VertexPRVAG,
                                 g2o::VertexPRVAG> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int kIndexP{0};
  static const int kIndexR{3};

  EdgePRVAGOdoPreIntegration()
      : g2o::BaseBinaryEdge<kDim,
                            Measurement,
                            g2o::VertexPRVAG,
                            g2o::VertexPRVAG>() {}

  virtual void computeError() override {
    g2o::VertexPRVAG* v0 = dynamic_cast<g2o::VertexPRVAG*>(_vertices[0]);
    g2o::VertexPRVAG* v1 = dynamic_cast<g2o::VertexPRVAG*>(_vertices[1]);

    const Eigen::Vector3d& pos_i = v0->estimate().pos_;
    const Sophus::SO3d& ori_i = v0->estimate().ori_;

    const Eigen::Vector3d& pos_j = v1->estimate().pos_;
    const Sophus::SO3d& ori_j = v1->estimate().ori_;

    const Eigen::Vector3d& alpha_ij = _measurement.block<3, 1>(kIndexP, 0);
    const Eigen::Vector3d& theta_ij = _measurement.block<3, 1>(kIndexR, 0);

    _error.block<3, 1>(kIndexP, 0) =
        ori_i.inverse() * (pos_j - pos_i) - alpha_ij;
    _error.block<3, 1>(kIndexR, 0) =
        (Sophus::SO3d::exp(theta_ij).inverse() * ori_i.inverse() * ori_j).log();
  }

  virtual void setMeasurement(const Measurement& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    Measurement m;
    is >> m(kIndexP + 0) >> m(kIndexP + 1) >> m(kIndexP + 2) >>
        m(kIndexR + 0) >> m(kIndexR + 1) >> m(kIndexR + 2);

    setMeasurement(m);

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
    Measurement m = _measurement;

    os << m(kIndexP + 0) << " " << m(kIndexP + 1) << " " << m(kIndexP + 2)
       << " " << m(kIndexR + 0) << " " << m(kIndexR + 1) << " "
       << m(kIndexR + 2) << " ";

    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);

    return os.good();
  }
};

}  // namespace EdgePRVAGOdoPreIntegration

}  // namespace g2o

#endif  // LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_ODO_PRE_INTEGRATION_H_
