/*
 * @Description:
 * @Autor: ZiJieChen
 * @Date: 2022-10-31 09:45:58
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-01 22:51:37
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_PRIOR_BODY_VEL_H_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_PRIOR_BODY_VEL_H_

#include <iostream>

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.h"

namespace g2o {

class EdgePRVAGPriorBodyVel
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPRVAG> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgePRVAGPriorBodyVel()
      : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPRVAG>() {}

  virtual void computeError() override {
    const g2o::VertexPRVAG* v0 =
        static_cast<const g2o::VertexPRVAG*>(_vertices[0]);

    const Sophus::SO3d& ori_i = v0->estimate().ori_;
    const Eigen::Vector3d& vel_i = v0->estimate().vel_;
    // 速度转到body系下
    const Eigen::Vector3d vel_i_b = ori_i.inverse() * vel_i;

    const Eigen::Vector3d vel_i_b_prior = _measurement;

    _error = vel_i_b_prior - vel_i_b;
  }

  virtual void setMeasurement(const Eigen::Vector3d& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override { return true; }

  virtual bool write(std::ostream& os) const override { return true; }
};
}  // namespace g2o

#endif