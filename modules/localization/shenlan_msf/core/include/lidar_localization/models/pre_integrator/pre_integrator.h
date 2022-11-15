/*
 * @Description: pre-integrator interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-20 22:54:14
 */

#ifndef LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_PRE_INTEGRATOR_H_
#define LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_PRE_INTEGRATOR_H_

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace lidar_localization {

class PreIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  /**
   * @brief  whether the pre-integrator is inited:
   * @return true if inited false otherwise
   */
  double IsInited() const { return is_inited_; }

  /**
   * @brief  get pre-integrator time
   * @return pre-integrator time as double
   */
  double GetTime() const { return time_; }

 protected:
  PreIntegrator() {}

  // init:
  bool is_inited_ = false;

  // time:
  double time_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_PRE_INTEGRATOR_H_
