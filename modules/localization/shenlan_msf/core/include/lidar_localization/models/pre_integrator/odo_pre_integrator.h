/*
 * @Description: Odometer pre-integrator for LIO mapping, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 20:14:16
 */

#ifndef LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_ODO_PRE_INTEGRATOR_H_
#define LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_ODO_PRE_INTEGRATOR_H_

#include <sophus/so3.hpp>

#include "lidar_localization/models/graph_optimizer/g2o/edge/edge_prvag_odo_pre_integration.h"
#include "lidar_localization/models/pre_integrator/pre_integrator.h"
#include "lidar_localization/sensor_data/velocity_data.h"

namespace lidar_localization {

class OdoPreIntegrator : public PreIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  static const int DIM_STATE = 6;

  typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixP;

  struct OdoPreIntegration {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // a. measurement:
    // a.1. relative translation:
    Eigen::Vector3d alpha_ij_;
    // a.2. relative orientation:
    Sophus::SO3d theta_ij_;

    // b. information:
    MatrixP P_;

    g2o::EdgePRVAGOdoPreIntegration::Measurement GetMeasurement() const {
      g2o::EdgePRVAGOdoPreIntegration::Measurement measurement =
          g2o::EdgePRVAGOdoPreIntegration::Measurement::Zero();

      measurement.block<3, 1>(
          g2o::EdgePRVAGOdoPreIntegration::EdgePRVAGOdoPreIntegration::kIndexP,
          0) = alpha_ij_;
      measurement.block<3, 1>(
          g2o::EdgePRVAGOdoPreIntegration::EdgePRVAGOdoPreIntegration::kIndexR,
          0) = theta_ij_.log();

      return measurement;
    }

    Eigen::MatrixXd GetInformation() const { return P_.inverse(); }
  };

  OdoPreIntegrator(const YAML::Node& node);

  /**
   * @brief  init odo pre-integrator
   * @param  init_velocity_data, init odometer measurement
   * @return true if success false otherwise
   */
  bool Init(const VelocityData& init_velocity_data);

  /**
   * @brief  update odo pre-integrator
   * @param  velocity_data, current odometer measurement
   * @return true if success false otherwise
   */
  bool Update(const VelocityData& velocity_data);

  /**
   * @brief  reset odo pre-integrator using new init odo measurement
   * @param  init_velocity_data, new init odo measurements
   * @param  output pre-integration result for constraint building as
   * OdoPreIntegration
   * @return true if success false otherwise
   */
  bool Reset(const VelocityData& init_velocity_data,
             OdoPreIntegration& odo_pre_integration);

 private:
  static const int DIM_NOISE = 12;

  static const int INDEX_ALPHA = 0;
  static const int INDEX_THETA = 3;

  static const int INDEX_M_V_PREV = 0;
  static const int INDEX_M_W_PREV = 3;
  static const int INDEX_M_V_CURR = 6;
  static const int INDEX_M_W_CURR = 9;

  typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixF;
  typedef Eigen::Matrix<double, DIM_STATE, DIM_NOISE> MatrixB;
  typedef Eigen::Matrix<double, DIM_NOISE, DIM_NOISE> MatrixQ;

  // data buff:
  std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>>
      odo_data_buff_;

  // hyper-params:
  // a. prior state covariance, process & measurement noise:
  struct {
    struct {
      double V;
      double W;
    } MEASUREMENT;
  } COV;

  // pre-integration state:
  struct {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // a. relative translation:
    Eigen::Vector3d alpha_ij_;
    // b. relative orientation:
    Sophus::SO3d theta_ij_;
  } state;

  // state covariance:
  MatrixP P_ = MatrixP::Zero();

  // process noise:
  MatrixQ Q_ = MatrixQ::Zero();

  // process equation:
  MatrixF F_ = MatrixF::Zero();
  MatrixB B_ = MatrixB::Zero();

  /**
   * @brief  reset pre-integrator state using odo measurement
   * @param  void
   * @return void
   */
  void ResetState(const VelocityData& init_velocity_data);

  /**
   * @brief  update pre-integrator state: mean, covariance
   * @param  void
   * @return void
   */
  void UpdateState();
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_ODO_PRE_INTEGRATOR_H_
