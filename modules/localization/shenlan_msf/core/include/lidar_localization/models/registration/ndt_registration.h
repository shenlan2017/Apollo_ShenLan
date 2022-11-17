/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 21:40:40
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_H_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_H_

#include "glog/logging.h"

#include <pcl/registration/ndt.h>

#include "lidar_localization/models/registration/registration_interface.h"

namespace lidar_localization {
class NDTRegistration : public RegistrationInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  NDTRegistration(const YAML::Node& node);
  NDTRegistration(const float res,
                  const float step_size,
                  const float trans_eps,
                  const int max_iter);

  bool SetInputTarget(const CloudData::CloudTypePtr& input_target) override;

  bool ScanMatch(const CloudData::CloudTypePtr& input_source,
                 const Eigen::Matrix4f& predict_pose,
                 CloudData::CloudTypePtr& result_cloud_ptr,
                 Eigen::Matrix4f& result_pose) override;

  float GetFitnessScore() override;

  bool IsDegeneracy() override;

 private:
  bool SetRegistrationParam(const float res,
                            const float step_size,
                            const float trans_eps,
                            const int max_iter);

  pcl::NormalDistributionsTransform<CloudData::PointType,
                                    CloudData::PointType>::Ptr ndt_ptr_{};
};
}  // namespace lidar_localization

#endif
