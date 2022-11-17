/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-02 10:40:05
 */
#include "lidar_localization/models/registration/ndt_registration.h"

namespace lidar_localization {

NDTRegistration::NDTRegistration(const YAML::Node& node)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::PointType,
                                                     CloudData::PointType>()) {
  float res = node["res"].as<float>();
  float step_size = node["step_size"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  int max_iter = node["max_iter"].as<int>();

  SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(const float res,
                                 const float step_size,
                                 const float trans_eps,
                                 const int max_iter)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::PointType,
                                                     CloudData::PointType>()) {
  SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetRegistrationParam(const float res,
                                           const float step_size,
                                           const float trans_eps,
                                           const int max_iter) {
  ndt_ptr_->setResolution(res);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setTransformationEpsilon(trans_eps);
  ndt_ptr_->setMaximumIterations(max_iter);

  std::cout << "NDT params:" << std::endl
            << "res: " << res << ", "
            << "step_size: " << step_size << ", "
            << "trans_eps: " << trans_eps << ", "
            << "max_iter: " << max_iter << std::endl
            << std::endl;

  return true;
}

bool NDTRegistration::SetInputTarget(
    const CloudData::CloudTypePtr& input_target) {
  ndt_ptr_->setInputTarget(input_target);
  return true;
}

bool NDTRegistration::ScanMatch(const CloudData::CloudTypePtr& input_source,
                                const Eigen::Matrix4f& predict_pose,
                                CloudData::CloudTypePtr& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
  ndt_ptr_->setInputSource(input_source);
  ndt_ptr_->align(*result_cloud_ptr, predict_pose);
  result_pose = ndt_ptr_->getFinalTransformation();

  return true;
}

float NDTRegistration::GetFitnessScore() { return ndt_ptr_->getFitnessScore(); }

bool NDTRegistration::IsDegeneracy() {
  // 这种判断退化的方式非常不准
  return (ndt_ptr_->getFitnessScore() > 10);
}

}  // namespace lidar_localization