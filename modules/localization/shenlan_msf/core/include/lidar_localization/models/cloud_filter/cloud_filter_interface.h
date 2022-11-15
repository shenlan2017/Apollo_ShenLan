/*
 * @Description: 点云滤波模块的接口
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:29:50
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-20 21:49:28
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_H_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_H_

#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.h"

namespace lidar_localization {
class CloudFilterInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  virtual ~CloudFilterInterface() = default;

  virtual bool Filter(const CloudData::CloudTypePtr& input_cloud_ptr,
                      CloudData::CloudTypePtr& filtered_cloud_ptr) = 0;
};
}  // namespace lidar_localization

#endif
