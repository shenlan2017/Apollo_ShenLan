/*
 * @Description: 不滤波
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:37:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 21:06:40
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_H_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_H_

#include "glog/logging.h"

#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"

namespace lidar_localization {

class NoFilter : public CloudFilterInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  NoFilter();

  bool Filter(const CloudData::CloudTypePtr& input_cloud_ptr,
              CloudData::CloudTypePtr& filtered_cloud_ptr) override;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_H_
