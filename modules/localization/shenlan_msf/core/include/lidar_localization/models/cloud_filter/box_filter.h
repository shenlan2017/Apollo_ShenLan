/*
 * @Description: 从点云中截取一个立方体部分
 * @Author: Ren Qian
 * @Date: 2020-03-04 20:09:37
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:17:33
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_H_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_BOX_FILTER_H_

#include <iostream>
#include <vector>

#include "glog/logging.h"

#include <pcl/filters/crop_box.h>

#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"

namespace lidar_localization {
class BoxFilter : public CloudFilterInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  BoxFilter(const YAML::Node& node);

  BoxFilter() = default;

  bool Filter(const CloudData::CloudTypePtr& input_cloud_ptr,
              CloudData::CloudTypePtr& filtered_cloud_ptr) override;

  const std::vector<float>& edge() const { return edge_; }

  void set_size(const std::vector<float>& size);

  void set_origin(const std::vector<float>& origin);

 private:
  void CalculateEdge();

  std::vector<float> origin_;
  std::vector<float> size_;
  std::vector<float> edge_;

};
}  // namespace lidar_localization

#endif
