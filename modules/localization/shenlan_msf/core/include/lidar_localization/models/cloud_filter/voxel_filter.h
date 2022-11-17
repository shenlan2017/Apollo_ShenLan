/*
 * @Description: voxel filter 模块
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:37:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 21:10:46
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_H_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_H_

#include "glog/logging.h"

#include <pcl/filters/voxel_grid.h>

#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"

namespace lidar_localization {
class VoxelFilter : public CloudFilterInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  VoxelFilter(const YAML::Node& node);

  VoxelFilter(const float leaf_size_x,
              const float leaf_size_y,
              const float leaf_size_z);

  bool Filter(const CloudData::CloudTypePtr& input_cloud_ptr,
              CloudData::CloudTypePtr& filtered_cloud_ptr) override;

 private:
  bool SetFilterParam(const float leaf_size_x,
                      const float leaf_size_y,
                      const float leaf_size_z);

  pcl::VoxelGrid<CloudData::PointType> voxel_filter_;
};
}  // namespace lidar_localization
#endif
