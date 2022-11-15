/*
 * @Description: 不滤波
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:53:20
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 21:06:41
 */
#include "lidar_localization/models/cloud_filter/no_filter.h"

namespace lidar_localization {

NoFilter::NoFilter() {}

bool NoFilter::Filter(const CloudData::CloudTypePtr& input_cloud_ptr,
                      CloudData::CloudTypePtr& filtered_cloud_ptr) {
  filtered_cloud_ptr.reset(new CloudData::CloudType(*input_cloud_ptr));
  return true;
}

}  // namespace lidar_localization