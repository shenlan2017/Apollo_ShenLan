/*
 * @Description: 打印信息
 * @Author: Ren Qian
 * @Date: 2020-03-02 23:25:26
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 10:47:48
 */
#ifndef LIDAR_LOCALIZATION_TOOLS_PRINT_INFO_H_
#define LIDAR_LOCALIZATION_TOOLS_PRINT_INFO_H_

#include <cmath>
#include <string>

#include "glog/logging.h"

#include <Eigen/Dense>

#include <pcl/common/eigen.h>

namespace lidar_localization {
class PrintInfo {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  static void PrintPose(const std::string head, const Eigen::Matrix4f pose);
};
}  // namespace lidar_localization
#endif
