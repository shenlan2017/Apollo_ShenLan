/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2020-03-02 23:28:54
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 10:48:12
 */
#include "lidar_localization/tools/print_info.h"

namespace lidar_localization {
void PrintInfo::PrintPose(const std::string head, const Eigen::Matrix4f pose) {
  Eigen::Affine3f aff_pose;
  aff_pose.matrix() = pose;
  float x, y, z, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(aff_pose, x, y, z, roll, pitch, yaw);
  std::cout << head << x << "," << y << "," << z << "," << roll * 180 / M_PI
            << "," << pitch * 180 / M_PI << "," << yaw * 180 / M_PI
            << std::endl;
}
}  // namespace lidar_localization