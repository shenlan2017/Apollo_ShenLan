/*
 * @Description: a scan-to-map registration method with point-to-plane metric
 * reference from "Linear Least-Squares Optimization for Point-to-Plane ICP
 * Surface Registration"
 *
 * @Autor: ZiJieChen
 * @Date: 2022-10-31 11:20:37
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-02 16:39:41
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_POINT_2_PLANE_ICP_H_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_POINT_2_PLANE_ICP_H_

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "glog/logging.h"
#include "lidar_localization/models/registration/registration_interface.h"

namespace lidar_localization {

class Point2PlaneICP : public RegistrationInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  Point2PlaneICP(const YAML::Node& node);

  bool SetInputTarget(const CloudData::CloudTypePtr& input_target) override;

  bool ScanMatch(const CloudData::CloudTypePtr& input_source,
                 const Eigen::Matrix4f& predict_pose,
                 CloudData::CloudTypePtr& result_cloud_ptr,
                 Eigen::Matrix4f& result_pose) override;

  float GetFitnessScore() override { return fitness_socre_; }

  bool IsDegeneracy() override { return is_degeneracy_; }

 private:
  void DetectSurfaceFeaturePair(
      const pcl::PointCloud<CloudData::PointType>::Ptr& source_cloud,
      const pcl::PointCloud<CloudData::PointType>::Ptr& target_cloud,
      const Eigen::Matrix4f& T,
      std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>,
                  Eigen::aligned_allocator<
                      std::pair<Eigen::Vector3f, Eigen::Vector3f>>>&
          surface_feature_pair,
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>&
          surface_normal);

  bool Optimization(
      const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>,
                        Eigen::aligned_allocator<
                            std::pair<Eigen::Vector3f, Eigen::Vector3f>>>&
          surface_feature_pair,
      const std::vector<Eigen::Vector3f,
                        Eigen::aligned_allocator<Eigen::Vector3f>>&
          surface_normal,
      Eigen::Matrix4f& T);

  static void transformPoint(const CloudData::PointType& pi,
                             const Eigen::Matrix4f& T,
                             CloudData::PointType& po);

  double trans_eps_{0.01};
  int max_iter_{30};
  double max_corr_dist_{1.0};
  int num_threads_{4};
  double fitness_socre_{0.0};
  double degeneracy_threshold_{10.0};
  bool is_degeneracy_{false};

  CloudData::CloudTypePtr input_target_{};
  pcl::KdTreeFLANN<CloudData::PointType>::Ptr kdtree_from_map_{};
};

}  // namespace lidar_localization

#endif
