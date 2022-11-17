#include "lidar_localization/models/registration/point2plane_icp.h"

#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>

namespace lidar_localization {

void Point2PlaneICP::transformPoint(const CloudData::PointType& pi,
                                    const Eigen::Matrix4f& T,
                                    CloudData::PointType& po) {
  Eigen::Vector3f point_in(pi.x, pi.y, pi.z);
  Eigen::Vector3f point_out =
      T.block<3, 3>(0, 0) * point_in + T.block<3, 1>(0, 3);
  po.x = point_out(0, 0);
  po.y = point_out(1, 0);
  po.z = point_out(2, 0);
}

void Point2PlaneICP::DetectSurfaceFeaturePair(
    const pcl::PointCloud<CloudData::PointType>::Ptr& source_cloud,
    const pcl::PointCloud<CloudData::PointType>::Ptr& target_cloud,
    const Eigen::Matrix4f& T,
    std::vector<
        std::pair<Eigen::Vector3f, Eigen::Vector3f>,
        Eigen::aligned_allocator<std::pair<Eigen::Vector3f, Eigen::Vector3f>>>&
        surface_feature_pair,
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>&
        surface_normal) {
#pragma omp parallel num_threads(num_threads_)
  {
    std::vector<
        std::pair<Eigen::Vector3f, Eigen::Vector3f>,
        Eigen::aligned_allocator<std::pair<Eigen::Vector3f, Eigen::Vector3f>>>
        feature_pair_private;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> feature_normal_private;

#pragma omp for nowait
    for (size_t i = 0; i < source_cloud->size(); i++) {
      CloudData::PointType point_ori, point_sel;
      std::vector<int> point_search_index;
      std::vector<float> point_search_dist;

      point_ori = source_cloud->points[i];

      // 把特征点转到世界坐标系下
      transformPoint(point_ori, T, point_sel);

      // 在局部平面特征点云中 找5个与当前平面特征点相近的点
      if (pcl_isfinite(point_sel.x) && pcl_isfinite(point_sel.y) &&
          pcl_isfinite(point_sel.z)) {
        kdtree_from_map_->nearestKSearch(
            point_sel, 5, point_search_index, point_search_dist);
      } else {
        continue;
      }

      if (point_search_dist[4] < max_corr_dist_) {
        // 使用最小二乘 拟合平面方程 Ax + By + Cz + 1 = 0
        // 这里假设平面不通过原点可以少算一个参数
        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0 =
            -1 * Eigen::Matrix<float, 5, 1>::Ones();

        for (int j = 0; j < 5; j++) {
          matA0(j, 0) = target_cloud->at(point_search_index.at(j)).x;
          matA0(j, 1) = target_cloud->at(point_search_index.at(j)).y;
          matA0(j, 2) = target_cloud->at(point_search_index.at(j)).z;
        }
        // 用QR分解 求出 A B C
        Eigen::Vector3f plane_unit = matA0.colPivHouseholderQr().solve(matB0);
        Eigen::Vector3f plane_unit_norm = plane_unit.normalized();
        double negativ_OA_dot_norm = 1 / plane_unit.norm();

        // if OX * n > 0.2, then plane is not fit well
        // 点到面距离小于0.2则证明拟合得好 点到面的距离公式 fabs(Ax0 + By0 + Cz0
        // + 1)/sqrt(A^2+B^2+C^2)
        bool plane_valid = true;
        for (int j = 0; j < 5; j++) {
          if (fabs(plane_unit_norm(0) *
                       target_cloud->points[point_search_index[j]].x +
                   plane_unit_norm(1) *
                       target_cloud->points[point_search_index[j]].y +
                   plane_unit_norm(2) *
                       target_cloud->points[point_search_index[j]].z +
                   negativ_OA_dot_norm) > 0.2) {
            plane_valid = false;
            break;
          }
        }

        if (plane_valid) {
          // 取变换后的点 因为是用SVD求增量
          Eigen::Vector3f select_point(point_sel.x, point_sel.y, point_sel.z);

          // 最近的点作为匹配点
          Eigen::Vector3f match_point(
              target_cloud->points[point_search_index[0]].x,
              target_cloud->points[point_search_index[0]].y,
              target_cloud->points[point_search_index[0]].z);

          feature_pair_private.push_back(
              std::make_pair(match_point, select_point));
          feature_normal_private.push_back(plane_unit_norm);
        }
      }
    }

#pragma omp critical
    if (!feature_pair_private.empty() && !feature_normal_private.empty()) {
      surface_feature_pair.insert(surface_feature_pair.end(),
                                  feature_pair_private.begin(),
                                  feature_pair_private.end());
      surface_normal.insert(surface_normal.end(),
                            feature_normal_private.begin(),
                            feature_normal_private.end());
    }
  }
}

bool Point2PlaneICP::Optimization(
    const std::vector<
        std::pair<Eigen::Vector3f, Eigen::Vector3f>,
        Eigen::aligned_allocator<std::pair<Eigen::Vector3f, Eigen::Vector3f>>>&
        surface_feature_pair,
    const std::vector<Eigen::Vector3f,
                      Eigen::aligned_allocator<Eigen::Vector3f>>&
        surface_normal,
    Eigen::Matrix4f& T) {
  const int surface_num = surface_feature_pair.size();
  if (surface_num < 3) {
    // ROS_WARN("too little constrian!");
    return false;
  }

  Eigen::MatrixXf A = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>();
  A.resize(surface_num, 6);

  Eigen::MatrixXf B = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>();
  B.resize(surface_num, 1);

  // point to plane
  for (int i = 0; i < surface_num; i++) {
    Eigen::Vector3f n = surface_normal[i];
    Eigen::Vector3f d = surface_feature_pair[i].first;
    Eigen::Vector3f s = surface_feature_pair[i].second;

    // A
    float a1 = n.z() * s.y() - n.y() * s.z();
    float a2 = n.x() * s.z() - n.z() * s.x();
    float a3 = n.y() * s.x() - n.x() * s.y();

    A(i, 0) = a1;
    A(i, 1) = a2;
    A(i, 2) = a3;
    A(i, 3) = n.x();
    A(i, 4) = n.y();
    A(i, 5) = n.z();

    // B
    float temp1 = n.transpose() * d;
    float temp2 = n.transpose() * s;
    B(i, 0) = temp1 - temp2;
  }

  // 按照原始论文, 直接用svd求解即可
  // Eigen::Matrix<float, 6, 1> delta_x =
  //     A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

  // 为了判断是否发生退化, 这里改用QR分解求解
  Eigen::Matrix<float, 6, 6> ATA = A.transpose() * A;
  Eigen::Matrix<float, 6, 1> ATB = A.transpose() * B;
  Eigen::Matrix<float, 6, 1> delta_x = ATA.householderQr().solve(ATB);

  is_degeneracy_ = false;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> eg(ATA);
  Eigen::Matrix<float, 6, 1> eigen_value = eg.eigenvalues().real();
  for (size_t i = 0; i < 6; i++) {
    if (eigen_value(i, 0) < degeneracy_threshold_) {
      is_degeneracy_ = true;
      // LOG(INFO) << "lidar odomery is degeneracy, eigen value is "
      //           << eigen_value(i, 0) << " < 10.";
      break;
    }
  }
  // std::cout << "eigen value: " << eg.eigenvalues().real().transpose()
  //           << std::endl;

  float a = delta_x(0), b = delta_x(1), y = delta_x(2);
  Eigen::Matrix3f delta_R = Eigen::Matrix3f::Zero();
  delta_R(0, 0) = cos(y) * cos(b);
  delta_R(0, 1) = -sin(y) * cos(a) + cos(y) * sin(b) * sin(a);
  delta_R(0, 2) = sin(y) * sin(a) + cos(y) * sin(b) * cos(a);
  delta_R(1, 0) = sin(y) * cos(b);
  delta_R(1, 1) = cos(y) * cos(a) + sin(y) * sin(b) * sin(a);
  delta_R(1, 2) = -cos(y) * sin(a) + sin(y) * sin(b) * cos(a);
  delta_R(2, 0) = -sin(b);
  delta_R(2, 1) = cos(b) * sin(a);
  delta_R(2, 2) = cos(b) * cos(a);

  Eigen::Quaternionf delta_q = Eigen::Quaternionf(delta_R);
  delta_q.normalize();
  delta_R = delta_q.toRotationMatrix();

  Eigen::Vector3f delta_t = Eigen::Vector3f(delta_x(3), delta_x(4), delta_x(5));

  fitness_socre_ = 0;
  for (int i = 0; i < surface_num; i++) {
    Eigen::Vector3f n = surface_normal[i];
    Eigen::Vector3f d = surface_feature_pair[i].first;
    Eigen::Vector3f s = surface_feature_pair[i].second;

    s = delta_R * s + delta_t;

    fitness_socre_ += std::abs((s - d).transpose() * n);
  }
  fitness_socre_ /= (double)(surface_num);

  // 终止条件
  if (delta_t.norm() < trans_eps_ &&
      fabs(acos((delta_R.trace() - 1.0f) / 2.0f)) < trans_eps_) {
    return true;
  }

  // 更新
  Eigen::Quaternionf last_q(T.matrix().block<3, 3>(0, 0));
  Eigen::Vector3f last_t(T.matrix().block<3, 1>(0, 3));
  Eigen::Quaternionf curr_q = delta_q * last_q;
  curr_q.normalize();
  Eigen::Vector3f curr_t = delta_q * last_t + delta_t;

  T.block<3, 3>(0, 0) = curr_q.toRotationMatrix();
  T.block<3, 1>(0, 3) = curr_t;

  return false;
}

Point2PlaneICP::Point2PlaneICP(const YAML::Node& node)
    : input_target_(new CloudData::CloudType())
    , kdtree_from_map_(new pcl::KdTreeFLANN<CloudData::PointType>()) {
  trans_eps_ = node["trans_eps"].as<double>();
  max_corr_dist_ = node["max_corr_dis"].as<double>();
  max_iter_ = node["max_iter"].as<int>();
  num_threads_ = node["num_threads"].as<int>();
  degeneracy_threshold_ = node["degeneracy_threshold"].as<int>();

  if (num_threads_ == 0) {
    num_threads_ = omp_get_max_threads();
  }
  Eigen::setNbThreads(num_threads_);
  Eigen::initParallel();
}

bool Point2PlaneICP::SetInputTarget(
    const CloudData::CloudTypePtr& input_target) {
  input_target_ = input_target;
  kdtree_from_map_->setInputCloud(input_target);
  return true;
}

bool Point2PlaneICP::ScanMatch(const CloudData::CloudTypePtr& input_source,
                               const Eigen::Matrix4f& predict_pose,
                               CloudData::CloudTypePtr& result_cloud_ptr,
                               Eigen::Matrix4f& result_pose) {
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      surface_normal;
  std::vector<
      std::pair<Eigen::Vector3f, Eigen::Vector3f>,
      Eigen::aligned_allocator<std::pair<Eigen::Vector3f, Eigen::Vector3f>>>
      surface_feature_pair;

  int iterCount = 0;
  Eigen::Matrix4f T = predict_pose;
  while (iterCount < max_iter_) {
    surface_feature_pair.clear();
    surface_normal.clear();

    DetectSurfaceFeaturePair(
        input_source, input_target_, T, surface_feature_pair, surface_normal);

    if (Optimization(surface_feature_pair, surface_normal, T)) {
      break;
    }

    iterCount++;
  }

  result_pose = T;

  return true;
}

}  // namespace lidar_localization
