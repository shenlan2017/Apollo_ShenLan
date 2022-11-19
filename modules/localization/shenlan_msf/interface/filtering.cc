/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/shenlan_msf/interface/filtering.h"

namespace apollo {
namespace localization {

using apollo::cyber::Clock;
Filtering::Filtering()
    : global_map_ptr_(new CloudData::CloudType()),
      local_map_ptr_(new CloudData::CloudType()),
      current_scan_ptr_(new CloudData::CloudType()) {
  InitWithConfig();
}

bool Filtering::InitWithConfig(const CloudData& init_scan,
                                      const Eigen::Vector3f& init_vel,
                                      const IMUData& init_imu_data) {
  ResetLocalMap(relocalization_guess_pos_.x(), relocalization_guess_pos_.y(),
                relocalization_guess_pos_.z());

  relocalization_registration_ptr_->SetInputTarget(local_map_ptr_);

  // downsample:
  CloudData::CloudTypePtr filtered_cloud_ptr(new CloudData::CloudType());
  current_scan_filter_ptr_->Filter(init_scan.cloud_ptr_, filtered_cloud_ptr);

  Eigen::Matrix4f relocalization_pose = Eigen::Matrix4f::Identity();
  if (Relocalization(filtered_cloud_ptr, relocalization_pose)) {
    // current_vel_ = init_vel;
    current_vel_ = Eigen::Vector3f::Zero();

    kalman_filter_ptr_->Init(relocalization_pose.cast<double>(),
                             current_vel_.cast<double>(), init_imu_data);

    SetInitPose(relocalization_pose);
    has_inited_ = true;
    return true;
  } else {
    LOG(INFO) << "relocalization fail, please set right guess_pos.";
  }

  return false;
}

bool Filtering::Relocalization(
    const CloudData::CloudTypePtr& current_scan_ptr,
    Eigen::Matrix4f& relocalization_pose) {
  const size_t N = 36;

  // TODO: test greater
  std::multimap<
      double, Eigen::Matrix4f, std::greater<double>,
      Eigen::aligned_allocator<std::pair<const double, Eigen::Matrix4f>>>
      result;

  for (size_t i = 0; i < N; i++) {
    double guess_yaw = ((double)(i) * (2.0 * M_PI)) / (double)(N);
    if (guess_yaw > M_PI) {
      guess_yaw -= 2.0 * M_PI;
    }

    Eigen::AngleAxisf guess_orientation(guess_yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f guess_pose = Eigen::Matrix4f::Identity();
    guess_pose.block<3, 3>(0, 0) = guess_orientation.toRotationMatrix();
    guess_pose.block<3, 1>(0, 3) = relocalization_guess_pos_;

    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    CloudData::CloudTypePtr result_cloud_ptr(new CloudData::CloudType());
    relocalization_registration_ptr_->ScanMatch(current_scan_ptr, guess_pose,
                                                result_cloud_ptr, result_pose);
    double fitness_socre = relocalization_registration_ptr_->GetFitnessScore();
    LOG(INFO) << "i: " << i << ", yaw: " << guess_yaw
              << ", socre: " << fitness_socre;
    result.insert(std::make_pair(fitness_socre, result_pose));
  }

  if (result.begin()->first < fitness_score_threshold_) {
    relocalization_pose = result.begin()->second;

    LOG(INFO) << "Relocalization successed, fitness_socre: "
              << result.begin()->first << " < " << fitness_score_threshold_
              << std::endl
              << "relocalization_pose: " << std::endl
              << relocalization_pose;

    return true;
  }

  // 下次重定位根据本次的最优位置进行
  relocalization_guess_pos_ = result.begin()->second.block<3, 1>(0, 3);

  return false;
}

bool Filtering::InitWithGNSS(const Eigen::Matrix4f& init_pose,
                                    const Eigen::Vector3f& init_vel,
                                    const IMUData& init_imu_data) {
  if (SetInitGNSS(init_pose)) {
    current_vel_ = init_vel;

    // kalman_filter_ptr_->Init(current_vel_.cast<double>(), init_imu_data);
    kalman_filter_ptr_->Init(init_pose.cast<double>(),
                             current_vel_.cast<double>(), init_imu_data);

    return true;
  }

  return false;
}

bool Filtering::Init(const CloudData& init_scan,
                            const Eigen::Matrix4f& init_pose,
                            const Eigen::Vector3f& init_vel,
                            const IMUData& init_imu_data) {
  if (enable_relocalization_) {
    return InitWithConfig(init_scan, init_vel, init_imu_data);
  } else {
    return InitWithGNSS(init_pose, init_vel, init_imu_data);
  }

  return false;
}

bool Filtering::Update(const IMUData& imu_data) {
  if (kalman_filter_ptr_->Update(imu_data)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);
    return true;
  }

  return false;
}

bool Filtering::Correct(const IMUData& imu_data,
                               const CloudData& cloud_data,
                               const PosVelData& pos_vel_data,
                               Eigen::Matrix4f& cloud_pose) {
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;

  // remove invalid measurements:
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_, *cloud_data.cloud_ptr_,
                               indices);

  // downsample:
  CloudData::CloudTypePtr filtered_cloud_ptr(new CloudData::CloudType());
  current_scan_filter_ptr_->Filter(cloud_data.cloud_ptr_, filtered_cloud_ptr);

  // if (!has_inited_) {
  //   predict_pose = current_gnss_pose_;
  // }

  // matching:
  CloudData::CloudTypePtr result_cloud_ptr(new CloudData::CloudType());
  // registration_ptr_->ScanMatch(
  //     filtered_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);
  registration_ptr_->ScanMatch(filtered_cloud_ptr, current_pose_,
                               result_cloud_ptr, cloud_pose);
  pcl::transformPointCloud(*cloud_data.cloud_ptr_, *current_scan_ptr_,
                           cloud_pose);

  // update predicted pose:
  step_pose = last_pose.inverse() * cloud_pose;
  predict_pose = cloud_pose * step_pose;
  last_pose = cloud_pose;

  // shall the local map be updated:
  std::vector<float> edge = local_map_segmenter_ptr_->edge();
  // for (int i = 0; i < 3; i++) {
  // if (fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
  //     fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0) {
  //   continue;
  // }

  ResetLocalMap(cloud_pose(0, 3), cloud_pose(1, 3), cloud_pose(2, 3));
  //  break;
  //}

  // set lidar measurement:
  current_measurement_.time = cloud_data.time_;
  // current_measurement_.T_nb =
  //     (init_pose_.inverse() * cloud_pose).cast<double>();
  current_measurement_.T_nb = cloud_pose.cast<double>();
  // current_measurement_.v_b = pos_vel_data.vel_.cast<double>();
  // 车辆y轴朝前, 没有横向速度
  current_measurement_.v_b = Eigen::Vector3d(0.0, pos_vel_data.vel_.x(), 0.0);
  current_measurement_.w_b = Eigen::Vector3d(imu_data.angular_velocity_.x,
                                             imu_data.angular_velocity_.y,
                                             imu_data.angular_velocity_.z);
  current_measurement_.p_nb = pos_vel_data.pos_.cast<double>();

  // Kalman correction:
  if (kalman_filter_ptr_->Correct(imu_data, config_.fusion_strategy,
                                  current_measurement_)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);

    return true;
  }

  return false;
}

void Filtering::GetGlobalMap(CloudData::CloudTypePtr& global_map) {
  // downsample global map for visualization:
  global_map_filter_ptr_->Filter(global_map_ptr_, global_map);
  has_new_global_map_ = false;
}

void Filtering::GetOdometry(Eigen::Matrix4f& pose,
                                   Eigen::Vector3f& vel) {
  // pose = init_pose_ * current_pose_;
  // vel = init_pose_.block<3, 3>(0, 0) * current_vel_;
  pose = current_pose_;
  vel = current_vel_;
}

bool Filtering::InitWithConfig() {
  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/filtering/shenlan_filtering.yaml";

  YAML::Node config_node = YAML::LoadFile(config_file_path);
  LOG(INFO) << std::endl
            << "-----------------Init IMU-Lidar Fusion for "
               "Localization-------------------"
            << std::endl;

  // a. init filters:
  InitFilters(config_node);
  // b. init map:
  InitGlobalMap(config_node);
  // c. init frontend:
  InitRegistration(config_node, registration_ptr_);
  // d. init fusion:
  InitFusion(config_node);

  InitRelocalization(config_node);

  // init local map for frontend matching:
  ResetLocalMap(0.0, 0.0, 0.0);

  return true;
}

bool Filtering::InitFilter(
    const YAML::Node& config_node, const std::string& filter_user,
    std::shared_ptr<CloudFilterInterface>& filter_ptr) {
  const std::string filter_mothod =
      config_node[filter_user + "_filter"].as<std::string>();

  std::cout << "\tFilter Method for " << filter_user << ": " << filter_mothod
            << std::endl;

  if (filter_mothod == "voxel_filter") {
    filter_ptr = std::shared_ptr<VoxelFilter>(
        new VoxelFilter(config_node[filter_mothod][filter_user]));
  } else if (filter_mothod == "no_filter") {
    filter_ptr = std::shared_ptr<NoFilter>(new NoFilter);
  } else {
    LOG(ERROR) << "Filter method " << filter_mothod << " for " << filter_user
               << " NOT FOUND!";
    return false;
  }

  return true;
}

bool Filtering::InitLocalMapSegmenter(const YAML::Node& config_node) {
  local_map_segmenter_ptr_ =
      std::shared_ptr<BoxFilter>(new BoxFilter(config_node));
  return true;
}

bool Filtering::InitFilters(const YAML::Node& config_node) {
  // a. global map filter -- downsample point cloud map for visualization:
  InitFilter(config_node, "global_map", global_map_filter_ptr_);
  // b. local map filter -- downsample & ROI filtering for scan-map matching:
  InitLocalMapSegmenter(config_node);
  InitFilter(config_node, "local_map", local_map_filter_ptr_);
  // c. scan filter --
  InitFilter(config_node, "current_scan", current_scan_filter_ptr_);

  return true;
}

bool Filtering::InitGlobalMap(const YAML::Node& config_node) {
  map_path_ = config_node["map_path"].as<std::string>();

  pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
  LOG(INFO) << "Load global map, size:" << global_map_ptr_->points.size();

  // since scan-map matching is used, here apply the same filter to local map &
  // scan:
  local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
  LOG(INFO) << "Filtered global map, size:" << global_map_ptr_->points.size();

  has_new_global_map_ = true;

  return true;
}

bool Filtering::InitRegistration(
    const YAML::Node& config_node,
    std::shared_ptr<RegistrationInterface>& registration_ptr) {
  std::string registration_method =
      config_node["registration_method"].as<std::string>();

  LOG(INFO) << "\tPoint Cloud Registration Method: " << registration_method
            << std::endl;

  if (registration_method == "NDT") {
    registration_ptr = std::shared_ptr<NDTRegistration>(
        new NDTRegistration(config_node[registration_method]));
  } else if (registration_method == "Point2Plane") {
    registration_ptr = std::shared_ptr<Point2PlaneICP>(
        new Point2PlaneICP(config_node[registration_method]));
  } else {
    LOG(ERROR) << "Registration method " << registration_method
               << " NOT FOUND!";
    return false;
  }

  return true;
}

bool Filtering::InitFusion(const YAML::Node& config_node) {
  // set up fusion strategy:
  config_.fusion_strategy_id["pose"] = KalmanFilter::MeasurementType::POSE;
  config_.fusion_strategy_id["pose_position"] =
      KalmanFilter::MeasurementType::POSE_POSI;
  config_.fusion_strategy_id["pose_velocity"] =
      KalmanFilter::MeasurementType::POSE_VEL;
  config_.fusion_strategy_id["pose_velocity_constrain"] =
      KalmanFilter::MeasurementType::POSE_VEL_CONS;
  config_.fusion_strategy_id["position"] = KalmanFilter::MeasurementType::POSI;
  config_.fusion_strategy_id["position_velocity"] =
      KalmanFilter::MeasurementType::POSI_VEL;

  std::string fusion_strategy =
      config_node["fusion_strategy"].as<std::string>();

  if (config_.fusion_strategy_id.end() !=
      config_.fusion_strategy_id.find(fusion_strategy)) {
    config_.fusion_strategy = config_.fusion_strategy_id.at(fusion_strategy);
  } else {
    LOG(ERROR) << "Fusion strategy " << fusion_strategy << " NOT FOUND!";
    return false;
  }
  std::cout << "\tGNSS-INS-Sim Localization Fusion Strategy: "
            << fusion_strategy << std::endl;

  // set up fusion method:
  config_.fusion_method = config_node["fusion_method"].as<std::string>();

  if (config_.fusion_method == "error_state_kalman_filter") {
    kalman_filter_ptr_ = std::shared_ptr<ErrorStateKalmanFilter>(
        new ErrorStateKalmanFilter(config_node[config_.fusion_method]));
  } else {
    LOG(ERROR) << "Fusion method " << config_.fusion_method << " NOT FOUND!";
    return false;
  }
  LOG(INFO) << "\tKITTI Localization Fusion Method: " << config_.fusion_method
            << std::endl;

  return true;
}

bool Filtering::InitRelocalization(const YAML::Node& config_node) {
  relocalization_registration_ptr_ = std::shared_ptr<NDTRegistration>(
      new NDTRegistration(config_node["relocalization"]));

  enable_relocalization_ =
      config_node["relocalization"]["enable_relocalization"].as<bool>();

  relocalization_guess_pos_.x() =
      config_node["relocalization"]["guess_pos"][0].as<float>();
  relocalization_guess_pos_.y() =
      config_node["relocalization"]["guess_pos"][1].as<float>();
  relocalization_guess_pos_.z() =
      config_node["relocalization"]["guess_pos"][2].as<float>();

  fitness_score_threshold_ =
      config_node["relocalization"]["fitness_socre_threshold"].as<double>();

  LOG(INFO) << "\tenable_relocalization: " << enable_relocalization_
            << std::endl
            << "\tfitness_score_threshold: " << fitness_score_threshold_
            << std::endl
            << "\tguess_pos: " << relocalization_guess_pos_.transpose()
            << std::endl;

  return true;
}

bool Filtering::SetInitGNSS(const Eigen::Matrix4f& gnss_pose) {
  SetInitPose(gnss_pose);
  has_inited_ = true;

  return true;
}

bool Filtering::SetInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;

  ResetLocalMap(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));

  return true;
}

bool Filtering::ResetLocalMap(const float x, const float y,
                                     const float z) {
  const std::vector<float> origin = {x, y, z};

  // segment local map from global map:
  local_map_segmenter_ptr_->set_origin(origin);
  local_map_segmenter_ptr_->Filter(global_map_ptr_, local_map_ptr_);
  registration_ptr_->SetInputTarget(local_map_ptr_);
  has_new_local_map_ = true;

  const std::vector<float> edge = local_map_segmenter_ptr_->edge();

  LOG(INFO) << "New local map:" << edge.at(0) << "," << edge.at(1) << ","
            << edge.at(2) << "," << edge.at(3) << "," << edge.at(4) << ","
            << edge.at(5) << std::endl
            << std::endl;

  return true;
}

}  // namespace localization
}  // namespace apollo
