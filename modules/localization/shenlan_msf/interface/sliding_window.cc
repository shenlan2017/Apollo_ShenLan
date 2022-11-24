#include "modules/localization/shenlan_msf/interface/sliding_window.h"

namespace apollo{
namespace localization {

SlidingWindow::SlidingWindow(const std::string& config_file_path) {
  InitWithConfig(config_file_path);
}

bool SlidingWindow::InitWithConfig(const std::string& config_file_path) {

  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::cout
      << "-----------------Init LIO Localization, Backend-------------------"
      << std::endl;

  // a. estimation output path:
  InitDataPath(config_node);
  // b. key frame selection config:
  InitKeyFrameSelection(config_node);
  // c. sliding window config:
  InitSlidingWindow(config_node);
  // d. IMU pre-integration:
  InitIMUPreIntegrator(config_node);

  return true;
}

bool SlidingWindow::InitDataPath(const YAML::Node& config_node) {
  std::string data_path = config_node["data_path"].as<std::string>();
  if (data_path == "./") {
    data_path = WORK_SPACE_PATH;
  }

  if (!FileManager::CreateDirectory(data_path + "/slam_data")) {
    return false;
  }

  trajectory_path_ = data_path + "/slam_data/trajectory";
  if (!FileManager::InitDirectory(trajectory_path_, "Estimated Trajectory")) {
    return false;
  }

  return true;
}

bool SlidingWindow::InitKeyFrameSelection(const YAML::Node& config_node) {
  key_frame_config_.max_distance =
      config_node["key_frame"]["max_distance"].as<float>();
  key_frame_config_.max_interval =
      config_node["key_frame"]["max_interval"].as<float>();

  return true;
}

bool SlidingWindow::InitSlidingWindow(const YAML::Node& config_node) {
  // init sliding window:
  const int sliding_window_size = config_node["sliding_window_size"].as<int>();
  sliding_window_ptr_ = std::shared_ptr<CeresSlidingWindow>(
      new CeresSlidingWindow(sliding_window_size));

  // select measurements:
  measurement_config_.source.map_matching =
      config_node["measurements"]["map_matching"].as<bool>();
  measurement_config_.source.imu_pre_integration =
      config_node["measurements"]["imu_pre_integration"].as<bool>();

  // get measurement noises, pose:
  measurement_config_.noise.lidar_odometry.resize(6);
  measurement_config_.noise.map_matching.resize(6);
  for (int i = 0; i < 6; ++i) {
    measurement_config_.noise.lidar_odometry(i) =
        config_node["lidar_odometry"]["noise"][i].as<double>();
    measurement_config_.noise.map_matching(i) =
        config_node["map_matching"]["noise"][i].as<double>();
  }

  // get measurement noises, position:
  measurement_config_.noise.gnss_position.resize(3);
  for (int i = 0; i < 3; i++) {
    measurement_config_.noise.gnss_position(i) =
        config_node["gnss_position"]["noise"][i].as<double>();
  }

  return true;
}

bool SlidingWindow::InitIMUPreIntegrator(const YAML::Node& config_node) {
  imu_pre_integrator_ptr_ = nullptr;

  if (measurement_config_.source.imu_pre_integration) {
    imu_pre_integrator_ptr_ = std::shared_ptr<IMUPreIntegrator>(
        new IMUPreIntegrator(config_node["imu_pre_integration"]));
  }

  return true;
}

bool SlidingWindow::UpdateIMUPreIntegration(const IMUData& imu_data) {
  if (!measurement_config_.source.imu_pre_integration ||
      nullptr == imu_pre_integrator_ptr_) {
    return false;
  }

  if (!imu_pre_integrator_ptr_->IsInited() ||
      imu_pre_integrator_ptr_->Update(imu_data)) {
    return true;
  }

  return false;
}

bool SlidingWindow::Update(const PoseData& laser_odom,
                           const PoseData& map_matching_odom,
                           const IMUData& imu_data,
                           const PoseData& gnss_pose) {
  ResetParam();

  if (MaybeNewKeyFrame(laser_odom, map_matching_odom, imu_data, gnss_pose)) {
    Update();
    MaybeOptimized();
  }

  return true;
}

void SlidingWindow::GetLatestOptimizedOdometry(KeyFrame& key_frame) {
  sliding_window_ptr_->GetLatestOptimizedKeyFrame(key_frame);
}

void SlidingWindow::GetOptimizedKeyFrames(
    std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>&
        key_frames_deque) {
  key_frames_deque.clear();

  // load optimized key frames:
  sliding_window_ptr_->GetOptimizedKeyFrames(key_frames_.optimized);

  key_frames_deque.insert(key_frames_deque.end(),
                          key_frames_.optimized.begin(),
                          key_frames_.optimized.end());
}

bool SlidingWindow::SavePose(const Eigen::Matrix4f& pose, std::ofstream& ofs) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << pose(i, j);

      if (i == 2 && j == 3) {
        ofs << std::endl;
      } else {
        ofs << " ";
      }
    }
  }

  return true;
}

bool SlidingWindow::SaveOptimizedTrajectory() {
  static Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();

  if (sliding_window_ptr_->GetNumParamBlocks() == 0)
    return false;

  // create output files:
  std::ofstream laser_odom_ofs, optimized_ofs, ground_truth_ofs;
  if (!FileManager::CreateFile(trajectory_path_ + "/laser_odom.txt",
                               laser_odom_ofs) ||
      !FileManager::CreateFile(trajectory_path_ + "/optimized.txt",
                               optimized_ofs) ||
      !FileManager::CreateFile(trajectory_path_ + "/ground_truth.txt",
                               ground_truth_ofs)) {
    return false;
  }

  // load optimized key frames:
  sliding_window_ptr_->GetOptimizedKeyFrames(key_frames_.optimized);

  // write
  //
  //     a. lidar odometry estimation
  //     b. sliding window optimized odometry estimation
  //     c. IMU/GNSS position reference
  //
  // as trajectories for evo evaluation:
  for (size_t i = 0; i < key_frames_.optimized.size(); ++i) {
    // a. lidar odometry:
    current_pose = key_frames_.lidar.at(i).pose_;
    // current_pose(2, 3) = 0.0f;
    SavePose(current_pose, laser_odom_ofs);
    // b. sliding window optimized odometry:
    current_pose = key_frames_.optimized.at(i).pose_;
    // current_pose(2, 3) = 0.0f;
    SavePose(current_pose, optimized_ofs);
    // c. IMU/GNSS position reference as ground truth:
    current_pose = key_frames_.reference.at(i).pose_;
    // current_pose(2, 3) = 0.0f;
    SavePose(current_pose, ground_truth_ofs);
  }

  return true;
}

bool SlidingWindow::MaybeNewKeyFrame(const PoseData& laser_odom,
                                     const PoseData& map_matching_odom,
                                     const IMUData& imu_data,
                                     const PoseData& gnss_odom) {
  static KeyFrame last_key_frame;

  //
  // key frame selection for sliding window:
  //
  if (key_frames_.lidar.empty()) {
    // init IMU pre-integrator:
    // 如果系统没初始化则初始化预积分
    if (imu_pre_integrator_ptr_) {
      imu_pre_integrator_ptr_->Init(imu_data);
    }

    has_new_key_frame_ = true;
  } else if (
      // spatial:
      (laser_odom.pose_.block<3, 1>(0, 3) -
       last_key_frame.pose_.block<3, 1>(0, 3))
              .lpNorm<1>() > key_frame_config_.max_distance ||
      // temporal:
      (laser_odom.time_ - last_key_frame.time_) >
          key_frame_config_.max_interval) {
    // finish current IMU pre-integration:
    // 如果是关键帧 重置预积分
    if (imu_pre_integrator_ptr_) {
      imu_pre_integrator_ptr_->Reset(imu_data, imu_pre_integration_);
    }

    has_new_key_frame_ = true;
  } else {
    has_new_key_frame_ = false;
  }

  // if so:
  if (has_new_key_frame_) {
    // create key frame for lidar odometry, relative pose measurement:
    current_key_frame_.time_ = laser_odom.time_;
    current_key_frame_.index_ = key_frames_.lidar.size();
    current_key_frame_.pose_ = laser_odom.pose_;
    current_key_frame_.vel_.v = gnss_odom.vel_.v;
    current_key_frame_.vel_.w = gnss_odom.vel_.w;

    current_map_matching_pose_ = map_matching_odom;

    // create key frame for GNSS measurement, full LIO state:
    current_key_gnss_.time_ = current_key_frame_.time_;
    current_key_gnss_.index_ = current_key_frame_.index_;
    current_key_gnss_.pose_ = gnss_odom.pose_;
    current_key_gnss_.vel_.v = gnss_odom.vel_.v;
    current_key_gnss_.vel_.w = gnss_odom.vel_.w;

    // add to cache for later evo evaluation:
    key_frames_.lidar.push_back(current_key_frame_);
    key_frames_.reference.push_back(current_key_gnss_);

    // save for next key frame selection:
    last_key_frame = current_key_frame_;
  }

  return has_new_key_frame_;
}

bool SlidingWindow::Update() {
  static KeyFrame last_key_frame_ = current_key_frame_;

  //
  // add node for new key frame pose:
  //
  // fix the pose of the first key frame for lidar only mapping:
  if (sliding_window_ptr_->GetNumParamBlocks() == 0) {
    sliding_window_ptr_->AddPRVAGParam(current_key_frame_, true);
  } else {
    sliding_window_ptr_->AddPRVAGParam(current_key_frame_, false);
  }

  // get num. of vertices:
  const int N = sliding_window_ptr_->GetNumParamBlocks();
  // get param block ID, current:
  const int param_index_j = N - 1;

  //
  // add unary constraints:
  //
  //
  // a. map matching / GNSS position:
  //
  if (N > 0 && measurement_config_.source.map_matching) {
    Eigen::Matrix4d prior_pose = current_key_frame_.pose_.cast<double>();
    LOG(INFO) << "add matching pose: "
              << prior_pose.block<3, 1>(0, 3).transpose();
    sliding_window_ptr_->AddPRVAGMapMatchingPoseFactor(
        param_index_j, prior_pose, measurement_config_.noise.map_matching);

    // TODO: add GNSS position constraint
  }

  //
  // add binary constraints:
  //
  if (N > 1) {
    // get param block ID, previous:
    const int param_index_i = N - 2;

    // TODO: add lidar odometry constraints
    //
    // a. lidar frontend:
    //
    // get relative pose measurement:
    // Eigen::Matrix4d relative_pose =
    //     (last_key_frame_.pose_.inverse() * current_key_frame_.pose_)
    //         .cast<double>();
    // // TODO: add constraint, lidar frontend / loop closure detection:
    // sliding_window_ptr_->AddPRVAGRelativePoseFactor(
    //     param_index_i,
    //     param_index_j,
    //     relative_pose,
    //     measurement_config_.noise.lidar_odometry);

    //
    // b. IMU pre-integration:
    //
    if (measurement_config_.source.imu_pre_integration) {
      // TODO: add constraint, IMU pre-integraion:
      sliding_window_ptr_->AddPRVAGIMUPreIntegrationFactor(
          param_index_i, param_index_j, imu_pre_integration_);
    }
  }

  // move forward:
  last_key_frame_ = current_key_frame_;

  return true;
}

bool SlidingWindow::MaybeOptimized() {
  if (sliding_window_ptr_->Optimize()) {
    has_new_optimized_ = true;
    return true;
  }

  return false;
}

}
}
