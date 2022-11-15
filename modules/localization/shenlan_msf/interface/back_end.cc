#include "modules/localization/shenlan_msf/interface/back_end.h"

namespace apollo {
namespace localization {

BackEnd::BackEnd(const std::string& config_file_path) {
  InitWithConfig(config_file_path);
}

bool BackEnd::InitWithConfig(const std::string& config_file_path) {
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::cout << "-----------------Init LIO Backend-------------------"
            << std::endl;

  InitDataPath(config_node);
  InitParam(config_node);
  InitGraphOptimizer(config_node);
  InitIMUPreIntegrator(config_node);
  InitOdoPreIntegrator(config_node);
  std::cout << "Init BackEnd Config Finished!\n";

  return true;
}

bool BackEnd::InitDataPath(const YAML::Node& config_node) {
  std::string data_path = config_node["data_path"].as<std::string>();
  if (data_path == "./") {
    data_path = WORK_SPACE_PATH;
  }

  if (!FileManager::CreateDirectory(data_path + "/slam_data")) return false;

  key_frames_path_ = data_path + "/slam_data/key_frames";
  scan_context_path_ = data_path + "/slam_data/scan_context";
  trajectory_path_ = data_path + "/slam_data/trajectory";

  if (!FileManager::InitDirectory(key_frames_path_, "Point Cloud Key Frames"))
    return false;
  if (!FileManager::InitDirectory(scan_context_path_,
                                  "Scan Context Index & Data"))
    return false;
  if (!FileManager::InitDirectory(trajectory_path_, "Estimated Trajectory"))
    return false;

  return true;
}

bool BackEnd::InitParam(const YAML::Node& config_node) {
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  is_kitti_ = config_node["is_kitti"].as<bool>();

  std::cout << "\tKey frame distance: " << key_frame_distance_ << std::endl
            << "\tis_kitti: " << is_kitti_ << std::endl
            << std::endl;

  return true;
}

bool BackEnd::InitGraphOptimizer(const YAML::Node& config_node) {
  const std::string graph_optimizer_type =
      config_node["graph_optimizer_type"].as<std::string>();

  if (graph_optimizer_type == "g2o") {
    graph_optimizer_ptr_ =
        std::shared_ptr<G2oGraphOptimizer>(new G2oGraphOptimizer("lm_var"));
  } else {
    LOG(ERROR) << "Optimizer " << graph_optimizer_type << " NOT FOUND!";
    return false;
  }
  std::cout << "\tOptimizer:" << graph_optimizer_type << std::endl << std::endl;

  graph_optimizer_config_.use_gnss = config_node["use_gnss"].as<bool>();
  graph_optimizer_config_.use_loop_close =
      config_node["use_loop_close"].as<bool>();
  graph_optimizer_config_.use_imu_pre_integration =
      config_node["use_imu_pre_integration"].as<bool>();
  graph_optimizer_config_.use_odo_pre_integration =
      config_node["use_odo_pre_integration"].as<bool>();

  graph_optimizer_config_.optimization_step_size.key_frame =
      (config_node["optimization_step_size"]["key_frame"].as<int>());
  graph_optimizer_config_.optimization_step_size.loop_closure =
      (config_node["optimization_step_size"]["loop_closure"].as<int>());

  // x-y-z & yaw-roll-pitch
  for (int i = 0; i < 6; ++i) {
    graph_optimizer_config_.odom_edge_noise(i) =
        config_node[graph_optimizer_type + "_param"]["odom_edge_noise"][i]
            .as<double>();
    graph_optimizer_config_.odom_edge_degeneracy_noise(i) =
        config_node[graph_optimizer_type + "_param"]
                   ["odom_edge_degeneracy_noise"][i]
                       .as<double>();
    graph_optimizer_config_.close_loop_noise(i) =
        config_node[graph_optimizer_type + "_param"]["close_loop_noise"][i]
            .as<double>();
  }

  // x-y-z:
  for (int i = 0; i < 3; i++) {
    graph_optimizer_config_.gnss_noise(i) =
        config_node[graph_optimizer_type + "_param"]["gnss_noise"][i]
            .as<double>();
  }

  // vbx-vby-vbz
  for (int i = 0; i < 3; i++) {
    graph_optimizer_config_.body_vel_noise(i) =
        config_node[graph_optimizer_type + "_param"]["body_vel_noise"][i]
            .as<double>();
  }

  return true;
}

bool BackEnd::InitIMUPreIntegrator(const YAML::Node& config_node) {
  if (graph_optimizer_config_.use_imu_pre_integration) {
    imu_pre_integrator_ptr_ = std::shared_ptr<IMUPreIntegrator>(
        new IMUPreIntegrator(config_node["imu_pre_integration"]));
  }
  return true;
}

bool BackEnd::InitOdoPreIntegrator(const YAML::Node& config_node) {
  odo_pre_integrator_ptr_ = nullptr;

  if (graph_optimizer_config_.use_odo_pre_integration) {
    odo_pre_integrator_ptr_ = std::shared_ptr<OdoPreIntegrator>(
        new OdoPreIntegrator(config_node["odo_pre_integration"]));
  }

  return true;
}

bool BackEnd::InsertLoopPose(const LoopPose& loop_pose) {
  if (!graph_optimizer_config_.use_loop_close) return false;

  // get vertex IDs:
  const int vertex_index_i = loop_pose.index0_;
  const int vertex_index_j = loop_pose.index1_;
  // get relative pose measurement:
  Eigen::Matrix4d relative_pose = loop_pose.pose_.cast<double>();
  // add constraint, lidar frontend / loop closure detection:
  graph_optimizer_ptr_->AddPRVAGRelativePoseEdge(
      vertex_index_i, vertex_index_j, relative_pose,
      graph_optimizer_config_.close_loop_noise);

  // update loop closure count:
  ++counter_.loop_closure;

  std::cout << "Add loop closure: " << loop_pose.index0_ << ","
            << loop_pose.index1_ << std::endl;

  return true;
}

bool BackEnd::UpdateIMUPreIntegration(const IMUData& imu_data) {
  if (!graph_optimizer_config_.use_imu_pre_integration ||
      nullptr == imu_pre_integrator_ptr_)
    return false;

  if (!imu_pre_integrator_ptr_->IsInited() ||
      imu_pre_integrator_ptr_->Update(imu_data)) {
    return true;
  }

  return false;
}

bool BackEnd::UpdateOdoPreIntegration(const VelocityData& velocity_data) {
  if (!graph_optimizer_config_.use_odo_pre_integration ||
      nullptr == odo_pre_integrator_ptr_)
    return false;

  if (!odo_pre_integrator_ptr_->IsInited() ||
      odo_pre_integrator_ptr_->Update(velocity_data)) {
    return true;
  }

  return false;
}

bool BackEnd::Update(const CloudData& cloud_data, const PoseData& laser_odom,
                     const PoseData& gnss_pose, const IMUData& imu_data) {
  ResetParam();

  if (MaybeNewKeyFrame(cloud_data, laser_odom, gnss_pose, imu_data)) {
    AddNodeAndEdge(gnss_pose);
    MaybeOptimized();
  }
  return true;
}

void BackEnd::ResetParam() {
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
}

bool BackEnd::MaybeNewKeyFrame(const CloudData& cloud_data,
                               const PoseData& laser_odom,
                               const PoseData& gnss_odom,
                               const IMUData& imu_data) {
  static bool start = false;
  static int count = 0;
  static VelocityData velocity_data;
  if (!start) {
    last_laser_pose = laser_odom;
    last_gnss_pose = gnss_odom;
    start = true;
    std::cout << "back_end_ptr start!\n";
  }

  if (key_frames_deque_.size() == 0) {
    // init IMU pre-integrator:
    if (imu_pre_integrator_ptr_) {
      imu_pre_integrator_ptr_->Init(imu_data);
    }

    // init odometer pre-integrator:
    if (odo_pre_integrator_ptr_) {
      gnss_odom.GetVelocityData(velocity_data);
      odo_pre_integrator_ptr_->Init(velocity_data);
    }

    last_laser_pose = laser_odom;
    last_gnss_pose = gnss_odom;

    has_new_key_frame_ = true;
  }

  // whether the current scan is far away enough from last key frame:
  if ((laser_odom.pose_.block<3, 1>(0, 3) -
       last_laser_pose.pose_.block<3, 1>(0, 3))
          .norm() > key_frame_distance_) {
    if (imu_pre_integrator_ptr_) {
      imu_pre_integrator_ptr_->Reset(imu_data, imu_pre_integration_);
    }

    if (odo_pre_integrator_ptr_) {
      gnss_odom.GetVelocityData(velocity_data);
      odo_pre_integrator_ptr_->Reset(velocity_data, odo_pre_integration_);
    }

    //
    // for IMU pre-integration debugging ONLY:
    // this is critical to IMU pre-integration verification
    //
    if (0 == (++count) % 10) {
      // display IMU pre-integration:
      ShowIMUPreIntegrationResidual(last_gnss_pose, gnss_odom,
                                    imu_pre_integration_);
      // reset counter:
      count = 0;
    }

    last_laser_pose = laser_odom;
    last_gnss_pose = gnss_odom;

    has_new_key_frame_ = true;
  }

  // if so:
  if (has_new_key_frame_) {
    // a. first write new key scan to disk:
    std::string file_path = key_frames_path_ + "/key_frame_" +
                            std::to_string(key_frames_deque_.size()) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr_);

    current_key_scan_.time_ = cloud_data.time_;
    current_key_scan_.cloud_ptr_.reset(
        new CloudData::CloudType(*cloud_data.cloud_ptr_));

    current_key_gnss_.time_ = current_key_frame_.time_ = laser_odom.time_;

    current_key_gnss_.index_ = current_key_frame_.index_ =
        key_frames_deque_.size();

    // b. create key frame index for lidar scan, relative pose measurement:
    current_key_frame_.pose_ = laser_odom.pose_;
    // current_key_frame_.pose_ = Eigen::Matrix4f::Zero();
    current_key_frame_.solution_status_ = (int)(laser_odom.cov_.at(0));

    // c. create key frame index for GNSS measurement, full LIO state:
    current_key_gnss_.pose_ = gnss_odom.pose_;

    if (is_kitti_) {
      current_key_gnss_.vel_.v =
          gnss_odom.pose_.block<3, 3>(0, 0) * gnss_odom.vel_.v;
    } else {
      current_key_gnss_.vel_.v = gnss_odom.vel_.v;
    }

    current_key_gnss_.vel_.w = gnss_odom.vel_.w;
    current_key_gnss_.solution_status_ = (int)(gnss_odom.cov_.at(1));

    // add to cache for later evo evaluation:
    key_frames_deque_.push_back(current_key_frame_);
    key_gnss_deque_.push_back(current_key_gnss_);
  }

  return has_new_key_frame_;
}

bool BackEnd::AddNodeAndEdge(const PoseData& gnss_data) {
  static KeyFrame last_key_frame_ = current_key_frame_;

  //
  // add node for new key frame pose:
  //
  // fix the pose of the first key frame for lidar only mapping:
  if (!graph_optimizer_config_.use_gnss &&
      graph_optimizer_ptr_->GetNodeNum() == 0) {
    graph_optimizer_ptr_->AddPRVAGNode(current_key_frame_, true);
  } else {
    // graph_optimizer_ptr_->AddPRVAGNode(current_key_gnss_, false);
    graph_optimizer_ptr_->AddPRVAGNode(current_key_frame_, false);
  }

  //
  // add constraints:
  //
  // get num. of vertices:
  const int N = graph_optimizer_ptr_->GetNodeNum();
  // get vertex IDs:
  const int vertex_index_i = N - 2;
  const int vertex_index_j = N - 1;
  // a. lidar frontend / loop closure detection:
  if (N > 1) {
    // get relative pose measurement:
    Eigen::Matrix4d relative_pose =
        (last_key_frame_.pose_.inverse() * current_key_frame_.pose_)
            .cast<double>();
    // add constraint, lidar frontend / loop closure detection:
    if (current_key_frame_.solution_status_ == 1) {
      graph_optimizer_ptr_->AddPRVAGRelativePoseEdge(
          vertex_index_i, vertex_index_j, relative_pose,
          graph_optimizer_config_.odom_edge_degeneracy_noise);
      LOG(INFO) << "add degeneracy lidar odometry.";
    } else {
      graph_optimizer_ptr_->AddPRVAGRelativePoseEdge(
          vertex_index_i, vertex_index_j, relative_pose,
          graph_optimizer_config_.odom_edge_noise);
    }
  }

  // b. GNSS position:
  // 这个if是用来测试beijing数据集, 因为它没gnss标志位
  // if (graph_optimizer_config_.use_gnss &&
  //     (current_key_gnss_.time_ < 1665824072.971791)) {
  //   // 这个if才是正常的if
  if (graph_optimizer_config_.use_gnss &&
      (current_key_gnss_.solution_status_ == 56 ||
       current_key_gnss_.solution_status_ == 0)) {
    // get prior position measurement:
    Eigen::Vector3d pos =
        current_key_gnss_.pose_.block<3, 1>(0, 3).cast<double>();
    // add constraint, GNSS position:
    graph_optimizer_ptr_->AddPRVAGPriorPosEdge(
        vertex_index_j, pos, graph_optimizer_config_.gnss_noise);
  }

  // c. IMU pre-integration:
  if (N > 1 && graph_optimizer_config_.use_imu_pre_integration &&
      (imu_pre_integration_.T < 5.0)) {
    // add constraint, IMU pre-integraion:
    graph_optimizer_ptr_->AddPRVAGIMUPreIntegrationEdge(
        vertex_index_i, vertex_index_j, imu_pre_integration_);
  }

  // d. Odo pre-integration:
  if (graph_optimizer_config_.use_odo_pre_integration) {
    graph_optimizer_ptr_->AddPRVAGPriorBodyVelEdge(
        vertex_index_j, current_key_gnss_.vel_.v.x(),
        graph_optimizer_config_.body_vel_noise);
  }

  // move forward:
  last_key_frame_ = current_key_frame_;
  ++counter_.key_frame;

  return true;
}

bool BackEnd::MaybeOptimized() {
  bool need_optimize = false;

  if (counter_.HasEnoughKeyFrames(
          graph_optimizer_config_.optimization_step_size.key_frame) ||
      counter_.HasEnoughLoopClosures(
          graph_optimizer_config_.optimization_step_size.loop_closure)) {
    need_optimize = true;
    std::cout << "-----------------\n"
              << "need_optimize = true\n"
              << "-----------------\n";
  }

  if (need_optimize && graph_optimizer_ptr_->Optimize()) {
    has_new_optimized_ = true;
    return true;
  }

  return false;
}

bool BackEnd::SavePose(const Eigen::Matrix4f& pose, std::ofstream& ofs) {
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

bool BackEnd::ForceOptimize() {
  if (graph_optimizer_ptr_->Optimize()) has_new_optimized_ = true;

  return has_new_optimized_;
}

bool BackEnd::SaveOptimizedPose() {
  static Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();

  if (graph_optimizer_ptr_->GetNodeNum() == 0) return false;

  if (!FileManager::CreateFile(trajectory_path_ + "/ground_truth.txt",
                               ground_truth_ofs_) ||
      !FileManager::CreateFile(trajectory_path_ + "/laser_odom.txt",
                               laser_odom_ofs_) ||
      !FileManager::CreateFile(trajectory_path_ + "/optimized.txt",
                               optimized_pose_ofs_))
    return false;

  graph_optimizer_ptr_->GetOptimizedKeyFrame(optimized_key_frames_);

  // write GNSS/IMU pose and lidar odometry estimation as trajectory for evo
  // evaluation:
  for (size_t i = 0; i < optimized_key_frames_.size(); ++i) {
    // a. ground truth, IMU/GNSS:
    current_pose = key_gnss_deque_.at(i).pose_;
    current_pose(2, 3) = 0.0f;
    SavePose(current_pose, ground_truth_ofs_);
    // b. lidar odometry:
    current_pose = key_frames_deque_.at(i).pose_;
    current_pose(2, 3) = 0.0f;
    SavePose(current_pose, laser_odom_ofs_);
    // c. optimized odometry:
    current_pose = optimized_key_frames_.at(i).pose_;
    current_pose(2, 3) = 0.0f;
    SavePose(current_pose, optimized_pose_ofs_);
  }

  return true;
}

void BackEnd::GetOptimizedKeyFrames(
    std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>&
        key_frames_deque) {
  key_frames_deque.clear();
  // TODO: check if necessary
  graph_optimizer_ptr_->GetOptimizedKeyFrame(optimized_key_frames_);
  key_frames_deque.insert(key_frames_deque.end(), optimized_key_frames_.begin(),
                          optimized_key_frames_.end());
}

void BackEnd::GetLatestKeyScan(CloudData& key_scan) {
  key_scan.time_ = current_key_scan_.time_;
  key_scan.cloud_ptr_.reset(
      new CloudData::CloudType(*current_key_scan_.cloud_ptr_));
}

void BackEnd::ShowIMUPreIntegrationResidual(
    const PoseData& last_gnss_pose, const PoseData& curr_gnss_pose,
    const IMUPreIntegrator::IMUPreIntegration& imu_pre_integration) {
  const double& T = imu_pre_integration.T;
  const Eigen::Vector3d& g = imu_pre_integration.g;

  const Eigen::Vector3d r_p =
      last_gnss_pose.pose_.block<3, 3>(0, 0).transpose().cast<double>() *
          (curr_gnss_pose.pose_.block<3, 1>(0, 3).cast<double>() -
           last_gnss_pose.pose_.block<3, 1>(0, 3).cast<double>() -
           (last_gnss_pose.pose_.block<3, 3>(0, 0).cast<double>() *
                last_gnss_pose.vel_.v.cast<double>() -
            0.50 * g * T) *
               T) -
      imu_pre_integration.alpha_ij;

  const Sophus::SO3d prev_theta(Eigen::Quaterniond(
      last_gnss_pose.pose_.block<3, 3>(0, 0).cast<double>()));
  const Sophus::SO3d curr_theta(Eigen::Quaterniond(
      curr_gnss_pose.pose_.block<3, 3>(0, 0).cast<double>()));
  const Eigen::Vector3d r_q = (imu_pre_integration.theta_ij.inverse() *
                               prev_theta.inverse() * curr_theta)
                                  .log();

  const Eigen::Vector3d r_v =
      last_gnss_pose.pose_.block<3, 3>(0, 0).transpose().cast<double>() *
          (curr_gnss_pose.pose_.block<3, 3>(0, 0).cast<double>() *
               curr_gnss_pose.vel_.v.cast<double>() -
           last_gnss_pose.pose_.block<3, 3>(0, 0).cast<double>() *
               last_gnss_pose.vel_.v.cast<double>() +
           g * T) -
      imu_pre_integration.beta_ij;

  LOG(INFO) << "IMU Pre-Integration Measurement: " << std::endl
            << "\tT: " << T << " --- "
            << curr_gnss_pose.time_ - last_gnss_pose.time_ << std::endl
            << "\talpha:" << std::endl
            << "\t\t" << r_p.x() << ", " << r_p.y() << ", " << r_p.z()
            << std::endl
            << "\ttheta:" << std::endl
            << "\t\t" << r_q.x() << ", " << r_q.y() << ", " << r_q.z()
            << std::endl
            << "\tbeta:" << std::endl
            << "\t\t" << r_v.x() << ", " << r_v.y() << ", " << r_v.z()
            << std::endl
            << "\tbias_accel:" << imu_pre_integration.b_a_i.x() << ", "
            << imu_pre_integration.b_a_i.y() << ", "
            << imu_pre_integration.b_a_i.z() << std::endl
            << "\tbias_gyro:" << imu_pre_integration.b_g_i.x() << ", "
            << imu_pre_integration.b_g_i.y() << ", "
            << imu_pre_integration.b_g_i.z() << std::endl
            << "\tcovariance:" << std::endl
            << "\t\talpha: " << imu_pre_integration.P(0, 0) << ", "
            << imu_pre_integration.P(1, 1) << ", "
            << imu_pre_integration.P(2, 3) << std::endl
            << "\t\ttheta: " << imu_pre_integration.P(3, 3) << ", "
            << imu_pre_integration.P(4, 4) << ", "
            << imu_pre_integration.P(5, 5) << std::endl
            << "\t\tbeta: " << imu_pre_integration.P(6, 6) << ", "
            << imu_pre_integration.P(7, 7) << ", "
            << imu_pre_integration.P(8, 8) << std::endl
            << "\t\tbias_accel: " << imu_pre_integration.P(9, 9) << ", "
            << imu_pre_integration.P(10, 10) << ", "
            << imu_pre_integration.P(11, 11) << std::endl
            << "\t\tbias_gyro: " << imu_pre_integration.P(12, 12) << ", "
            << imu_pre_integration.P(13, 13) << ", "
            << imu_pre_integration.P(14, 14) << std::endl;
}

}  // namespace localization
}  // namespace apollo
