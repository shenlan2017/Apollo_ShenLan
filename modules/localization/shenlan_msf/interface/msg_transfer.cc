#include "msg_transfer.h"

namespace apollo {
namespace localization {

void MsgTransfer::LidarMsgTransfer(
    const std::shared_ptr<drivers::PointCloud> &msg, CloudData *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  if (msg->height() > 1 && msg->width() > 1) {
    for (unsigned int i = 0; i < msg->height(); ++i) {
      for (unsigned int j = 0; j < msg->width(); ++j) {
        CloudData::PointType pt3d;
        pt3d.x = msg->point(i * msg->width() + j).x();
        pt3d.y = msg->point(i * msg->width() + j).y();
        pt3d.z = msg->point(i * msg->width() + j).z();
        if (std::isnan(pt3d.x)) {
          continue;
        }
        //   unsigned char intensity = static_cast<unsigned char>(
        //       msg->point(i * msg->width() + j).intensity());
        lidar_frame->cloud_ptr_->push_back(pt3d);
      }
    }
  } else {
    AINFO << "Receiving un-organized-point-cloud, width " << msg->width()
          << " height " << msg->height() << "size " << msg->point_size();
    for (int i = 0; i < msg->point_size(); ++i) {
      CloudData::PointType pt3d;
      pt3d.x = msg->point(i).x();
      pt3d.y = msg->point(i).y();
      pt3d.z = msg->point(i).z();
      if (std::isnan(pt3d.x)) {
        continue;
      }
      //   unsigned char intensity = static_cast<unsigned
      //   char>(msg->point(i).intensity());
      lidar_frame->cloud_ptr_->push_back(pt3d);
    }
  }
  lidar_frame->time_ = cyber::Time(msg->measurement_time()).ToSecond();
}

void MsgTransfer::IMUMsgTransfer(
    const std::shared_ptr<drivers::gnss::Imu> &imu_msg, IMUData &imu) {
  double measurement_time = TimeUtil::Gps2Unix(imu_msg->measurement_time());
  imu.time_ = measurement_time;

  // 坐标系是右前上（RFU）还是前左上（FLU）
  if (1) {
    imu.linear_acceleration_.x = imu_msg->linear_acceleration().x();
    imu.linear_acceleration_.y = imu_msg->linear_acceleration().y();
    imu.linear_acceleration_.z = imu_msg->linear_acceleration().z();
    imu.angular_velocity_.x = imu_msg->angular_velocity().x();
    imu.angular_velocity_.y = imu_msg->angular_velocity().y();
    imu.angular_velocity_.z = imu_msg->angular_velocity().z();
  } else {
    imu.linear_acceleration_.x = -imu_msg->linear_acceleration().y();
    imu.linear_acceleration_.y = imu_msg->linear_acceleration().x();
    imu.linear_acceleration_.z = imu_msg->linear_acceleration().z();
    imu.angular_velocity_.x = -imu_msg->angular_velocity().y();
    imu.angular_velocity_.y = imu_msg->angular_velocity().x();
    imu.angular_velocity_.z = imu_msg->angular_velocity().z();
  }
}

void MsgTransfer::OdometryMsgTransfer(
    const std::shared_ptr<localization::Gps> &odom_msg, PoseData &odom_frame) {
  odom_frame.time_ = odom_msg->header().timestamp_sec();
  // set the position:
  odom_frame.pose_(0, 3) = odom_msg->localization().position().x();
  odom_frame.pose_(1, 3) = odom_msg->localization().position().y() - 4457770.0;
  odom_frame.pose_(2, 3) = odom_msg->localization().position().z();

  Eigen::Quaternionf q;
  q.x() = odom_msg->localization().orientation().qx();
  q.y() = odom_msg->localization().orientation().qy();
  q.z() = odom_msg->localization().orientation().qz();
  q.w() = odom_msg->localization().orientation().qw();
  odom_frame.pose_.block<3, 3>(0, 0) = q.matrix();

  // set the linear velocity:
  odom_frame.vel_.v.x() = odom_msg->localization().linear_velocity().x();
  odom_frame.vel_.v.y() = odom_msg->localization().linear_velocity().y();
  odom_frame.vel_.v.z() = odom_msg->localization().linear_velocity().z();
}

void MsgTransfer::ChassisMsgTransfer(
    const std::shared_ptr<canbus::Chassis> &chassis_msg,
    VelocityData &chassis_frame) {
  chassis_frame.time_ = chassis_msg->header().timestamp_sec();

  chassis_frame.linear_velocity_.x = chassis_msg->speed_mps();
  chassis_frame.linear_velocity_.y = 0.0f;
  chassis_frame.linear_velocity_.z = 0.0f;

  double wheel_spd_rl = chassis_msg->wheel_speed().wheel_spd_rl();
  double wheel_spd_rr = chassis_msg->wheel_speed().wheel_spd_rr();
  double angular_speed = 0.0f;

  if (std::fabs(wheel_spd_rr) > kEpsilon) {
    double kappa = wheel_spd_rl / wheel_spd_rr;
    angular_speed = -2.0 * (kappa - 1) / (kappa + 1) *
                    chassis_frame.linear_velocity_.x / wheel_base;
  }

  chassis_frame.angular_velocity_.x = 0.0f;
  chassis_frame.angular_velocity_.y = 0.0f;
  chassis_frame.angular_velocity_.z = wheel_speed_coff * angular_speed;
}

bool MsgTransfer::LoadLidarExtrinsic(const std::string &file_path,
                                           Eigen::Affine3d *lidar_extrinsic) {
  CHECK_NOTNULL(lidar_extrinsic);
  YAML::Node config = YAML::LoadFile(file_path);

  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      lidar_extrinsic->translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      lidar_extrinsic->translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      lidar_extrinsic->translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        lidar_extrinsic->linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        return true;
      }
    }
  }

  return false;
}

void MsgTransfer::ComposeLocalizationResult(
    double time_stamp, const Eigen::Matrix4f &pose,
    LocalizationEstimate *localization, double status) {
  localization->Clear();
  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(time_stamp);
  auto *mutable_pose = localization->mutable_pose();
  Eigen::Vector3f t = pose.block<3, 1>(0, 3);
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  // mutable_pose->mutable_position()->set_x(t.x() + gnss_origin.x());
  // mutable_pose->mutable_position()->set_y(t.y() + gnss_origin.y());
  // mutable_pose->mutable_position()->set_z(t.z() + gnss_origin.z());
  mutable_pose->mutable_position()->set_x(t.x());
  mutable_pose->mutable_position()->set_y(t.y());
  mutable_pose->mutable_position()->set_z(t.z());
  mutable_pose->mutable_orientation()->set_qw(quat.w());
  mutable_pose->mutable_orientation()->set_qx(quat.x());
  mutable_pose->mutable_orientation()->set_qy(quat.y());
  mutable_pose->mutable_orientation()->set_qz(quat.z());
  // double heading =
  //     common::math::QuaternionToHeading(quat.w(), quat.x(), quat.y(),
  //     quat.z());
  // mutable_pose->set_heading(heading);

  // common::math::EulerAnglesZXYd euler(quat.w(), quat.x(), quat.y(),
  // quat.z()); mutable_pose->mutable_euler_angles()->set_x(euler.pitch());
  // mutable_pose->mutable_euler_angles()->set_y(euler.roll());
  // mutable_pose->mutable_euler_angles()->set_z(euler.yaw());
  if (int(status) == 1) {
    localization->mutable_msf_status()->set_local_lidar_quality(
        LocalLidarQuality::MSF_LOCAL_LIDAR_BAD);
  }else{
    localization->mutable_msf_status()->set_local_lidar_quality(
        LocalLidarQuality::MSF_LOCAL_LIDAR_GOOD);
  }

}

void MsgTransfer::FillLocalizationMsgHeader(
    LocalizationEstimate *localization) {
  auto *header = localization->mutable_header();
  double timestamp = apollo::cyber::Clock::NowInSeconds();
  header->set_module_name("shenlan_msf_localization");
  header->set_timestamp_sec(timestamp);
  // header->set_sequence_num(++localization_seq_num_);
}

}  // namespace localization
}  // namespace apollo
