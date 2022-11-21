#pragma once
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/gnss_data.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/pose_data.h"
#include "lidar_localization/sensor_data/velocity_data.h"
#include "yaml-cpp/yaml.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/message/raw_message.h"
#include "cyber/time/clock.h"
#include "modules/common/util/time_util.h"

using namespace lidar_localization;

using apollo::common::util::TimeUtil;
using apollo::cyber::Clock;

namespace apollo {
namespace localization {

class MsgTransfer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MsgTransfer(){};
  ~MsgTransfer(){};

  void LidarMsgTransfer(const std::shared_ptr<drivers::PointCloud> &msg,
                        CloudData *lidar_frame);

  void IMUMsgTransfer(const std::shared_ptr<drivers::gnss::Imu> &imu_msg,
                      IMUData &imu);

  void OdometryMsgTransfer(const std::shared_ptr<localization::Gps> &odom_msg,
                           PoseData &odom_frame);

  void PoseMsgTransfer(const std::shared_ptr<LocalizationEstimate> &pose_msg, 
                      PoseData &pose_frame);

  void ChassisMsgTransfer(const std::shared_ptr<canbus::Chassis> &chassis_msg,
                          VelocityData &chassis_frame);

  bool LoadLidarExtrinsic(const std::string &file_path,
                          Eigen::Affine3d *lidar_extrinsic);

  void ComposeLocalizationResult(double time_stamp, const Eigen::Matrix4f &pose,
                                 LocalizationEstimate *localization,
                                 double status = 0);

  void FillLocalizationMsgHeader(LocalizationEstimate *localization);

 public:
  static constexpr double DEG_TO_RAD = 0.017453292519943;
  static constexpr double wheel_speed_coff = 1.0;
  static constexpr double kEpsilon = 1e-7;
  static constexpr double wheel_base = 1.465;
};

}  // namespace localization
}  // namespace apollo
