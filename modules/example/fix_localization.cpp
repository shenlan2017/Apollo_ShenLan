#include <time.h>

#include <iomanip>

#include "modules/localization/proto/localization.pb.h"

#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"
#include "modules/transform/transform_broadcaster.h"

namespace apollo {
namespace localization {

class FixLocalization {
 private:
  std::shared_ptr<cyber::Node> talker_node_;
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> talker_ = nullptr;
  std::unique_ptr<apollo::transform::TransformBroadcaster> tf2_broadcaster_ = nullptr;
  LocalizationEstimate result_;
  std::string topic_ = "/apollo/localization/pose";
  long long seq_num_ = 0;
  double unix_timestamp = 0.0;
  double gps_timestamp = 0.0;
  std::string local_time_str = "";

 public:
  FixLocalization() {
    talker_node_ = std::unique_ptr<cyber::Node>(
        apollo::cyber::CreateNode("fix_localization"));
    tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(talker_node_));
  };
  ~FixLocalization(){};

  bool InitIO() {
    talker_ = talker_node_->CreateWriter<LocalizationEstimate>(topic_);
    return true;
  };

  void PublishMessages(const LocalizationEstimate& msg) {
    talker_->Write(msg);
  };

  void FillMsgHeader(LocalizationEstimate* msg_header) {
    auto* header = msg_header->mutable_header();
    unix_timestamp = apollo::cyber::Clock::NowInSeconds();
    header->set_module_name("localization");
    header->set_timestamp_sec(unix_timestamp);
    header->set_sequence_num(++seq_num_);
  };

  void PublishPoseBroadcastTF(const LocalizationEstimate& localization) {
    apollo::transform::TransformStamped tf2_msg;

    auto mutable_head = tf2_msg.mutable_header();
    mutable_head->set_timestamp_sec(localization.measurement_time());
    mutable_head->set_frame_id("world");
    tf2_msg.set_child_frame_id("localization");

    auto mutable_translation =
        tf2_msg.mutable_transform()->mutable_translation();
    mutable_translation->set_x(localization.pose().position().x());
    mutable_translation->set_y(localization.pose().position().y());
    mutable_translation->set_z(localization.pose().position().z());

    auto mutable_rotation = tf2_msg.mutable_transform()->mutable_rotation();
    mutable_rotation->set_qx(localization.pose().orientation().qx());
    mutable_rotation->set_qy(localization.pose().orientation().qy());
    mutable_rotation->set_qz(localization.pose().orientation().qz());
    mutable_rotation->set_qw(localization.pose().orientation().qw());

    tf2_broadcaster_->SendTransform(tf2_msg);
  };

  void RunExampleOnce() {
    result_.Clear();
    FillMsgHeader(&result_);

    result_.set_measurement_time(unix_timestamp);

    // combine gps and imu
    auto mutable_pose = result_.mutable_pose();
    mutable_pose->mutable_position()->set_x(443986.970412012);
    mutable_pose->mutable_position()->set_y(4436652.635347065);
    mutable_pose->mutable_position()->set_z(33.189556338);
    mutable_pose->mutable_orientation()->set_qw(0.707);
    mutable_pose->mutable_orientation()->set_qx(0.707);
    mutable_pose->mutable_orientation()->set_qy(0.0);
    mutable_pose->mutable_orientation()->set_qz(0.0);

    // AINFO << "unix_timestamp : " << std::setprecision(16) << unix_timestamp;
    // AINFO << "---------------------------------";
    PublishPoseBroadcastTF(result_);
    PublishMessages(result_);
  };
};

}  // namespace localization
}  // namespace apollo

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  apollo::localization::FixLocalization localization;
  if (!localization.InitIO()) {
    AERROR << "Init Interval false.";
    return false;
  }

  apollo::cyber::Rate rate(100.0);
  while (apollo::cyber::OK()) {
    localization.RunExampleOnce();
    rate.Sleep();
  }

  return 0;
}
