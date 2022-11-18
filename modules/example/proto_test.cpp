#include <time.h>

#include <iomanip>

#include "modules/example/proto/times_utils.pb.h"

#include "cyber/common/time_conversion.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/time/clock.h"
#include "cyber/time/rate.h"

namespace apollo{
namespace example{

class TimeConversionExample {
 private:
  std::unique_ptr<cyber::Node> talker_node_;
  std::shared_ptr<cyber::Writer<TimeConversion>> talker_ = nullptr;
  TimeConversion result_;
  std::string topic_ = "/apollo/time/conversion";
  long long seq_num_ = 0;
  double unix_timestamp = 0.0;
  double gps_timestamp = 0.0;
  std::string local_time_str = "";

 public:
  TimeConversionExample(){
    talker_node_ =
        std::unique_ptr<cyber::Node>(apollo::cyber::CreateNode("conversion"));
  };
  ~TimeConversionExample(){};

  bool InitIO() {
    talker_ = talker_node_->CreateWriter<TimeConversion>(topic_);
    return true;
  };

  void PublishMessages(const TimeConversion& msg) {
    talker_->Write(msg);
  };

  void FillMsgHeader(TimeConversion* msg_header) {
    auto* header = msg_header->mutable_header();
    unix_timestamp = apollo::cyber::Clock::NowInSeconds();
    header->set_module_name("TimeConversionExample");
    header->set_timestamp_sec(unix_timestamp);
    header->set_sequence_num(++seq_num_);
  }

  std::string ToLocalTime(double& unix_timestamp) {
    auto timestamp = time_t((int)unix_timestamp);
    struct tm* time = localtime(&timestamp);
    int year = time->tm_year;
    int month = time->tm_mon;
    int day = time->tm_mday;
    int hour = time->tm_hour;
    int minute = time->tm_min;
    int second = time->tm_sec;

    std::ostringstream obj;
    obj << year + 1900 << "-" << month + 1 << "-" << day + 1 << " " << hour << ":" << minute << ":" << second;
    std::string local_time_result = obj.str();
    return local_time_result;
  };

  void RunExampleOnce() {
    result_.Clear();
    FillMsgHeader(&result_);
    gps_timestamp = apollo::cyber::common::UnixToGpsSeconds(unix_timestamp);
    local_time_str = ToLocalTime(unix_timestamp);

    result_.set_current_unix_timestamp(unix_timestamp);
    result_.set_current_gps_timestamp(gps_timestamp);

    auto mutable_data_time = result_.mutable_current_datetime();
    mutable_data_time->set_time_zone(TimeZone::BEIJING);
    mutable_data_time->set_time_description(local_time_str);
    AINFO << "unix_timestamp : " << std::setprecision(16) << unix_timestamp;
    AINFO << "gps_timestamp  : " << std::setprecision(16) << gps_timestamp;
    AINFO << "local time     : " << local_time_str;
    AINFO << "---------------------------------";
    PublishMessages(result_);
  };
};

}
}

int main(int argc, char* argv[]) {
  apollo::cyber::Init(argv[0]);
  apollo::example::TimeConversionExample example;
  if (!example.InitIO()) {
    AERROR << "Init Interval false.";
    return false;
  }

  apollo::cyber::Rate rate(10.0);
  while (apollo::cyber::OK()) {
    example.RunExampleOnce();
    rate.Sleep();
  }

  return 0;
}
