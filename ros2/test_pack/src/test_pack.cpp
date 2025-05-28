#include "test_pack/test_pack.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iostream>

#include "json.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
using namespace std;
using namespace std::chrono;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

#pragma pack(push, 1)
struct CeptonPoint {
  int16_t x;
  uint16_t y;
  int16_t z;
  uint8_t reflectivity;
  uint8_t relative_timestamp;
  uint8_t channel_id;
  uint8_t flags;
};
#pragma pack(pop)

TestPack::TestPack() : Node("test_pack") {
  declare_parameter("logfile", "");
  rclcpp::Parameter p_logfile = get_parameter("logfile");
  if (p_logfile.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    logfile_ = p_logfile.as_string();
  } else {
    cerr << "Must specify a logfile" << endl;
    throw new runtime_error("Missing logfile");
  }
  cerr << "Using logfile " << logfile_ << endl;

  // Subscribe to PointCloud2
  points_subscriber_ = create_subscription<PointCloud2>(
      "cepton_pcl2", 10,
      bind(&TestPack::receive_points, this, placeholders::_1));

  // Subscribe to CeptonPointData
  cepton_points_subscriber_ =
      create_subscription<cepton_messages::msg::CeptonPointData>(
          "cepton_points", 50,
          bind(&TestPack::receive_cepton_points, this, placeholders::_1));

  // Subscribe to info messages
  info_subscriber_ =
      create_subscription<cepton_messages::msg::CeptonSensorInfo>(
          "cepton_info", 10,
          bind(&TestPack::receive_info, this, placeholders::_1));

  // Subscribe to panic messages
  panic_subscriber_ = create_subscription<cepton_messages::msg::CeptonPanic>(
      "cepton_panic", 10,
      bind(&TestPack::receive_panic, this, placeholders::_1));
}

void TestPack::receive_info(
    const cepton_messages::msg::CeptonSensorInfo::SharedPtr info) {
  lock_guard<mutex> lock(message_lock_);
  info_messages_.push_back(*info);
}

void TestPack::receive_points(
    const sensor_msgs::msg::PointCloud2::SharedPtr points) {}

void TestPack::receive_cepton_points(
    const cepton_messages::msg::CeptonPointData::SharedPtr points) {}

void TestPack::receive_panic(
    const cepton_messages::msg::CeptonPanic::SharedPtr panic_msg) {
  lock_guard<mutex> lock(message_lock_);
  panic_messages_.push_back(*panic_msg);
}

void TestPack::dump() {
  cout << "dump to " << logfile_ << endl;
  lock_guard<mutex> lock(message_lock_);
  ofstream s(logfile_);
  if (!s.is_open()) {
    cerr << "Failed to open " << logfile_ << endl;
    throw new runtime_error("failed to open filepath");
  }

  auto info  = nlohmann::json::array();
  auto panic = nlohmann::json::array();

  for (auto inf : info_messages_) {
    info.push_back(
        nlohmann::json{{"serial_number", inf.serial_number},
                       {"handle", inf.handle},
                       {"model_name", inf.model_name},
                       {"model", inf.model},
                       {"part_number", inf.part_number},
                       {"firmware_version", inf.firmware_version},
                       {"power_up_timestamp", inf.power_up_timestamp},
                       {"time_sync_offset", inf.time_sync_offset},
                       {"time_sync_drift", inf.time_sync_drift},
                       {"return_count", inf.return_count},
                       {"channel_count", inf.channel_count},
                       {"status_flags", inf.status_flags},
                       {"temperature", inf.temperature},
                       {"fault_summary", inf.fault_summary}});
  }

  for (auto pan : panic_messages_) {
    panic.push_back(
        nlohmann::json{{"handle", pan.handle}, {"timestamp", pan.timestamp}});
  }

  auto log     = nlohmann::json();
  log["info"]  = info;
  log["panic"] = panic;
  s << log.dump(2) << endl;
}
