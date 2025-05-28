#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cepton_messages/msg/cepton_panic.hpp"
#include "cepton_messages/msg/cepton_point_data.hpp"
#include "cepton_messages/msg/cepton_sensor_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

enum class DataTypes { Points, CeptonPoints, Info, Panic };

class TestPack : public rclcpp::Node {
 public:
  TestPack();
  ~TestPack() {}

  void dump();

 private:
  std::vector<cepton_messages::msg::CeptonSensorInfo> info_messages_;
  std::vector<cepton_messages::msg::CeptonPanic> panic_messages_;

  // Subscriber to PointCloud2 format
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      points_subscriber_;

  // Subscriber to CeptonPointData format
  rclcpp::Subscription<cepton_messages::msg::CeptonPointData>::SharedPtr
      cepton_points_subscriber_;

  // Subscriber to info data
  rclcpp::Subscription<cepton_messages::msg::CeptonSensorInfo>::SharedPtr
      info_subscriber_;

  rclcpp::Subscription<cepton_messages::msg::CeptonPanic>::SharedPtr
      panic_subscriber_;

  void receive_cepton_points(
      const cepton_messages::msg::CeptonPointData::SharedPtr points);
  void receive_points(const sensor_msgs::msg::PointCloud2::SharedPtr points);
  void receive_info(
      const cepton_messages::msg::CeptonSensorInfo::SharedPtr info);
  void receive_panic(
      const cepton_messages::msg::CeptonPanic::SharedPtr panic_msg);

  void dump(const std::string& filename);

  std::string logfile_;
  std::mutex message_lock_;
};
