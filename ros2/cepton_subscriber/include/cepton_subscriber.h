#include <memory>
#include <string>

#include "cepton_messages/msg/cepton_sensor_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// version is generated from package.xml at build time, will display in
// terminal running subscriber
#ifndef CEPTON_SUBSCRIBER_VERSION
#define CEPTON_SUBSCRIBER_VERSION "unknown"
#endif
inline std::string VERSION = CEPTON_SUBSCRIBER_VERSION;

class CeptonSubscriber : public rclcpp::Node
{
public:
  CeptonSubscriber();
  ~CeptonSubscriber() {}

private:
  // Subscriber to PointCloud2 format
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointsSubscriber;

  // Subscriber to info data
  rclcpp::Subscription<cepton_messages::msg::CeptonSensorInfo>::SharedPtr infoSubscriber;

  void recv_points(const sensor_msgs::msg::PointCloud2::SharedPtr points);
  void recv_info(const cepton_messages::msg::CeptonSensorInfo::SharedPtr info);

  rclcpp::NodeOptions options;

  bool export_to_csv_;
};
