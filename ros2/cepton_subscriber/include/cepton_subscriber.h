#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cepton_messages/msg/cepton_point_data.hpp"
#include "cepton_messages/msg/cepton_sensor_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

class CeptonSubscriber : public rclcpp::Node {
 public:
  CeptonSubscriber();
  ~CeptonSubscriber() {}

 private:
  // Subscriber to PointCloud2 format
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointsSubscriber;

  // Subscriber to CeptonPointData format
  rclcpp::Subscription<cepton_messages::msg::CeptonPointData>::SharedPtr
      ceptonPointsSubscriber;

  // Subscriber to info data
  rclcpp::Subscription<cepton_messages::msg::CeptonSensorInfo>::SharedPtr
      infoSubscriber;

  void recv_cep_points(
      const cepton_messages::msg::CeptonPointData::SharedPtr points);
  void recv_points(const sensor_msgs::msg::PointCloud2::SharedPtr points);
  void recv_info(const cepton_messages::msg::CeptonSensorInfo::SharedPtr info);

  rclcpp::NodeOptions options;

  bool export_to_csv_;
};
