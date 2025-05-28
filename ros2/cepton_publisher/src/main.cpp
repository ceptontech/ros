#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cepton_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
std::shared_ptr<cepton_ros::CeptonPublisher> node;
void sigterm_handler(int) { rclcpp::shutdown(); }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  struct sigaction action;
  action.sa_handler = sigterm_handler;
  sigaction(SIGTERM, &action, nullptr);

  node = make_shared<cepton_ros::CeptonPublisher>();
  rclcpp::spin(node);

  return 0;
}
