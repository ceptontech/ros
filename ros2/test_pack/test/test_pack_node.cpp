#include <signal.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "test_pack/test_pack.h"

using namespace std;

std::shared_ptr<TestPack> node;
void sigterm_handler(int sig) {
  node->dump();
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  struct sigaction action;
  action.sa_handler = sigterm_handler;
  sigaction(SIGTERM, &action, nullptr);

  node = make_shared<TestPack>();
  rclcpp::spin(node);

  return 0;
}
