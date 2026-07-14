#include <memory>

#include "cepton_subscriber.h"
#include "rclcpp/rclcpp.hpp"

using namespace std;
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<CeptonSubscriber>());
  rclcpp::shutdown();
  return 0;
}
