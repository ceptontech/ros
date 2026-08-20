cmake_minimum_required(VERSION 3.8)
project(stability_probe)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(stability_probe src/stability_probe_node.cpp)
ament_target_dependencies(stability_probe rclcpp sensor_msgs)

install(TARGETS stability_probe
  DESTINATION lib/${PROJECT_NAME})

ament_package()
