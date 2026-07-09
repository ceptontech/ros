cmake_minimum_required(VERSION 3.5)
project(stability_probe)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(stability_probe_node src/stability_probe_node.cpp)
target_link_libraries(stability_probe_node ${catkin_LIBRARIES})

install(TARGETS stability_probe_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
