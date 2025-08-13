cmake_minimum_required(VERSION 3.8)
project(cepton_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cepton_messages REQUIRED)

add_library(cepton_subscriber SHARED src/cepton_subscriber.cpp)
target_include_directories(
  cepton_subscriber PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
                           $<INSTALL_INTERFACE:${PROJECT_NAME}/include>)
# Link ros dependencies
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp_components rclcpp
                          std_msgs sensor_msgs cepton_messages)

# Install
install(
  TARGETS cepton_subscriber
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# -------------------------------------------------------------------
# Subscriber standalone node
add_executable(cepton_subscriber_node src/main.cpp)
target_link_libraries(cepton_subscriber_node cepton_subscriber)
install(TARGETS cepton_subscriber_node DESTINATION lib/${PROJECT_NAME})


# Copy dependencies to the node location
install(FILES ${CeptonSDK2_LIBRARY} ${CeptonSDKLegacy_LIBRARY} DESTINATION lib)

ament_package()
