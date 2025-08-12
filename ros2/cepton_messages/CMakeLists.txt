cmake_minimum_required(VERSION 3.8)
project(cepton_messages)

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
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CeptonSensorInfo.msg"
  "msg/CeptonPointData.msg"
  "msg/CeptonSensorStatus.msg"
)


# Check ROS distribution
# https://answers.ros.org/question/405158/expected-include-directory-with-humble/
if ("$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "foxy")
  install(DIRECTORY include/ DESTINATION "include")
else()
  install(DIRECTORY include/ DESTINATION "include/${PROJECT_NAME}")
endif()

ament_export_include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
