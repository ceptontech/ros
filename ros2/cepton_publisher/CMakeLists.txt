cmake_minimum_required(VERSION 3.5)
project(cepton_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

 
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
  add_compile_options(-Wno-cast-function-type) # suppress function pointer cast stderr messages for CI
endif()

# find the non-sdk dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

###################################################################
# Link the SDK, first get the binaries for the correct architecture

set(SDK_VERSION 3.0.18.0)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "amd64.*|x86_64.*|AMD64.*")
  set(SDK_SOURCE  ${PROJECT_SOURCE_DIR}/../../cepton-sdk-${SDK_VERSION}/lib/linux-x64)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*|arm64.*|ARM64.*)")
  set(SDK_SOURCE ${PROJECT_SOURCE_DIR}/../../cepton-sdk-${SDK_VERSION}/lib/linux-arm64)
else()
  message("NO LIBRARY FOUND MATCHING PROCESSOR")
endif()

# Then glob the shared objects
file(GLOB SDK_BINARIES ${SDK_SOURCE}/*)

set(CEPTON_SDK3_LIB ${SDK_SOURCE}/libcepton_sdk3.so)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:\$ORIGIN/..")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:\$ORIGIN/../lib")

# -------------------------------------------------------------------------------
# Compile CeptonPublisher
# -------------------------------------------------------------------------------

find_package(cepton_messages REQUIRED)
add_library(${PROJECT_NAME} SHARED src/cepton_publisher.cpp src/panic.h)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
                          $<INSTALL_INTERFACE:${PROJECT_NAME}/include>)

include_directories(../../cepton-sdk-${SDK_VERSION}/include)

# Link ros dependencies
target_link_libraries(cepton_publisher PUBLIC
  ${cepton_messages_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
  ${CEPTON_SDK3_LIB}
)


# Install
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# -------------------------------------------------------------------------------
# Other setup - Install launch file
# -------------------------------------------------------------------------------

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# -------------------------------------------------------------------
# Publisher standalone publisher node
# -------------------------------------------------------------------

add_executable(cepton_publisher_node src/main.cpp)
target_link_libraries(cepton_publisher_node cepton_publisher)
install(TARGETS cepton_publisher_node DESTINATION lib/${PROJECT_NAME})

# Install SDK dependencies to the node location
install(FILES ${SDK_BINARIES} DESTINATION lib)

ament_package()
