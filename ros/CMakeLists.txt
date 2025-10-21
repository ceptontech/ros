cmake_minimum_required(VERSION 3.5)

set(CEPTON_ROS_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")

project(cepton_ros)

# For using in vscode, you can set 'C_Cpp.default.compileCommands' in your vscode settings.json
# to reflect these ifdefs to the code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Example usage: `catkin_make -DWITH_POLAR=OFF` to disable polar coordinates.
option(WITH_POLAR "Compile point types with polar coordinates" ON)
option(WITH_TS_CH_F "Compile point types with timestamp, channel, and flag" ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SensorInformation.msg
  SensorPanic.msg
  CeptonSensorStatus.msg
)

###################################################################
# Link the SDK, first get the binaries for the correct architecture
set(SDK_VERSION 3.0.19.27)

# Get the path of this cmake lists file. We'll use it to find the SDK which should
# have also been downloaded as part of the repo
get_filename_component(ROS_PATH "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "amd64.*|x86_64.*|AMD64.*")
  set(SDK_SOURCE  ${ROS_PATH}/../cepton-sdk-${SDK_VERSION}/lib/linux-x64)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*|arm64.*|ARM64.*)")
  set(SDK_SOURCE ${ROS_PATH}/../cepton-sdk-${SDK_VERSION}/lib/linux-arm64)
else()
  message("NO LIBRARY FOUND MATCHING PROCESSOR")
endif()


# Then glob the shared objects
file(GLOB SDK_BINARIES ${SDK_SOURCE}/*)
set(CEPTON_SDK3_LIB ${SDK_SOURCE}/libcepton_sdk3.so)
set(CEPTON_SDK3_INCLUDE ${ROS_PATH}/../cepton-sdk-${SDK_VERSION}/include)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:\$ORIGIN/..")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:\$ORIGIN/../lib")

###################################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(CEPTON_ROS_LIBRARIES "")

add_library(
  cepton_ros
  "${PROJECT_SOURCE_DIR}/src/publisher_nodelet.cpp"
)

list(APPEND CEPTON_ROS_LIBRARIES cepton_ros)
message("Including ${CEPTON_SDK3_INCLUDE}")
include_directories(${CEPTON_SDK3_INCLUDE})

message(STATUS "Detected processor: ${CMAKE_SYSTEM_PROCESSOR}")
foreach(name IN LISTS CEPTON_ROS_LIBRARIES)
  target_include_directories(
    ${name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include>)
  target_link_libraries(${name} ${catkin_LIBRARIES} ${CEPTON_SDK3_LIB})
  add_dependencies(${name} cepton_ros_generate_messages_cpp)

  # If building with the polar coordinates then set the 
  # preprocessor definition
  if (WITH_POLAR)
    target_compile_definitions(${name} PUBLIC WITH_POLAR)
  endif()

  # If building with the timestamp, channel, flag, then set the 
  # preprocessor definition
  if (WITH_TS_CH_F)
    target_compile_definitions(${name} PUBLIC WITH_TS_CH_F)
  endif()

endforeach()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${CEPTON_ROS_LIBRARIES}
  CATKIN_DEPENDS message_runtime nodelet pcl_conversions pcl_ros roscpp pluginlib roslib std_msgs)

message("Install to ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
install(DIRECTORY "include/cepton/"
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

message("install libraries to ${CATKIN_PACKAGE_LIB_DESTINATION}")
install(
  TARGETS ${CEPTON_ROS_LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# Subscriber
include_directories(${catkin_INCLUDE_DIRS})
add_executable(cepton_subscriber src/cepton_subscriber.cpp)
target_link_libraries(cepton_subscriber ${catkin_LIBRARIES})
install(TARGETS cepton_subscriber
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
