cmake_minimum_required(VERSION 3.5)

set(CEPTON_ROS_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")

project(cepton_ros)

# Version shown by the publisher at startup. Same rule as the ROS2
# cepton_publisher, so both publishers report the same version.
# package.xml holds the version set by hand (X.Y.Z) and must match
# ros2/cepton_publisher/package.xml. When git is available, the patch number
# is replaced with the number of commits since the latest vX.Y.Z tag:
# tag v2.3.0 + 5 commits -> v2.3.5.
# If package.xml is newer than the latest tag (version bumped but not tagged
# yet), the package.xml version is used as is.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" PACKAGE_XML_CONTENT)
string(REGEX MATCH "<version>([^<]*)</version>" _ ${PACKAGE_XML_CONTENT})
set(PACKAGE_XML_VERSION "${CMAKE_MATCH_1}")
set(CEPTON_PUBLISHER_VERSION "v${PACKAGE_XML_VERSION}")

find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --match "v[0-9]*.[0-9]*.[0-9]*" --long
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE
    RESULT_VARIABLE GIT_DESCRIBE_RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  # e.g. v2.3.0-5-g1a2b3c4
  if(GIT_DESCRIBE_RESULT EQUAL 0 AND
     GIT_DESCRIBE MATCHES "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)-([0-9]+)-g[0-9a-f]+$")
    set(TAG_VERSION "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${CMAKE_MATCH_3}")
    math(EXPR BUILD_PATCH "${CMAKE_MATCH_3} + ${CMAKE_MATCH_4}")
    if(NOT PACKAGE_XML_VERSION VERSION_GREATER TAG_VERSION)
      set(CEPTON_PUBLISHER_VERSION "v${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${BUILD_PATCH}")
    endif()
  endif()

  # Re-run CMake when HEAD moves (commit, checkout, pull) so the version
  # does not go stale between catkin builds.
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --absolute-git-dir
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DIR
    RESULT_VARIABLE GIT_DIR_RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(GIT_DIR_RESULT EQUAL 0 AND EXISTS "${GIT_DIR}/logs/HEAD")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${GIT_DIR}/logs/HEAD")
  endif()
endif()
message(STATUS "cepton_ros publisher version: ${CEPTON_PUBLISHER_VERSION}")

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
set(SDK_VERSION 3.0.24.3)

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
  target_compile_definitions(${name} PUBLIC CEPTON_PUBLISHER_VERSION="${CEPTON_PUBLISHER_VERSION}")

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

# Install the computed version so tools can read the version of the
# installed publisher: <install>/share/cepton_ros/VERSION
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/VERSION" "${CEPTON_PUBLISHER_VERSION}\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/VERSION" DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# Subscriber
include_directories(${catkin_INCLUDE_DIRS})
add_executable(cepton_subscriber src/cepton_subscriber.cpp)
target_link_libraries(cepton_subscriber ${catkin_LIBRARIES})
install(TARGETS cepton_subscriber
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
