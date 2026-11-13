cmake_minimum_required(VERSION 3.5)
project(cepton_publisher)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Version shown by the publisher at startup.
# package.xml holds the version set by hand (X.Y.Z). When git is available,
# the patch number is replaced with the number of commits since the latest
# vX.Y.Z tag: tag v2.3.0 + 5 commits -> v2.3.5.
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
  # does not go stale between colcon builds.
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
message(STATUS "cepton_publisher version: ${CEPTON_PUBLISHER_VERSION}")

# Example usage: `colcon build --cmake-args -DWITH_POLAR=OFF` to disable polar coordinates.
option(WITH_POLAR "Compile point types with polar coordinates" ON)
option(WITH_TS "Include timestamp fields" ON)
option(WITH_CH "Include channel ID field" ON)
option(WITH_F "Include flag field" ON)
# Empty keeps the individual switches; ON/OFF preserves the old group switch.
set(WITH_TS_CH_F "" CACHE STRING "Legacy switch for timestamp, channel ID, and flag fields")
set_property(CACHE WITH_TS_CH_F PROPERTY STRINGS "" ON OFF)
if(NOT WITH_TS_CH_F STREQUAL "")
  set(WITH_TS ${WITH_TS_CH_F})
  set(WITH_CH ${WITH_TS_CH_F})
  set(WITH_F ${WITH_TS_CH_F})
endif()
option(WITH_PTP "PTP MODE: ON" ON)
option(RELATIVE "Include frame offset field" OFF)


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

set(SDK_VERSION 3.0.24.3)

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
add_library(${PROJECT_NAME} SHARED src/cepton_publisher.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
                          $<INSTALL_INTERFACE:${PROJECT_NAME}/include>)
target_compile_definitions(
  ${PROJECT_NAME} PUBLIC CEPTON_PUBLISHER_VERSION="${CEPTON_PUBLISHER_VERSION}")
if (WITH_POLAR)
  target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_POLAR)
endif()
if (WITH_TS)
  target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_TS)
endif()
if (WITH_CH)
  target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_CH)
endif()
if (WITH_F)
  target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_F)
endif()
if (WITH_PTP)
  target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_PTP)
endif()
if (RELATIVE)
  target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_FRAME_OFFSET)
endif()

include_directories(../../cepton-sdk-${SDK_VERSION}/include)

# Link ros dependencies
# CMake recommends replacing this with target_link_libraries, but that doesn't
# work for ubuntu 20 ROS. So just disable the warning for now.
set(CMAKE_WARN_DEPRECATED OFF)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp_components rclcpp
                          std_msgs sensor_msgs cepton_messages)

target_link_libraries(${PROJECT_NAME} PUBLIC ${CEPTON_SDK3_LIB})

# Install
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# -------------------------------------------------------------------
# Publisher standalone publisher node
# -------------------------------------------------------------------

add_executable(cepton_publisher_node src/main.cpp)
target_link_libraries(cepton_publisher_node cepton_publisher)
install(TARGETS cepton_publisher_node DESTINATION lib/${PROJECT_NAME})

# Install SDK dependencies to the node location
install(FILES ${SDK_BINARIES} DESTINATION lib)

ament_package()
