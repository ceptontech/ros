#pragma once

#include <cstdint>

#include <nodelet/nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "cepton_sdk3.h"

#ifdef WITH_TS_CH_F
#if defined(CEPTON_ROS_TIMESTAMP_MODE_RELATIVE)
#pragma message("✅ relative point timestamps, channel, and flag fields are enabled")
#elif defined(CEPTON_ROS_TIMESTAMP_MODE_FRAME_OFFSET)
#pragma message("✅ frame-offset point timestamps, channel, and flag fields are enabled")
#elif defined(CEPTON_ROS_TIMESTAMP_MODE_ABSOLUTE)
#pragma message("✅ absolute point timestamps, channel, and flag fields are enabled")
#else
#error "WITH_TS_CH_F requires a CEPTON_ROS_TIMESTAMP_MODE_* compile definition"
#endif
#endif

#ifdef WITH_POLAR
#pragma message("✅ polar-coordinate fields are enabled")
#endif

namespace cepton_ros
{

struct Point
{
  float x;
  float y;
  float z;
  float intensity;
#ifdef WITH_TS_CH_F
#ifdef CEPTON_ROS_TIMESTAMP_MODE_RELATIVE
  uint16_t relative_timestamp;
#else
  // Integer-valued microseconds transported as FLOAT64 for ROS1/PCL PointCloud2 compatibility.
  double timestamp;
#endif
  uint16_t flags;
  uint16_t channel_id;
  uint16_t valid;
#endif
#ifdef WITH_POLAR
  float azimuth;
  float elevation;
#endif
};

using Cloud = pcl::PointCloud<Point>;

}  // namespace cepton_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    #ifdef WITH_TS_CH_F
    #ifdef CEPTON_ROS_TIMESTAMP_MODE_RELATIVE
    (std::uint16_t, relative_timestamp, relative_timestamp)
    #else
    (double, timestamp, timestamp)
    #endif
    (std::uint16_t, flags, flags)
    (std::uint16_t, channel_id, channel_id)
    (std::uint16_t, valid, valid)
    #endif
    #ifdef WITH_POLAR
    (float, azimuth, azimuth)
    (float, elevation, elevation)
    #endif
  )
// clang-format on
