#pragma once

#include <nodelet/nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "cepton_sdk3.h"

#ifdef WITH_TS_CH_F
#pragma message("✅ timestamp-channel-flag fields are enabled")
#endif

#ifdef WITH_POLAR
#pragma message("✅ polar-coordinate fields are enabled")
#endif

namespace cepton_ros {

struct Point {
  float x;
  float y;
  float z;
  float intensity;
#ifdef WITH_TS_CH_F
  uint16_t relative_timestamp;
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
    (std::uint16_t, relative_timestamp, relative_timestamp)
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
