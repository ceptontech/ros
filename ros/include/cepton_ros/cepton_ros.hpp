#pragma once

#include <nodelet/nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <cstdint>

#include "cepton_sdk3.h"

#ifdef WITH_TS_CH_F
#pragma message("✅ timestamp-channel-flag fields are enabled")
#endif

#ifdef WITH_POLAR
#pragma message("✅ polar-coordinate fields are enabled")
#endif

namespace cepton_ros {

struct PointDelta {
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

struct PointFrameOffset {
  float x;
  float y;
  float z;
  float intensity;
#ifdef WITH_TS_CH_F
  uint32_t relative_timestamp;
  uint16_t flags;
  uint16_t channel_id;
  uint16_t valid;
#endif
#ifdef WITH_POLAR
  float azimuth;
  float elevation;
#endif
};

struct PointAbsolute {
  float x;
  float y;
  float z;
  float intensity;
#ifdef WITH_TS_CH_F
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
  uint16_t flags;
  uint16_t channel_id;
  uint16_t valid;
#endif
#ifdef WITH_POLAR
  float azimuth;
  float elevation;
#endif
};

using CloudDelta = pcl::PointCloud<PointDelta>;
using CloudFrameOffset = pcl::PointCloud<PointFrameOffset>;
using CloudAbsolute = pcl::PointCloud<PointAbsolute>;
using Point = PointFrameOffset;
using Cloud = CloudFrameOffset;

}  // namespace cepton_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_ros::PointDelta,
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

POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_ros::PointFrameOffset,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    #ifdef WITH_TS_CH_F
    (std::uint32_t, relative_timestamp, relative_timestamp)
    (std::uint16_t, flags, flags)
    (std::uint16_t, channel_id, channel_id)
    (std::uint16_t, valid, valid)
    #endif
    #ifdef WITH_POLAR
    (float, azimuth, azimuth)
    (float, elevation, elevation)
    #endif
  )

POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_ros::PointAbsolute,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    #ifdef WITH_TS_CH_F
    (std::uint32_t, timestamp_sec, timestamp_sec)
    (std::uint32_t, timestamp_nsec, timestamp_nsec)
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
