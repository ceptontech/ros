#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cepton_sdk2.h"

namespace cepton2_ros {

struct CustomCeptonPoint {
  float x;
  float y;
  float z;
  float reflectivity;
  uint8_t relative_timestamp;
  uint8_t flags;
  uint8_t channel_id;
  uint8_t valid;
  float azimuth;
  float elevation;
};

using CeptonPointCloud = pcl::PointCloud<CustomCeptonPoint>;
}  // namespace cepton2_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton2_ros::CustomCeptonPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, reflectivity, reflectivity)
    (std::uint8_t, relative_timestamp, relative_timestamp)
    (std::uint8_t, flags, flags)
    (std::uint8_t, channel_id, channel_id)
    (std::uint8_t, valid, valid)
    (float, azimuth, azimuth)
    (float, elevation, elevation)
  )
// clang-format on
