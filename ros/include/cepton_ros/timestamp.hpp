#pragma once

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace cepton_ros
{

enum class TimestampMode
{
  Legacy,
  Relative,
  Absolute
};

inline TimestampMode parse_timestamp_mode(const std::string& value)
{
  if (value == "legacy")
    return TimestampMode::Legacy;
  if (value == "relative")
    return TimestampMode::Relative;
  if (value == "absolute")
    return TimestampMode::Absolute;
  throw std::invalid_argument("timestamp_mode must be legacy, relative, or absolute (got: " + value + ")");
}

// SDK point times are successive deltas in microseconds. Advance for every
// input point, including points removed by output filters. Keep this state
// across input frames belonging to the same output cloud.
class PointTimestamp
{
public:
  void reset(int64_t cloud_header_us)
  {
    current_us_ = cloud_header_us;
    cloud_header_us_ = cloud_header_us;
  }

  void advance(uint16_t delta_us)
  {
    current_us_ += delta_us;
  }

  uint32_t frame_offset_us() const
  {
    const int64_t offset_us = current_us_ - cloud_header_us_;
    if (offset_us <= 0)
      return 0;
    if (offset_us >= std::numeric_limits<uint32_t>::max())
      return std::numeric_limits<uint32_t>::max();
    return static_cast<uint32_t>(offset_us);
  }

  uint64_t timestamp_us() const
  {
    // PointCloud2's UINT64 field cannot represent a pre-epoch timestamp.
    return current_us_ <= 0 ? 0 : static_cast<uint64_t>(current_us_);
  }

private:
  int64_t current_us_{ 0 };
  int64_t cloud_header_us_{ 0 };
};

}  // namespace cepton_ros
