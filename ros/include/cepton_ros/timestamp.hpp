#pragma once

#include <cstdint>
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
// input point, including points removed by output filters.
class PointTimestamp
{
public:
  PointTimestamp(int64_t frame_start_us, int64_t ptp_offset_us, int64_t cloud_header_us)
    : current_us_(frame_start_us + ptp_offset_us), cloud_header_us_(cloud_header_us)
  {
  }

  void advance(uint16_t delta_us)
  {
    current_us_ += delta_us;
  }

  double seconds(TimestampMode mode) const
  {
    const int64_t value = mode == TimestampMode::Relative ? current_us_ - cloud_header_us_ : current_us_;
    return static_cast<double>(value) / 1000000.0;
  }

private:
  int64_t current_us_;
  int64_t cloud_header_us_;
};

}  // namespace cepton_ros
