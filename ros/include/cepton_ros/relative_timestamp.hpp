#pragma once

#include <cstdint>
#include <limits>

namespace cepton_ros {

enum class RelativeTimestampMode {
  Delta,
  FrameOffset,
  Absolute,
};

struct AbsoluteTimestamp {
  uint32_t sec;
  uint32_t nsec;
};

class RelativeTimestampAccumulator {
 public:
  RelativeTimestampAccumulator(int64_t start_timestamp,
                               int64_t frame_start_timestamp)
      : timestamp_(start_timestamp),
        frame_start_timestamp_(frame_start_timestamp) {}

  uint16_t add_delta_and_get_delta(uint16_t relative_timestamp) {
    timestamp_ += relative_timestamp;
    return relative_timestamp;
  }

  uint32_t add_delta_and_get_frame_offset(uint16_t relative_timestamp) {
    timestamp_ += relative_timestamp;
    const int64_t offset = timestamp_ - frame_start_timestamp_;
    if (offset <= 0) return 0;
    if (offset > std::numeric_limits<uint32_t>::max()) {
      return std::numeric_limits<uint32_t>::max();
    }
    return static_cast<uint32_t>(offset);
  }

  AbsoluteTimestamp add_delta_and_get_absolute(uint16_t relative_timestamp) {
    timestamp_ += relative_timestamp;
    if (timestamp_ <= 0) return {0, 0};
    const auto sec = timestamp_ / 1000000;
    const auto usec = timestamp_ % 1000000;
    if (sec > std::numeric_limits<uint32_t>::max()) {
      return {std::numeric_limits<uint32_t>::max(), 999999000};
    }
    return {static_cast<uint32_t>(sec), static_cast<uint32_t>(usec * 1000)};
  }

 private:
  int64_t timestamp_;
  int64_t frame_start_timestamp_;
};

}  // namespace cepton_ros
