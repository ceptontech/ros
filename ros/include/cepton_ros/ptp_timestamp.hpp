#pragma once

#include <cstdint>
#include <mutex>
#include <unordered_map>

#include "cepton_sdk3.h"

namespace cepton_ros
{

struct FrameTimestamp
{
  int64_t raw_frame_start_us;
  uint64_t header_stamp_us;
};

/**
 * Resolves SDK sensor-uptime timestamps into PointCloud header timestamps.
 *
 * WITH_PTP=OFF has no maps or locking. It preserves the legacy timestamp
 * behavior, including the original frame timestamp during aggregation.
 */
class PtpTimestampResolver
{
public:
  void update_time_sync_offset(CeptonSensorHandle handle, int64_t time_sync_offset_us)
  {
#ifdef WITH_PTP
    std::lock_guard<std::mutex> lock(lock_);
    time_sync_offsets_[handle] = time_sync_offset_us;
#else
    (void)handle;
    (void)time_sync_offset_us;
#endif
  }

  FrameTimestamp begin_frame(CeptonSensorHandle handle, int64_t packet_start_us, bool reset_cloud,
                             uint64_t current_header_stamp_us)
  {
#ifdef WITH_PTP
    std::lock_guard<std::mutex> lock(lock_);

    const auto frame_it = frame_start_timestamps_.find(handle);
    const int64_t raw_frame_start_us =
        reset_cloud || frame_it == frame_start_timestamps_.end() ? packet_start_us : frame_it->second;
    if (reset_cloud)
      frame_start_timestamps_[handle] = raw_frame_start_us;

    const auto offset_it = time_sync_offsets_.find(handle);
    const int64_t time_sync_offset_us = offset_it == time_sync_offsets_.end() ? 0 : offset_it->second;
    return { raw_frame_start_us,
             static_cast<uint64_t>(raw_frame_start_us) + static_cast<uint64_t>(time_sync_offset_us) };
#else
    (void)handle;
    const int64_t raw_frame_start_us = reset_cloud ? packet_start_us : static_cast<int64_t>(current_header_stamp_us);
    return { raw_frame_start_us, static_cast<uint64_t>(raw_frame_start_us) };
#endif
  }

private:
#ifdef WITH_PTP
  std::mutex lock_;
  std::unordered_map<CeptonSensorHandle, int64_t> time_sync_offsets_;
  std::unordered_map<CeptonSensorHandle, int64_t> frame_start_timestamps_;
#endif
};

}  // namespace cepton_ros
