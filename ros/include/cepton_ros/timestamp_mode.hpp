#pragma once

#include "cepton_ros/cepton_ros.hpp"

namespace cepton_ros
{

/**
 * Compile-time selected representation for per-point timestamps.
 *
 * All mode-specific code lives here so the point conversion loop can remain
 * independent of the PointCloud2 timestamp layout.
 */
class TimestampMode
{
public:
#if defined(WITH_TS_CH_F) && !defined(CEPTON_ROS_TIMESTAMP_MODE_RELATIVE)
  struct State
  {
    double offset_us;
    bool first_packet;

    explicit State(double initial_offset_us) : offset_us(initial_offset_us), first_packet(true)
    {
    }
  };

  static State begin_packet(int64_t packet_start_us, int64_t frame_start_us)
  {
    return State(static_cast<double>(packet_start_us) - static_cast<double>(frame_start_us));
  }

  static void advance(State& state, const CeptonPointEx& point)
  {
    if (point.channel_id != 0)
      return;

    // The first channel-0 delta belongs to the preceding frame.
    if (state.first_packet)
    {
      state.first_packet = false;
      return;
    }
    state.offset_us += static_cast<double>(point.relative_timestamp);
  }

  static void assign(Point& output, const CeptonPointEx&, const State& state, uint64_t header_stamp_us)
  {
#if defined(CEPTON_ROS_TIMESTAMP_MODE_FRAME_OFFSET)
    output.timestamp = state.offset_us;
#elif defined(CEPTON_ROS_TIMESTAMP_MODE_ABSOLUTE)
    output.timestamp = static_cast<double>(header_stamp_us) + state.offset_us;
#endif
  }
#elif defined(WITH_TS_CH_F)
  struct State
  {
  };

  static State begin_packet(int64_t, int64_t)
  {
    return {};
  }

  static void advance(State&, const CeptonPointEx&)
  {
  }

  static void assign(Point& output, const CeptonPointEx& point, const State&, uint64_t)
  {
    output.relative_timestamp = point.relative_timestamp;
  }
#else
  struct State
  {
  };

  static State begin_packet(int64_t, int64_t)
  {
    return {};
  }

  static void advance(State&, const CeptonPointEx&)
  {
  }

  static void assign(Point&, const CeptonPointEx&, const State&, uint64_t)
  {
  }
#endif
};

}  // namespace cepton_ros
