#include "publisher_nodelet.hpp"

#include <cepton_ros/relative_timestamp.hpp>

#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <pluginlib/class_list_macros.h>

#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(cepton_ros::PublisherNodelet, nodelet::Nodelet);

inline double degrees_to_radians(double t) { return t * M_PI / 180.0; }

/**
 * Parse a network source string in the format "ip:port" or "ip:port:multicast"
 * @param source_str The source string to parse
 * @param ip Output parameter for IP address (empty string for nullptr/0.0.0.0)
 * @param port Output parameter for port number
 * @param multicast_group Output parameter for multicast group (empty string for
 * nullptr)
 * @return true if parsing succeeded, false otherwise
 */
bool parse_network_source(const std::string& source_str, std::string& ip,
                          uint16_t& port, std::string& multicast_group) {
  // Split by ':' delimiter
  size_t first_colon = source_str.find(':');
  if (first_colon == std::string::npos) {
    return false;
  }

  ip = source_str.substr(0, first_colon);
  size_t second_colon = source_str.find(':', first_colon + 1);

  std::string port_str;
  if (second_colon != std::string::npos) {
    // Format: ip:port:multicast
    port_str =
        source_str.substr(first_colon + 1, second_colon - first_colon - 1);
    multicast_group = source_str.substr(second_colon + 1);
  } else {
    // Format: ip:port
    port_str = source_str.substr(first_colon + 1);
    multicast_group = "";
  }

  // Parse port
  try {
    int port_int = std::stoi(port_str);
    if (port_int < 0 || port_int > 65535) {
      return false;
    }
    port = static_cast<uint16_t>(port_int);
  } catch (...) {
    return false;
  }

  return true;
}

namespace cepton_ros {

const char* timestamp_mode_to_string(RelativeTimestampMode mode) {
  switch (mode) {
    case RelativeTimestampMode::Delta:
      return "delta";
    case RelativeTimestampMode::FrameOffset:
      return "frame_offset";
    case RelativeTimestampMode::Absolute:
      return "absolute";
  }
  return "frame_offset";
}

RelativeTimestampMode parse_timestamp_mode(const std::string& mode) {
  if (mode == "delta") return RelativeTimestampMode::Delta;
  if (mode == "frame_offset") return RelativeTimestampMode::FrameOffset;
  if (mode == "absolute") return RelativeTimestampMode::Absolute;
  ROS_WARN("Unknown timestamp_mode '%s'; using frame_offset", mode.c_str());
  return RelativeTimestampMode::FrameOffset;
}

std::string make_mirror_sync_csv_path(const std::string& base_path,
                                      uint64_t sequence) {
  const auto slash = base_path.find_last_of("/\\");
  const auto dot = base_path.find_last_of('.');
  const bool has_extension =
      dot != std::string::npos && (slash == std::string::npos || dot > slash);
  const auto stem = has_extension ? base_path.substr(0, dot) : base_path;
  const auto extension = has_extension ? base_path.substr(dot) : ".csv";

  std::ostringstream path;
  path << stem << '_' << std::setfill('0') << std::setw(6) << sequence
       << extension;
  return path.str();
}

ros::Publisher advertise_points(ros::NodeHandle& node_handle,
                                const std::string& topic,
                                RelativeTimestampMode mode,
                                uint32_t queue_size) {
  if (mode == RelativeTimestampMode::Delta) {
    return node_handle.advertise<cepton_ros::CloudDelta>(topic, queue_size);
  }
  if (mode == RelativeTimestampMode::Absolute) {
    return node_handle.advertise<cepton_ros::CloudAbsolute>(topic, queue_size);
  }
  return node_handle.advertise<cepton_ros::CloudFrameOffset>(topic, queue_size);
}

/**
 * SDK callback
 */
void on_ex_frame(CeptonSensorHandle handle, int64_t start_timestamp,
                 size_t n_points, const struct CeptonPointEx* points,
                 void* user_data) {
  reinterpret_cast<PublisherNodelet*>(user_data)->publish_points(
      handle, start_timestamp, n_points, points);
}

/**
 * SDK callback
 */
void on_sensor_info(CeptonSensorHandle handle, const struct CeptonSensor* info,
                    void* user_data) {
  reinterpret_cast<PublisherNodelet*>(user_data)->publish_sensor_info(info);
}

PublisherNodelet::~PublisherNodelet() {
  int ret;

  // Remove all networking sources
  for (const auto& source : networking_sources_) {
    const char* ip_ptr = (source.ip.empty() || source.ip == "0.0.0.0")
                             ? nullptr
                             : source.ip.c_str();
    const char* multicast_ptr = source.multicast_group.empty()
                                    ? nullptr
                                    : source.multicast_group.c_str();
    ret = CeptonRemoveNetworkingSource(ip_ptr, source.port, multicast_ptr);
    check_api_error(ret, "CeptonRemoveNetworkingSource");
  }

  ret = CeptonDeinitialize();
  check_api_error(ret, "CeptonDeinitialize");
}

void PublisherNodelet::check_api_error(int err, char const* api) {
  if (err != CEPTON_SUCCESS) {
    NODELET_ERROR("[%s] API error: %s -> %s", getName().c_str(), api,
                  CeptonGetErrorCodeName(err));
    exit(1);
  }
}

/**
 * Sensors are reported as "timed-out" after this duration, in the sensor status
 * topic
 */
const auto SENSOR_POINTS_TIMEOUT = std::chrono::seconds(3);

void PublisherNodelet::onInit() {
  ROS_INFO("\n\n========== Version: %s ==========\n ", VERSION.c_str());
  ROS_INFO("Publisher Nodelet Started");

  int ret;
  node_handle_ = getNodeHandle();
  private_node_handle_ = getPrivateNodeHandle();

  // Get parameters
  std::string capture_path = "";
  private_node_handle_.param("capture_path", capture_path, capture_path);

  bool capture_loop = true;
  uint32_t replay_flags = 0;
  private_node_handle_.param("capture_loop", capture_loop, capture_loop);
  if (capture_loop) {
    replay_flags = CEPTON_REPLAY_FLAG_PLAY_LOOPED;
  }

  private_node_handle_.param("min_altitude", min_altitude_, min_altitude_);
  private_node_handle_.param("max_altitude", max_altitude_, max_altitude_);
  private_node_handle_.param("min_azimuth", min_azimuth_, min_azimuth_);
  private_node_handle_.param("max_azimuth", max_azimuth_, max_azimuth_);

  // Clip so that we can safely take the tangent
  min_azimuth_ = std::max(-89.9, min_azimuth_);
  max_azimuth_ = std::min(89.9, max_azimuth_);
  min_altitude_ = std::max(-89.9, min_altitude_);
  max_altitude_ = std::min(89.9, max_altitude_);

  min_image_x_ = std::tan(degrees_to_radians(min_azimuth_));
  max_image_x_ = std::tan(degrees_to_radians(max_azimuth_));
  min_image_z_ = std::tan(degrees_to_radians(min_altitude_));
  max_image_z_ = std::tan(degrees_to_radians(max_altitude_));

  private_node_handle_.param("max_distance", max_distance_, max_distance_);
  private_node_handle_.param("min_distance", min_distance_, min_distance_);

  private_node_handle_.param("output_by_handle", output_by_handle_,
                             output_by_handle_);
  private_node_handle_.param("output_by_sn", output_by_sn_, output_by_sn_);

  std::vector<std::string> sensor_network_sources;
  private_node_handle_.param("sensor_network_sources", sensor_network_sources,
                             sensor_network_sources);

  private_node_handle_.param("expected_sensor_ips", expected_sensor_ips_,
                             expected_sensor_ips_);

  private_node_handle_.param("aggregate_frames", aggregate_frames_,
                             aggregate_frames_);

  mirror_sync_csv_path_ = "mirror_sync.csv";
  private_node_handle_.param("mirror_sync_csv_path", mirror_sync_csv_path_,
                             mirror_sync_csv_path_);
  private_node_handle_.param("mirror_sync_interval_sec",
                             mirror_sync_interval_sec_,
                             mirror_sync_interval_sec_);
  private_node_handle_.param("mirror_sync_duration_sec",
                             mirror_sync_duration_sec_,
                             mirror_sync_duration_sec_);

  if (mirror_sync_interval_sec_ <= 0.0 || mirror_sync_duration_sec_ <= 0.0 ||
      mirror_sync_duration_sec_ > mirror_sync_interval_sec_) {
    ROS_WARN("Invalid mirror sync sampling period; using 1 second every 20 "
             "seconds");
    mirror_sync_interval_sec_ = 20.0;
    mirror_sync_duration_sec_ = 1.0;
  }

  if (!mirror_sync_csv_path_.empty()) {
    mirror_sync_start_time_ = std::chrono::steady_clock::now();
    ROS_INFO("mirror sync CSV base path: %s", mirror_sync_csv_path_.c_str());
    ROS_INFO("mirror sync sampling: one CSV per %.3f-second measurement, "
             "every %.3f seconds",
             mirror_sync_duration_sec_, mirror_sync_interval_sec_);
  }

#ifdef WITH_TS_CH_F
  std::string timestamp_mode = "frame_offset";
  private_node_handle_.param("relative_timestamp_mode", timestamp_mode,
                             timestamp_mode);
  private_node_handle_.param("timestamp_mode", timestamp_mode, timestamp_mode);
  timestamp_mode_ = parse_timestamp_mode(timestamp_mode);
  ROS_INFO("timestamp mode: %s", timestamp_mode_to_string(timestamp_mode_));
#endif

  // Check for which points should be included based on params for flag bits
  {
    bool include = true;

    ROS_INFO("============= Point Flag Parameters =============\n");

    private_node_handle_.param("include_saturated_points", include, true);
    include_flag_ |= (include ? CEPTON_POINT_SATURATED : 0);
    ROS_INFO("Including Saturated points: %s\n", include ? "true" : "false");

    // keep frame low_snr/parity/boundary as internal for now

    private_node_handle_.param("include_second_return_points", include, true);
    include_flag_ |= (include ? CEPTON_POINT_SECOND_RETURN : 0);
    ROS_INFO("Including Second Return  points: %s\n",
             include ? "true" : "false");

    private_node_handle_.param("include_invalid_points", include, false);
    include_flag_ |= (include ? CEPTON_POINT_NO_RETURN : 0);
    ROS_INFO("Including Invalid (No Return) points: %s\n",
             include ? "true" : "false");

    private_node_handle_.param("include_noise_points", include, false);
    include_flag_ |= (include ? CEPTON_POINT_NOISE : 0);
    ROS_INFO("Including Noise points: %s\n", include ? "true" : "false");

    private_node_handle_.param("include_blocked_points", include, false);
    include_flag_ |= (include ? CEPTON_POINT_BLOCKED : 0);
    ROS_INFO("Including Blocked points: %s\n", include ? "true" : "false");

    private_node_handle_.param("include_retro_points", include, false);
    include_flag_ |= (include ? CEPTON_POINT_RETRO : 0);
    ROS_INFO("Including Retro points: %s\n", include ? "true" : "false");

    private_node_handle_.param("include_retro_weak_points", include, false);
    include_flag_ |= (include ? CEPTON_POINT_RETRO_WEAK : 0);
    ROS_INFO("Including Retro Weak points: %s\n",
             include ? "true" : "false");

    private_node_handle_.param("include_ambient_points", include, false);
    include_flag_ |= (include ? CEPTON_POINT_AMBIENT : 0);
    ROS_INFO("Including Ambient points: %s\n", include ? "true" : "false");

    ROS_INFO("=================================================\n");
  }

  // Assign publisher for info
  sensor_info_publisher_ =
      node_handle_.advertise<cepton_ros::SensorInformation>(
          "cepton3/sensor_information", 2);

  // All points
  points_publisher_ =
      advertise_points(node_handle_, "cepton3/points", timestamp_mode_, 50);

  // Initialize SDK
  ret = CeptonInitialize(CEPTON_API_VERSION, nullptr);
  check_api_error(ret, "CeptonInitialize");

  uint32_t control_flags = 0;

  if (include_flag_ & CEPTON_POINT_AMBIENT) {
    control_flags |= CEPTON_SDK_CONTROL_FLAG_PARSE_AMBIENT;
  } else {
    control_flags |= CEPTON_SDK_CONTROL_FLAG_PARSE_TOF;
  }

  if (include_flag_ & CEPTON_POINT_SECOND_RETURN) {
    control_flags |= CEPTON_SDK_CONTROL_FLAG_RETURN_BOTH;
  } else {
    control_flags |= CEPTON_SDK_CONTROL_FLAG_RETURN_FIRST;
  }

  ret = CeptonSetSdkControlFlags(control_flags);
  check_api_error(ret, "CeptonSetSdkControlFlags");

  // Start SDK CaptureReplay or Networking
  if (!capture_path.empty()) {
    ret = CeptonReplayLoadPcap(capture_path.c_str(), replay_flags,
                               &replay_handle_);
    check_api_error(ret, "CeptonReplayLoadPcap");
  } else {
    // Add networking sources for UDP data from specified network sources.
    // Format is "ip:port" or "ip:port:multicast_group"
    if (sensor_network_sources.empty()) {
      // If no sources specified, add default networking source (all
      // interfaces, port 8808). Listen on all interfaces (nullptr) with no
      // multicast group (nullptr for unicast operation)
      ROS_INFO("Add networking source on default: 0.0.0.0:8808");
      ret = CeptonAddNetworkingSource(nullptr, 8808, nullptr);
      check_api_error(ret, "CeptonAddNetworkingSource");
      networking_sources_.push_back({"0.0.0.0", 8808, ""});
    } else {
      // Add a networking source for each configured source string
      for (const auto& source_str : sensor_network_sources) {
        std::string ip;
        uint16_t port;
        std::string multicast_group;

        if (!parse_network_source(source_str, ip, port, multicast_group)) {
          ROS_ERROR("Failed to parse network source: %s", source_str.c_str());
          continue;
        }

        // Convert empty IP string or "0.0.0.0" to nullptr for SDK
        const char* ip_ptr =
            (ip.empty() || ip == "0.0.0.0") ? nullptr : ip.c_str();
        const char* multicast_ptr =
            multicast_group.empty() ? nullptr : multicast_group.c_str();

        ROS_INFO("Add networking source: ip=%s, port=%d, multicast=%s",
                 ip_ptr ? ip_ptr : "0.0.0.0", port,
                 multicast_ptr ? multicast_ptr : "none");

        ret = CeptonAddNetworkingSource(ip_ptr, port, multicast_ptr);
        if (ret != CEPTON_SUCCESS) {
          NODELET_ERROR(
              "Source not added successfully, check that sensor is configured "
              "properly and running: %s",
              source_str.c_str());
        }
        // Store for cleanup
        networking_sources_.push_back({ip, port, multicast_group});
      }
    }
  }

  // Listen for frames
  ret =
      CeptonListenFramesEx(CEPTON_AGGREGATION_MODE_NATURAL, on_ex_frame, this);
  check_api_error(ret, "CeptonListenFrames");

  // Listen for sensor info
  ret = CeptonListenSensorInfo(on_sensor_info, this);
  check_api_error(ret, "CeptonListenSensorInfo");

  // Start watchdog timer, if using pcap replay
  if (!capture_path.empty()) {
    watchdog_timer_ = node_handle_.createTimer(
        ros::Duration(0.1), [&](const ros::TimerEvent& event) {
          // Shut down if a replay was loaded, and if the replay is finished
          if (replay_handle_ != 0 &&
              CeptonReplayIsFinished(replay_handle_) == 1) {
            NODELET_INFO("[%s] capture replay done", getName().c_str());
            ret = CeptonReplayUnloadPcap(replay_handle_);
            check_api_error(ret, "CeptonReplayUnload");

            ros::shutdown();
          }
        });
  }

  // Status monitoring
  {
    sensor_status_publisher_ =
        node_handle_.advertise<cepton_ros::CeptonSensorStatus>(
            "cepton3/cepton_sensor_status", 2);

    // Start the status monitor
    sensor_status_thread = std::thread([&]() {
      while (!stopping_) {
        // Store the handle and serial number of each timed out sensor.
        // Use 0 if no serial number has been found.
        std::unordered_map<CeptonSensorHandle, uint32_t> timed_out_sensors;
        {
          std::lock_guard<std::mutex> lock(status_lock_);
          // Update status messages for each sensor
          auto now = std::chrono::system_clock::now();

          for (auto itr = last_points_time_.begin();
               itr != last_points_time_.end(); itr++) {
            auto const handle = itr->first;
            auto const last_points_time = itr->second;
            if (now - last_points_time > SENSOR_POINTS_TIMEOUT) {
              timed_out_sensors[handle] = handle_to_serial_number_.count(handle)
                                              ? handle_to_serial_number_[handle]
                                              : 0;
            }
          }
          for (auto itr = timed_out_sensors.begin();
               itr != timed_out_sensors.end(); itr++) {
            auto const handle = itr->first;
            auto const serial_number = itr->second;
            auto msg = cepton_ros::CeptonSensorStatus();
            msg.serial_number = serial_number;
            msg.handle = handle;
            msg.status = SENSOR_TIMED_OUT;
            sensor_status_publisher_.publish(msg);
          }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });
  }

  // Set up the expected IPs (convert IP strings to u32)
  for (auto const& expected_ip : expected_sensor_ips_) {
    ROS_INFO("Adding expected IP %s", expected_ip.c_str());
    struct in_addr addr;
    inet_aton(expected_ip.c_str(), &addr);
    // handle is in big-endian. inet_aton returns little endian
    last_points_time_[__bswap_32(addr.s_addr)] =
        std::chrono::system_clock::now();
  }
}

template <typename CloudT>
void set_point_timestamp(typename CloudT::PointType& point,
                         RelativeTimestampAccumulator& timestamp_accumulator,
                         uint16_t relative_timestamp) {
  (void)point;
  (void)timestamp_accumulator;
  (void)relative_timestamp;
}

#ifdef WITH_TS_CH_F
template <>
void set_point_timestamp<cepton_ros::CloudDelta>(
    cepton_ros::PointDelta& point,
    RelativeTimestampAccumulator& timestamp_accumulator,
    uint16_t relative_timestamp) {
  point.relative_timestamp =
      timestamp_accumulator.add_delta_and_get_delta(relative_timestamp);
}

template <>
void set_point_timestamp<cepton_ros::CloudFrameOffset>(
    cepton_ros::PointFrameOffset& point,
    RelativeTimestampAccumulator& timestamp_accumulator,
    uint16_t relative_timestamp) {
  point.relative_timestamp =
      timestamp_accumulator.add_delta_and_get_frame_offset(relative_timestamp);
}

template <>
void set_point_timestamp<cepton_ros::CloudAbsolute>(
    cepton_ros::PointAbsolute& point,
    RelativeTimestampAccumulator& timestamp_accumulator,
    uint16_t relative_timestamp) {
  const auto timestamp =
      timestamp_accumulator.add_delta_and_get_absolute(relative_timestamp);
  point.timestamp_sec = timestamp.sec;
  point.timestamp_nsec = timestamp.nsec;
}
#endif

template <typename CloudT>
void extend_from_points(CloudT& cloud, int64_t start_timestamp,
                        size_t n_points, const CeptonPointEx* points,
                        bool reset_cloud, float min_distance, float max_distance,
                        float min_image_x, float max_image_x, float min_image_z,
                        float max_image_z, uint16_t include_flag) {
  const auto max_distance_squared = max_distance * max_distance;
  const auto min_distance_squared = min_distance * min_distance;

  if (reset_cloud) {
    // Reset
    cloud.clear();
    cloud.header.stamp = start_timestamp;
    cloud.header.frame_id = "cepton3";
    cloud.height = 1;
    cloud.reserve(n_points);
  } else {
    cloud.reserve(cloud.points.size() + n_points);
  }

#ifdef WITH_TS_CH_F
  const auto cloud_start_timestamp = static_cast<int64_t>(cloud.header.stamp);
  cepton_ros::RelativeTimestampAccumulator timestamp_accumulator(
      start_timestamp, cloud_start_timestamp);
#endif

  // Add the points
  for (size_t i = 0; i < n_points; ++i) {
    typename CloudT::PointType cp;
    auto const& p = points[i];

#ifdef WITH_TS_CH_F
    set_point_timestamp<CloudT>(cp, timestamp_accumulator,
                                p.relative_timestamp);
#endif

    // If point has flags that should not be included (specified by the
    // include_flag), continue
    if ((~include_flag & p.flags) != 0) continue;

    // Convert the units to meters (SDK unit is 1.0 / 65536.0)
    float x = static_cast<float>(p.x) / 65536.0;
    float y = static_cast<float>(p.y) / 65536.0;
    float z = static_cast<float>(p.z) / 65536.0;

    // Convert the coordinates to ROS coordinates.
    {
      const auto tx = y;
      const auto ty = -x;
      const auto tz = z;
      x = tx;
      y = ty;
      z = tz;
    }

    const float distance_squared = x * x + y * y + z * z;

    // Filter out points that are labelled ambient but have invalid
    // distance until point flag definitions are finalized (> 500m for
    // now)
    if (distance_squared >= 500 * 500) continue;

    const float tan_yx = y / x;
    const float tan_zx = z / x;

    if (tan_yx < min_image_x || tan_yx > max_image_x || tan_zx < min_image_z ||
        tan_zx > max_image_z || distance_squared < min_distance_squared ||
        distance_squared > max_distance_squared)
      continue;

    cp.x = x;
    cp.y = y;
    cp.z = z;
    cp.intensity = p.reflectivity * 0.01;

#ifdef WITH_TS_CH_F
    cp.channel_id = p.channel_id;
    cp.flags = p.flags;
    cp.valid = !(p.flags & CEPTON_POINT_NO_RETURN);
#endif
#ifdef WITH_POLAR
    const double azimuth_rad = atan(tan_yx);
    const double elevation_rad = atan2(tan_zx, sqrt(tan_yx * tan_yx + 1));
    cp.azimuth = azimuth_rad;
    cp.elevation = elevation_rad;
#endif
    cloud.points.push_back(cp);
  }

  cloud.height = 1;
  cloud.width = cloud.points.size();
}

void PublisherNodelet::publish_points(CeptonSensorHandle handle,
                                      int64_t start_timestamp, size_t n_points,
                                      const CeptonPointEx* points) {
  // Buffer to build the output cloud
  static std::unordered_map<CeptonSensorHandle, cepton_ros::CloudDelta>
      delta_clouds;
  static std::unordered_map<CeptonSensorHandle, cepton_ros::CloudFrameOffset>
      frame_offset_clouds;
  static std::unordered_map<CeptonSensorHandle, cepton_ros::CloudAbsolute>
      absolute_clouds;

  // Store the parity of each cloud. This may be used for 2-frames aggregation
  static std::unordered_map<CeptonSensorHandle, uint8_t> cloud_parity;

  // Update the sensor status
  {
    std::lock_guard<std::mutex> lock(status_lock_);
    last_points_time_[handle] = std::chrono::system_clock::now();
  }

  // Update parity
  if (cloud_parity.find(handle) == cloud_parity.end()) {
    cloud_parity[handle] = 0;
  }
  cloud_parity[handle] = 1 - cloud_parity[handle];

  const auto first = cloud_parity[handle] == 1;

  if (timestamp_mode_ == RelativeTimestampMode::Delta) {
    if (delta_clouds.find(handle) == delta_clouds.end())
      delta_clouds[handle] = cepton_ros::CloudDelta();

    auto& cloud = delta_clouds[handle];
    const bool reset_cloud = first || !aggregate_frames_;
    extend_from_points(cloud, start_timestamp, n_points, points, reset_cloud,
                       min_distance_, max_distance_, min_image_x_, max_image_x_,
                       min_image_z_, max_image_z_, include_flag_);

    if (aggregate_frames_ && first) return;
    if (pub_fut_.valid()) pub_fut_.wait();

    pub_fut_ = std::async(std::launch::async, [this, handle]() {
      auto const& cloud = delta_clouds[handle];

      points_publisher_.publish(cloud);

      if (handle_points_publisher_.count(handle) && output_by_handle_)
        handle_points_publisher_[handle].publish(cloud);

      if (serial_points_publisher_.count(handle) && output_by_sn_)
        serial_points_publisher_[handle].publish(cloud);
    });
    return;
  }

  if (timestamp_mode_ == RelativeTimestampMode::Absolute) {
    if (absolute_clouds.find(handle) == absolute_clouds.end())
      absolute_clouds[handle] = cepton_ros::CloudAbsolute();

    auto& cloud = absolute_clouds[handle];
    const bool reset_cloud = first || !aggregate_frames_;
    extend_from_points(cloud, start_timestamp, n_points, points, reset_cloud,
                       min_distance_, max_distance_, min_image_x_, max_image_x_,
                       min_image_z_, max_image_z_, include_flag_);

    if (aggregate_frames_ && first) return;
#if defined(WITH_TS_CH_F) && defined(WITH_POLAR)
    write_mirror_sync_csv(handle, cloud);
#endif
    if (pub_fut_.valid()) pub_fut_.wait();

    pub_fut_ = std::async(std::launch::async, [this, handle]() {
      auto const& cloud = absolute_clouds[handle];

      points_publisher_.publish(cloud);

      if (handle_points_publisher_.count(handle) && output_by_handle_)
        handle_points_publisher_[handle].publish(cloud);

      if (serial_points_publisher_.count(handle) && output_by_sn_)
        serial_points_publisher_[handle].publish(cloud);
    });
    return;
  }

  if (frame_offset_clouds.find(handle) == frame_offset_clouds.end())
    frame_offset_clouds[handle] = cepton_ros::CloudFrameOffset();

  auto& cloud = frame_offset_clouds[handle];
  const bool reset_cloud = first || !aggregate_frames_;
  extend_from_points(cloud, start_timestamp, n_points, points, reset_cloud,
                     min_distance_, max_distance_, min_image_x_, max_image_x_,
                     min_image_z_, max_image_z_, include_flag_);

  if (aggregate_frames_ && first) return;
  if (pub_fut_.valid()) pub_fut_.wait();

  pub_fut_ = std::async(std::launch::async, [this, handle]() {
    auto const& cloud = frame_offset_clouds[handle];

    points_publisher_.publish(cloud);

    if (handle_points_publisher_.count(handle) && output_by_handle_)
      handle_points_publisher_[handle].publish(cloud);

    if (serial_points_publisher_.count(handle) && output_by_sn_)
      serial_points_publisher_[handle].publish(cloud);
  });
}

#if defined(WITH_TS_CH_F) && defined(WITH_POLAR)
void PublisherNodelet::write_mirror_sync_csv(CeptonSensorHandle handle,
                                             const CloudAbsolute& cloud) {
  if (mirror_sync_csv_path_.empty()) return;

  const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    mirror_sync_start_time_)
          .count();
  const auto window_index =
      static_cast<uint64_t>(elapsed / mirror_sync_interval_sec_);
  if (std::fmod(elapsed, mirror_sync_interval_sec_) >=
      mirror_sync_duration_sec_) {
    std::lock_guard<std::mutex> lock(mirror_sync_csv_lock_);
    if (window_index == mirror_sync_window_index_ && mirror_sync_csv_.is_open())
      mirror_sync_csv_.close();
    return;
  }

  uint32_t sensor_id = 0;
  {
    std::lock_guard<std::mutex> lock(status_lock_);
    const auto it = handle_to_serial_number_.find(handle);
    if (it == handle_to_serial_number_.end()) {
      ROS_WARN_THROTTLE(1.0,
                        "Skipping mirror sync CSV rows until sensor info is "
                        "available for handle %u",
                        handle);
      return;
    }
    sensor_id = it->second;
  }

  std::lock_guard<std::mutex> lock(mirror_sync_csv_lock_);
  if (window_index != mirror_sync_window_index_) {
    mirror_sync_csv_.close();
    mirror_sync_csv_.clear();
    mirror_sync_window_index_ = window_index;
    const auto path =
        make_mirror_sync_csv_path(mirror_sync_csv_path_, window_index + 1);
    mirror_sync_csv_.open(path, std::ios::out | std::ios::trunc);
    if (!mirror_sync_csv_) {
      ROS_ERROR("Failed to open mirror sync CSV: %s", path.c_str());
      return;
    }
    mirror_sync_csv_ << "sensor_id,timestamp_sec,timestamp_nsec,azimuth\n";
    ROS_INFO("mirror sync CSV: %s", path.c_str());
  }
  if (!mirror_sync_csv_) return;

  mirror_sync_csv_ << std::setprecision(9);
  for (const auto& point : cloud.points) {
    if (point.channel_id != 200) continue;
    mirror_sync_csv_ << sensor_id << ',' << point.timestamp_sec << ','
                      << point.timestamp_nsec << ',' << point.azimuth << '\n';
  }
  mirror_sync_csv_.flush();
}
#endif

void PublisherNodelet::publish_sensor_info(const CeptonSensor* info) {
  {
    std::lock_guard<std::mutex> lock(status_lock_);
    handle_to_serial_number_[info->handle] = info->serial_number;
  }

  // Create a points publisher by handle
  if (handle_points_publisher_.find(info->handle) ==
          handle_points_publisher_.end() &&
      output_by_handle_) {
    auto handle_topic_name =
        "cepton3/points_handle_" + std::to_string(info->handle);
    handle_points_publisher_.insert(
        std::pair<CeptonSensorHandle, ros::Publisher>(
            info->handle,
            advertise_points(node_handle_, handle_topic_name, timestamp_mode_,
                             2)));
  }

  // Create a points publisher by serial number
  if (serial_points_publisher_.find(info->handle) ==
          serial_points_publisher_.end() &&
      output_by_sn_) {
    auto sn_topic_name =
        "cepton3/points_sn_" + std::to_string(info->serial_number);
    serial_points_publisher_.insert(
        std::pair<CeptonSensorHandle, ros::Publisher>(
            info->handle,
            advertise_points(node_handle_, sn_topic_name, timestamp_mode_, 2)));
  }

  // Create an info publisher by handle
  if (handle_info_publisher_.find(info->handle) ==
      handle_info_publisher_.end()) {
    auto info_topic_name =
        "cepton3/info_handle_" + std::to_string(info->serial_number);
    handle_info_publisher_.insert(std::pair<CeptonSensorHandle, ros::Publisher>(
        info->handle, node_handle_.advertise<cepton_ros::SensorInformation>(
                          info_topic_name, 2)));
  }

  cepton_ros::SensorInformation msg;
  msg.header.stamp = ros::Time::now();

  msg.handle = info->handle;
  msg.serial_number = info->serial_number;
  msg.model_name = reinterpret_cast<char const*>(info->model_name);
  msg.model = info->model;
  msg.part_number = info->part_number;
  msg.firmware_version = info->firmware_version;
  msg.power_up_timestamp = info->power_up_timestamp;
  msg.time_sync_offset = info->time_sync_offset;
  msg.time_sync_drift = info->time_sync_drift;
  msg.return_count = info->return_count;
  msg.channel_count = info->channel_count;
  msg.status_flags = info->status_flags;
  msg.fault_summary = info->fault_summary;
  std::copy(std::begin(info->fault_entries), std::end(info->fault_entries),
            msg.fault_entries.begin());

  // Publish on the unified info topic
  sensor_info_publisher_.publish(msg);

  // Publish on the sensor-specific topic
  handle_info_publisher_[info->handle].publish(msg);
}

}  // namespace cepton_ros
