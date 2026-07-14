#include "publisher_nodelet.hpp"

#include <arpa/inet.h>
#include <cmath>
#include <netinet/in.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <string>
#include <utility>
#include <vector>

PLUGINLIB_EXPORT_CLASS(cepton_ros::PublisherNodelet, nodelet::Nodelet);

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr uint16_t kCeptonPointAmbient = 1U << 15;
constexpr char kFrameId[] = "cepton3";
const auto kSensorPointsTimeout = std::chrono::seconds(3);

double degrees_to_radians(double degrees) { return degrees * kPi / 180.0; }

const char* sdk_ip(const std::string& ip) {
  return (ip.empty() || ip == "0.0.0.0") ? nullptr : ip.c_str();
}

const char* sdk_multicast_group(const std::string& multicast_group) {
  return multicast_group.empty() ? nullptr : multicast_group.c_str();
}
}  // namespace

namespace cepton_ros {

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
void on_sensor_info(CeptonSensorHandle, const struct CeptonSensor* info,
                    void* user_data) {
  reinterpret_cast<PublisherNodelet*>(user_data)->publish_sensor_info(info);
}

PublisherNodelet::~PublisherNodelet() {
  sensor_status_timer_.stop();
  watchdog_timer_.stop();

  // Stop callbacks before releasing buffers and publishers.
  CeptonUnlistenFramesEx(on_ex_frame, this);
  CeptonUnlistenSensorInfo(on_sensor_info, this);

  // Remove all networking sources
  for (const auto& source : networking_sources_) {
    const int ret = CeptonRemoveNetworkingSource(
        sdk_ip(source.ip), source.port,
        sdk_multicast_group(source.multicast_group));
    if (ret != CEPTON_SUCCESS) {
      NODELET_ERROR("[%s] cleanup API error: %s -> %s", getName().c_str(),
                    "CeptonRemoveNetworkingSource",
                    CeptonGetErrorCodeName(ret));
    }
  }

  const int ret = CeptonDeinitialize();
  if (ret != CEPTON_SUCCESS) {
    NODELET_ERROR("[%s] cleanup API error: %s -> %s", getName().c_str(),
                  "CeptonDeinitialize", CeptonGetErrorCodeName(ret));
  }
}

void PublisherNodelet::check_api_error(int err, char const* api) {
  if (err != CEPTON_SUCCESS) {
    NODELET_ERROR("[%s] API error: %s -> %s", getName().c_str(), api,
                  CeptonGetErrorCodeName(err));
    exit(1);
  }
}

void PublisherNodelet::onInit() {
  ROS_INFO("\n\n========== Version: %s ==========\n ", VERSION.c_str());
  ROS_INFO("Publisher Nodelet Started");

  node_handle_ = getNodeHandle();
  private_node_handle_ = getPrivateNodeHandle();

  std::string capture_path;
  bool capture_loop = true;
  std::vector<std::string> sensor_network_sources;
  load_parameters(capture_path, capture_loop, sensor_network_sources);
  configure_point_flags();
  initialize_publishers();
  initialize_sdk();
  configure_input(capture_path, capture_loop, sensor_network_sources);
  register_sdk_callbacks();
  start_replay_watchdog(capture_path);
  start_status_monitor();
  register_expected_sensors();
}

void PublisherNodelet::load_parameters(
    std::string& capture_path, bool& capture_loop,
    std::vector<std::string>& sensor_network_sources) {
  private_node_handle_.param("capture_path", capture_path, capture_path);
  private_node_handle_.param("capture_loop", capture_loop, capture_loop);

  private_node_handle_.param("min_altitude", min_altitude_, min_altitude_);
  private_node_handle_.param("max_altitude", max_altitude_, max_altitude_);
  private_node_handle_.param("min_azimuth", min_azimuth_, min_azimuth_);
  private_node_handle_.param("max_azimuth", max_azimuth_, max_azimuth_);

  // Clip so that we can safely take the tangent
  min_azimuth_ = std::max(-89.9, std::min(89.9, min_azimuth_));
  max_azimuth_ = std::max(-89.9, std::min(89.9, max_azimuth_));
  min_altitude_ = std::max(-89.9, std::min(89.9, min_altitude_));
  max_altitude_ = std::max(-89.9, std::min(89.9, max_altitude_));
  if (min_azimuth_ > max_azimuth_) std::swap(min_azimuth_, max_azimuth_);
  if (min_altitude_ > max_altitude_) {
    std::swap(min_altitude_, max_altitude_);
  }

  point_filter_.min_image_x = std::tan(degrees_to_radians(min_azimuth_));
  point_filter_.max_image_x = std::tan(degrees_to_radians(max_azimuth_));
  point_filter_.min_image_z = std::tan(degrees_to_radians(min_altitude_));
  point_filter_.max_image_z = std::tan(degrees_to_radians(max_altitude_));

  private_node_handle_.param("max_distance", max_distance_, max_distance_);
  private_node_handle_.param("min_distance", min_distance_, min_distance_);
  min_distance_ = std::max(0.0F, min_distance_);
  max_distance_ = std::max(min_distance_, max_distance_);

  private_node_handle_.param("output_by_handle", output_by_handle_,
                             output_by_handle_);
  private_node_handle_.param("output_by_sn", output_by_sn_, output_by_sn_);

  private_node_handle_.param("sensor_network_sources", sensor_network_sources,
                             sensor_network_sources);

  private_node_handle_.param("expected_sensor_ips", expected_sensor_ips_,
                             expected_sensor_ips_);

  private_node_handle_.param("aggregate_frames", aggregate_frames_,
                             aggregate_frames_);

  point_filter_.min_distance_squared = min_distance_ * min_distance_;
  point_filter_.max_distance_squared = max_distance_ * max_distance_;
}

void PublisherNodelet::configure_point_flags() {
  bool include = true;

  ROS_INFO("============= Point Flag Parameters =============\n");

  private_node_handle_.param("include_saturated_points", include, true);
  include_flag_ |= (include ? CEPTON_POINT_SATURATED : 0);
  ROS_INFO("Including Saturated points: %s\n", include ? "true" : "false");

  // keep frame low_snr/parity/boundary as internal for now

  private_node_handle_.param("include_second_return_points", include, true);
  include_flag_ |= (include ? CEPTON_POINT_SECOND_RETURN : 0);
  ROS_INFO("Including Second Return points: %s\n",
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

  private_node_handle_.param("include_ambient_points", include, false);
  include_flag_ |= (include ? kCeptonPointAmbient : 0);
  ROS_INFO("Including Ambient points: %s\n", include ? "true" : "false");

  ROS_INFO("=================================================\n");
  point_filter_.include_flags = include_flag_;
}

void PublisherNodelet::initialize_publishers() {
  sensor_info_publisher_ =
      node_handle_.advertise<cepton_ros::SensorInformation>(
          "cepton3/sensor_information", 2);

  // All points
  points_publisher_ =
      node_handle_.advertise<cepton_ros::Cloud>("cepton3/points", 50);

  sensor_status_publisher_ =
      node_handle_.advertise<cepton_ros::CeptonSensorStatus>(
          "cepton3/cepton_sensor_status", 2);
}

void PublisherNodelet::initialize_sdk() {
  int ret = CeptonInitialize(CEPTON_API_VERSION, nullptr);
  check_api_error(ret, "CeptonInitialize");

  uint32_t control_flags = 0;

  if (include_flag_ & kCeptonPointAmbient) {
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
}

void PublisherNodelet::configure_input(
    const std::string& capture_path, bool capture_loop,
    const std::vector<std::string>& sensor_network_sources) {
  const uint32_t replay_flags =
      capture_loop ? CEPTON_REPLAY_FLAG_PLAY_LOOPED : 0;
  if (!capture_path.empty()) {
    const int ret = CeptonReplayLoadPcap(capture_path.c_str(), replay_flags,
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
      if (!add_networking_source({"0.0.0.0", 8808, ""})) {
        check_api_error(CEPTON_ERROR_GENERIC, "CeptonAddNetworkingSource");
      }
    } else {
      // Add a networking source for each configured source string
      for (const auto& source_str : sensor_network_sources) {
        NetworkSource source;
        if (!parse_network_source(source_str, source)) {
          ROS_ERROR("Failed to parse network source: %s", source_str.c_str());
          continue;
        }

        if (!add_networking_source(source)) {
          NODELET_ERROR(
              "Source not added successfully, check that sensor is configured "
              "properly and running: %s",
              source_str.c_str());
          continue;
        }
      }
    }
  }
}

void PublisherNodelet::register_sdk_callbacks() {
  int ret =
      CeptonListenFramesEx(CEPTON_AGGREGATION_MODE_NATURAL, on_ex_frame, this);
  check_api_error(ret, "CeptonListenFrames");

  // Listen for sensor info
  ret = CeptonListenSensorInfo(on_sensor_info, this);
  check_api_error(ret, "CeptonListenSensorInfo");
}

void PublisherNodelet::start_replay_watchdog(
    const std::string& capture_path) {
  if (!capture_path.empty()) {
    watchdog_timer_ = node_handle_.createTimer(
        ros::Duration(0.1), [this](const ros::TimerEvent&) {
          // Shut down if a replay was loaded, and if the replay is finished
          if (replay_handle_ != 0 &&
              CeptonReplayIsFinished(replay_handle_) == 1) {
            NODELET_INFO("[%s] capture replay done", getName().c_str());
            int ret = CeptonReplayUnloadPcap(replay_handle_);
            check_api_error(ret, "CeptonReplayUnload");
            replay_handle_ = 0;

            ros::shutdown();
          }
        });
  }
}

void PublisherNodelet::start_status_monitor() {
  sensor_status_timer_ = node_handle_.createTimer(
      ros::Duration(1.0), &PublisherNodelet::report_timed_out_sensors, this);
}

void PublisherNodelet::register_expected_sensors() {
  std::lock_guard<std::mutex> lock(status_lock_);
  for (auto const& expected_ip : expected_sensor_ips_) {
    ROS_INFO("Adding expected IP %s", expected_ip.c_str());
    struct in_addr addr {};
    if (inet_aton(expected_ip.c_str(), &addr) == 0) {
      NODELET_ERROR("Invalid expected sensor IP: %s", expected_ip.c_str());
      continue;
    }
    last_points_time_[ntohl(addr.s_addr)] =
        std::chrono::system_clock::now();
  }
}

bool PublisherNodelet::parse_network_source(const std::string& source,
                                            NetworkSource& result) {
  const auto first_colon = source.find(':');
  if (first_colon == std::string::npos) return false;

  const auto second_colon = source.find(':', first_colon + 1);
  if (second_colon != std::string::npos &&
      source.find(':', second_colon + 1) != std::string::npos) {
    return false;
  }

  result.ip = source.substr(0, first_colon);
  const auto port_text = source.substr(
      first_colon + 1,
      second_colon == std::string::npos ? std::string::npos
                                        : second_colon - first_colon - 1);
  result.multicast_group = second_colon == std::string::npos
                               ? std::string{}
                               : source.substr(second_colon + 1);

  try {
    size_t parsed_length = 0;
    const unsigned long port = std::stoul(port_text, &parsed_length);
    if (parsed_length != port_text.size() || port == 0 || port > 65535) {
      return false;
    }
    result.port = static_cast<uint16_t>(port);
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

bool PublisherNodelet::add_networking_source(const NetworkSource& source) {
  const char* ip = sdk_ip(source.ip);
  const char* multicast_group = sdk_multicast_group(source.multicast_group);
  ROS_INFO("Add networking source: ip=%s, port=%u, multicast=%s",
           ip ? ip : "0.0.0.0", static_cast<unsigned>(source.port),
           multicast_group ? multicast_group : "none");

  const int ret = CeptonAddNetworkingSource(ip, source.port, multicast_group);
  if (ret != CEPTON_SUCCESS) return false;
  networking_sources_.push_back(source);
  return true;
}

void PublisherNodelet::report_timed_out_sensors(const ros::TimerEvent&) {
  std::vector<std::pair<CeptonSensorHandle, uint32_t>> timed_out_sensors;
  {
    std::lock_guard<std::mutex> lock(status_lock_);
    const auto now = std::chrono::system_clock::now();
    for (const auto& entry : last_points_time_) {
      if (now - entry.second <= kSensorPointsTimeout) continue;
      const auto serial = handle_to_serial_number_.find(entry.first);
      timed_out_sensors.emplace_back(
          entry.first, serial == handle_to_serial_number_.end() ? 0
                                                                : serial->second);
    }
  }

  for (const auto& sensor : timed_out_sensors) {
    cepton_ros::CeptonSensorStatus msg;
    msg.handle = sensor.first;
    msg.serial_number = sensor.second;
    msg.status = SENSOR_TIMED_OUT;
    sensor_status_publisher_.publish(msg);
  }
}

void PublisherNodelet::extend_from_points(
    cepton_ros::Cloud& cloud, int64_t start_timestamp, size_t n_points,
    const CeptonPointEx* points, bool first) const {

  if (first) {
    // Reset
    cloud.clear();
    cloud.header.stamp = start_timestamp;
    cloud.header.frame_id = kFrameId;
    cloud.height = 1;
    cloud.width = n_points;
    cloud.reserve(n_points);
  } else {
    cloud.reserve(cloud.points.size() + n_points);
  }

  // Add the points
  for (size_t i = 0; i < n_points; ++i) {
    cepton_ros::Point cp;
    auto const& p = points[i];

    // If point has flags that should not be included (specified by the
    // include_flag), continue
    if ((~point_filter_.include_flags & p.flags) != 0) continue;

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
    // if (distance_squared >= 500 * 500) continue;

    if (x == 0.0F) continue;
    const float tan_yx = y / x;
    const float tan_zx = z / x;

    if (tan_yx < point_filter_.min_image_x ||
        tan_yx > point_filter_.max_image_x ||
        tan_zx < point_filter_.min_image_z ||
        tan_zx > point_filter_.max_image_z ||
        distance_squared < point_filter_.min_distance_squared ||
        distance_squared > point_filter_.max_distance_squared)
      continue;

    cp.x = x;
    cp.y = y;
    cp.z = z;
    cp.intensity = p.reflectivity * 0.01;

#ifdef WITH_TS_CH_F
    cp.relative_timestamp = p.relative_timestamp;
    cp.channel_id = p.channel_id;
    cp.flags = p.flags;
    cp.valid = !(p.flags & CEPTON_POINT_NO_RETURN);
#endif
#ifdef WITH_POLAR
    const double azimuth_rad = std::atan(tan_yx);
    const double elevation_rad =
        std::atan2(tan_zx, std::sqrt(tan_yx * tan_yx + 1));
    cp.azimuth = azimuth_rad;
    cp.elevation = elevation_rad;
#endif
    cloud.points.push_back(cp);
  }
  cloud.width = cloud.points.size();
}

void PublisherNodelet::publish_points(CeptonSensorHandle handle,
                                      int64_t start_timestamp, size_t n_points,
                                      const CeptonPointEx* points) {
  {
    std::lock_guard<std::mutex> lock(status_lock_);
    last_points_time_[handle] = std::chrono::system_clock::now();
  }

  cepton_ros::Cloud cloud_to_publish;
  ros::Publisher handle_publisher;
  ros::Publisher serial_publisher;
  {
    std::lock_guard<std::mutex> lock(sensor_data_lock_);
    auto& state = sensor_clouds_[handle];
    const bool first = aggregate_frames_ ? state.first_frame : true;
    if (aggregate_frames_) state.first_frame = !state.first_frame;
    extend_from_points(state.cloud, start_timestamp, n_points, points, first);
    if (aggregate_frames_ && first) return;

    cloud_to_publish = state.cloud;
    const auto handle_it = handle_points_publisher_.find(handle);
    if (output_by_handle_ && handle_it != handle_points_publisher_.end()) {
      handle_publisher = handle_it->second;
    }
    const auto serial_it = serial_points_publisher_.find(handle);
    if (output_by_sn_ && serial_it != serial_points_publisher_.end()) {
      serial_publisher = serial_it->second;
    }
  }

  points_publisher_.publish(cloud_to_publish);
  if (handle_publisher) handle_publisher.publish(cloud_to_publish);
  if (serial_publisher) serial_publisher.publish(cloud_to_publish);
}

void PublisherNodelet::publish_sensor_info(const CeptonSensor* info) {
  {
    std::lock_guard<std::mutex> lock(status_lock_);
    handle_to_serial_number_[info->handle] = info->serial_number;
  }

  ros::Publisher info_publisher;
  {
    std::lock_guard<std::mutex> lock(sensor_data_lock_);
    if (output_by_handle_ && !handle_points_publisher_.count(info->handle)) {
      handle_points_publisher_.emplace(
          info->handle, node_handle_.advertise<cepton_ros::Cloud>(
                            "cepton3/points_handle_" +
                                std::to_string(info->handle),
                            2));
    }
    if (output_by_sn_ && !serial_points_publisher_.count(info->handle)) {
      serial_points_publisher_.emplace(
          info->handle, node_handle_.advertise<cepton_ros::Cloud>(
                            "cepton3/points_sn_" +
                                std::to_string(info->serial_number),
                            2));
    }
    auto info_it = handle_info_publisher_.find(info->handle);
    if (info_it == handle_info_publisher_.end()) {
      info_it = handle_info_publisher_
                    .emplace(info->handle,
                             node_handle_
                                 .advertise<cepton_ros::SensorInformation>(
                                     "cepton3/info_handle_" +
                                         std::to_string(info->serial_number),
                                     2))
                    .first;
    }
    info_publisher = info_it->second;
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
  info_publisher.publish(msg);
}

}  // namespace cepton_ros
