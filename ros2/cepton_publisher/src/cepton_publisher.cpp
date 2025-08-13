#include "cepton_publisher.h"

#include <arpa/inet.h>
#include <cepton_sdk3.h>
#include <math.h>
#include <netinet/in.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <mutex>
#include <set>

#include "cepton_messages/cepton_messages.h"

/**
 * Set to 1 to include polar coordinates in the output
 */
#define WITH_POLAR 0

/**
 * Set to 1 to include timestamp, channel id, point flag, in the output
 */
#define WITH_TS_CH_F 0

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using namespace std::chrono_literals;
using namespace std;
namespace cepton_ros {

inline void check_sdk_error(int re, const char *msg) {
  if (re != CEPTON_SUCCESS) {
    cout << "Error: " << re << " " << string(msg) << endl;
  }
}

const float SDK_UNIT_TO_METERS = 1.0 / 65536.0;

inline double degrees_to_radians(double t) { return t * M_PI / 180.0; }

/**
 * SDK callback for point data
 */
void on_ex_frame(CeptonSensorHandle handle, int64_t start_timestamp,
                 size_t n_points, const struct CeptonPointEx *points,
                 void *user_data) {
  auto *node = reinterpret_cast<CeptonPublisher *>(user_data);
  node->publish_points(handle, start_timestamp, n_points, points);
}

/**
 * SDK callback for info data
 */
void on_info(CeptonSensorHandle handle, const struct CeptonSensor *info,
             void *user_data) {
  auto *node = reinterpret_cast<CeptonPublisher *>(user_data);
  node->publish_info(handle, info);
}

void CeptonPublisher::publish_info(CeptonSensorHandle handle,
                                   const struct CeptonSensor *info) {
  // Update the sensor status
  {
    lock_guard<mutex> lock(status_lock_);
    handle_to_serial_number_[handle] = info->serial_number;
  }

  auto msg = cepton_messages::msg::CeptonSensorInfo();
  msg.serial_number = info->serial_number;
  msg.handle = handle;
  msg.model_name = reinterpret_cast<char const *>(info->model_name);
  msg.model = info->model;
  msg.part_number = info->part_number;
  msg.firmware_version = info->firmware_version;
  msg.power_up_timestamp = info->power_up_timestamp;
  msg.time_sync_offset = info->time_sync_offset;
  msg.time_sync_drift = info->time_sync_drift;
  msg.return_count = info->return_count;
  msg.channel_count = info->channel_count;
  msg.status_flags = info->status_flags;
  msg.temperature = info->temperature;
  msg.fault_summary = info->fault_summary;
  copy(begin(info->fault_entries), end(info->fault_entries),
       msg.fault_entries.begin());

  // Make sure this serial number has the needed publishers
  if (use_sn_for_pcl2_)
    ensure_pcl2_publisher(handle, "serial_" + to_string(info->serial_number),
                          serial_points_publisher);

  // Publish info on the unified topic
  info_publisher->publish(msg);

  // Publish info on the per-sensor info topics.
  // Publish on the sensor-specific topic
  ensure_info_publisher(handle, "info_handle_" + to_string(handle),
                        handle_to_info_publisher);
  handle_to_info_publisher[handle]->publish(msg);
}

void CeptonPublisher::publish_points(CeptonSensorHandle handle,
                                     int64_t start_timestamp, size_t n_points,
                                     const CeptonPointEx *points) {
  // Update the sensor status (time when the last points are received).
  // This is used for monitoring sensor timeout.
  {
    lock_guard<mutex> lock(status_lock_);
    last_points_time_[handle] = chrono::system_clock::now();
  }

  // Store buffers for PointCloud2 point clouds (output)
  static unordered_map<CeptonSensorHandle, PointCloud2> clouds;

  // Make sure the buffer is ready to be written (publish not in progress)
  if (pub_fut_.valid()) pub_fut_.wait();

  // Write the cloud into our buffer for publishing.
  // This is okay to write here, because publish_points gets called from a
  // single SDK thread so we don't need to lock a buffer, we just need to make
  // sure that the cloud is not currently being published.

  // Create a buffer for this sensor's points, if needed
  if (clouds.find(handle) == clouds.end()) clouds[handle] = PointCloud2();

  auto &cloud = clouds[handle];
  cloud.header.stamp.sec = start_timestamp / 1'000'000;
  // nanosec is the timestamp portion that is truncated from the sec.
  // portion.
  cloud.header.stamp.nanosec = (start_timestamp % 1'000'000) * 1'000;
  cloud.header.frame_id = "cepton3";

  sensor_msgs::PointCloud2Modifier cloud_modifier(cloud);

  int n_fields = 4;
#if WITH_TS_CH_F
  n_fields += 4;
#endif
#if WITH_POLAR
  n_fields += 3;
#endif

  cloud_modifier.setPointCloud2Fields(
      n_fields,
      // clang-format off
              "x", 1, PointField::FLOAT32,
              "y", 1, PointField::FLOAT32,
              "z", 1, PointField::FLOAT32,
              "intensity", 1, PointField::FLOAT32
              #if WITH_TS_CH_F
              ,
              "timestamp_s", 1, PointField::INT32,
              "timestamp_us", 1, PointField::INT32,
              "flags", 1, PointField::UINT16,
              "channel_id", 1, PointField::UINT16
              #endif
              #if WITH_POLAR
              ,
              "azimuth", 1, PointField::FLOAT32,
              "elevation", 1, PointField::FLOAT32,
              "range", 1, PointField::FLOAT32
              #endif
      // clang-format on
  );

  // resizing should be done before the iters are declared, otw seg faults
  cloud_modifier.resize(n_points);

  // Populate the cloud fields
  // Each iterator is optional, and depends on whether the field was added
  // to the cloud.
  auto x_iter = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
  auto y_iter = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
  auto z_iter = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
  auto intensity_iter =
      sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");

#if WITH_TS_CH_F
  auto timestamp_sec_iter =
      sensor_msgs::PointCloud2Iterator<int32_t>(cloud, "timestamp_s");
  auto timestamp_usec_iter =
      sensor_msgs::PointCloud2Iterator<int32_t>(cloud, "timestamp_us");
  auto flag_iter = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "flags");
  auto channel_id_iter =
      sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "channel_id");
#endif

#if WITH_POLAR
  auto azim_iter = sensor_msgs::PointCloud2Iterator<float>(cloud, "azimuth");
  auto elev_iter = sensor_msgs::PointCloud2Iterator<float>(cloud, "elevation");
  auto dist_iter = sensor_msgs::PointCloud2Iterator<float>(cloud, "range");
#endif

  int kept = 0;

  auto const min_distance_squared = min_distance_ * min_distance_;
  auto const max_distance_squared = max_distance_ * max_distance_;

  auto timestamp = start_timestamp;

  for (unsigned i = 0; i < n_points; i++) {
    auto const &p0 = points[i];

    timestamp += p0.relative_timestamp;

    // If point has flags that should not be included (specified by the
    // include_flag), continue
    if ((~(include_flag_) & (p0.flags)) != 0) continue;

    float x = p0.x * SDK_UNIT_TO_METERS;
    float y = p0.y * SDK_UNIT_TO_METERS;
    float z = p0.z * SDK_UNIT_TO_METERS;

    // Convert cepton coordinates to ROS coordinates
    {
      auto tx = y;
      auto ty = -x;
      auto tz = z;
      x = tx;
      y = ty;
      z = tz;
    }

    const float distance_squared = x * x + y * y + z * z;

    // Filter out points that are labelled ambient but have invalid
    // distance until point flag definitions are finalized (> 500m for
    // now)
    if (distance_squared >= 500 * 500) continue;

    const float image_x = y / x;  // horizontal tangent
    const float image_z = z / x;  // vertical tangent

    // Filter if the point is outside of the FOV
    if (image_x < min_image_x_ || image_x > max_image_x_ ||
        image_z < min_image_z_ || image_z > max_image_z_ ||
        distance_squared < min_distance_squared ||
        distance_squared > max_distance_squared)
      continue;

    // x value
    *x_iter = x;
    ++x_iter;

    // y value
    *y_iter = y;
    ++y_iter;

    // z value
    *z_iter = z;
    ++z_iter;

    // intensity
    *intensity_iter = p0.reflectivity * 0.01;
    ++intensity_iter;

#if WITH_TS_CH_F
    // timestamp - seconds
    *timestamp_sec_iter = timestamp / (int64_t)1e6;
    ++timestamp_sec_iter;

    // timestamp - microseconds
    *timestamp_usec_iter = timestamp % (int64_t)1e6;
    ++timestamp_usec_iter;

    // flags
    *flag_iter = p0.flags;
    ++flag_iter;

    // channel id
    *channel_id_iter = p0.channel_id;
    ++channel_id_iter;
#endif

#if WITH_POLAR
    const double azimuth_rad = atan(image_x);
    const double elevation_rad = atan2(image_z, sqrt(image_x * image_x + 1));

    // If this point is a no-return, set the distance to 0
    bool const is_valid_return = (p0.flags & CEPTON_POINT_NO_RETURN) == 0;
    float range_meas = is_valid_return ? sqrt(x * x + y * y + z * z) : 0.0;

    // azimuth
    *azim_iter = azimuth_rad;
    ++azim_iter;

    // elevation
    *elev_iter = elevation_rad;
    ++elev_iter;

    // distance
    *dist_iter = range_meas;
    ++dist_iter;
#endif

    kept++;
  }

  // Set the final dimension of the cloud
  cloud_modifier.resize(kept);
  cloud.width = kept;
  cloud.height = 1;

  // Publish. Creating the publish future means we can no longer write buffers
  // until the publish completes.
  {
    pub_fut_ = std::async(
        std::launch::async, [this, start_timestamp, n_points, handle, cloud]() {
          // Publish points
          points_publisher->publish(cloud);

          // Publish by handle - create publisher if needed
          if (use_handle_for_pcl2_) {
            ensure_pcl2_publisher(handle, "handle_" + to_string(handle),
                                  handle_points_publisher);
            handle_points_publisher[handle]->publish(cloud);
          }

          // If info packet is received to link handle to serial number, publish
          // by serial number
          if (serial_points_publisher.count(handle) && use_sn_for_pcl2_)
            serial_points_publisher[handle]->publish(cloud);
        });
  }
}

/// @brief Create a pcl2 publisher and store in the provided map, if not
/// already existing
/// @param topic
/// @param m
void CeptonPublisher::ensure_pcl2_publisher(CeptonSensorHandle handle,
                                            string const &topic,
                                            PointPublisherMap &m) {
  if (!m.count(handle)) {
    RCLCPP_INFO(this->get_logger(), "Create point cloud publisher for %lu",
                handle);
    m[handle] = create_publisher<PointCloud2>(topic, 50);
  }
}

void CeptonPublisher::ensure_info_publisher(CeptonSensorHandle handle,
                                            string const &topic,
                                            InfoPublisherMap &m) {
  if (!m.count(handle)) {
    RCLCPP_INFO(this->get_logger(), "Create info publisher");
    m[handle] =
        create_publisher<cepton_messages::msg::CeptonSensorInfo>(topic, 10);
  }
}

CeptonPublisher::CeptonPublisher() : Node("cepton_publisher") {
  declare_parameter("capture_file", "");
  declare_parameter("capture_loop", false);
  declare_parameter("sensor_port", 8808);

  // Describe how to output point clouds (per-sensor topics).
  // Can be per-handle, per-serialnum, or both
  declare_parameter("pcl2_output_type",
                    "BOTH");  // "NONE" or "IP" or "SN" or "BOTH"

  declare_parameter("include_saturated_points", true);
  declare_parameter("include_second_return_points", true);
  declare_parameter("include_invalid_points", false);
  declare_parameter("include_noise_points", false);
  declare_parameter("include_blocked_points", true);
  declare_parameter("min_altitude", -90.);
  declare_parameter("max_altitude", 90.);
  declare_parameter("min_azimuth", -90.);
  declare_parameter("max_azimuth", 90.);
  declare_parameter("max_distance", numeric_limits<float>::max());
  declare_parameter("min_distance", 0.0);
  declare_parameter("expected_sensor_ips", vector<string>{});

  // Initialize sdk
  int ret = CeptonInitialize(CEPTON_API_VERSION, nullptr);
  check_sdk_error(ret, "CeptonInitialize");

  RCLCPP_DEBUG(this->get_logger(),
               "========= Point Cloud Output Parameters =========");

  // Check whether to output pcl2 points
  rclcpp::Parameter pOutputPcl2 = get_parameter("pcl2_output_type");
  std::string pcl2_output_type = pOutputPcl2.as_string();
  if (pcl2_output_type == "NONE") {
    use_handle_for_pcl2_ = use_sn_for_pcl2_ = false;
    RCLCPP_DEBUG(this->get_logger(), "Not publishing PCL2 points!");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Publishing PCL2 points with: %s",
                 pcl2_output_type.c_str());

    use_handle_for_pcl2_ =
        pcl2_output_type == "IP" || pcl2_output_type == "BOTH";
    RCLCPP_DEBUG(this->get_logger(), "\tPublishing PCL2 points with handle: %s",
                 use_handle_for_pcl2_ ? "true" : "false");

    use_sn_for_pcl2_ = pcl2_output_type == "SN" || pcl2_output_type == "BOTH";
    RCLCPP_DEBUG(this->get_logger(), "\tPublishing PCL2 points with SN: %s",
                 use_sn_for_pcl2_ ? "true" : "false");
    points_publisher = create_publisher<PointCloud2>("cepton_pcl2", 50);

    // Register callback
    ret = CeptonListenFramesEx(CEPTON_AGGREGATION_MODE_NATURAL, on_ex_frame,
                               this);
    check_sdk_error(ret, "CeptonListenFramesEx");
  }
  RCLCPP_DEBUG(this->get_logger(),
               "=================================================");

  // ---------------------------------
  // Set up the sensor info publishing
  {
    // Create publisher
    RCLCPP_INFO(this->get_logger(), "Creating info publisher");
    info_publisher = create_publisher<cepton_messages::msg::CeptonSensorInfo>(
        "cepton_info", 10);
    // Register callback
    ret = CeptonListenSensorInfo(on_info, this);
    check_sdk_error(ret, "CeptonListenSensorInfo");
  }

  // -----------------------------------
  // Set up the sensor status publishing
  {
    RCLCPP_INFO(this->get_logger(), "Creating status publisher");
    sensor_status_publisher =
        create_publisher<cepton_messages::msg::CeptonSensorStatus>(
            "cepton_sensor_status", 5);

    // Start the status monitor
    sensor_status_thread = thread([&]() {
      while (!stopping_) {
        // Store the handle and serial number of each timed out sensor.
        // Use 0 if no serial number has been found.
        unordered_map<CeptonSensorHandle, uint32_t> timed_out_sensors;
        {
          lock_guard<mutex> lock(status_lock_);
          // Update status messages for each sensor
          auto now = chrono::system_clock::now();

          for (auto const &[handle, last_points_time] : last_points_time_) {
            if (now - last_points_time > SENSOR_POINTS_TIMEOUT) {
              timed_out_sensors[handle] = handle_to_serial_number_.count(handle)
                                              ? handle_to_serial_number_[handle]
                                              : 0;
            }
          }
        }
        for (auto const &[handle, serial_number] : timed_out_sensors) {
          auto msg = cepton_messages::msg::CeptonSensorStatus();
          msg.serial_number = serial_number;
          msg.handle = handle;
          msg.status = SENSOR_TIMED_OUT;
          sensor_status_publisher->publish(msg);
        }
        this_thread::sleep_for(chrono::seconds(1));
      }
    });
  }

  RCLCPP_DEBUG(this->get_logger(),
               "============= Point Flag Parameters =============");

  // Check whether to keep flagged points
  rclcpp::Parameter pKeepSaturated = get_parameter("include_saturated_points");
  include_flag_ |= (pKeepSaturated.as_bool() ? CEPTON_POINT_SATURATED : 0);
  RCLCPP_DEBUG(this->get_logger(), "Including Saturated points: %s",
               pKeepSaturated.as_bool() ? "true" : "false");
  // keep frame low_snr/parity/boundary as internal for now

  rclcpp::Parameter pKeepSecondReturn =
      get_parameter("include_second_return_points");
  include_flag_ |=
      (pKeepSecondReturn.as_bool() ? CEPTON_POINT_SECOND_RETURN : 0);
  RCLCPP_DEBUG(this->get_logger(), "Including Second Return points: %s",
               pKeepSecondReturn.as_bool() ? "true" : "false");

  rclcpp::Parameter pKeepInvalid = get_parameter("include_invalid_points");
  include_flag_ |= (pKeepInvalid.as_bool() ? CEPTON_POINT_NO_RETURN : 0);
  RCLCPP_DEBUG(this->get_logger(), "Including Invalid (No Return) points: %s",
               pKeepInvalid.as_bool() ? "true" : "false");

  rclcpp::Parameter pKeepNoise = get_parameter("include_noise_points");
  include_flag_ |= (pKeepNoise.as_bool() ? CEPTON_POINT_NOISE : 0);
  RCLCPP_DEBUG(this->get_logger(), "Including Noise points: %s",
               pKeepNoise.as_bool() ? "true" : "false");

  rclcpp::Parameter pKeepBlocked = get_parameter("include_blocked_points");
  include_flag_ |= (pKeepBlocked.as_bool() ? CEPTON_POINT_BLOCKED : 0);
  RCLCPP_DEBUG(this->get_logger(), "Including Blocked points: %s",
               pKeepBlocked.as_bool() ? "true" : "false");

  RCLCPP_DEBUG(this->get_logger(),
               "=================================================");

  // Point filter settings
  min_altitude_ = get_parameter("min_altitude").as_double();
  max_altitude_ = get_parameter("max_altitude").as_double();
  min_azimuth_ = get_parameter("min_azimuth").as_double();
  max_azimuth_ = get_parameter("max_azimuth").as_double();

  // Clip so that we can safely take the tangent
  min_azimuth_ = max(-89.9, min_azimuth_);
  max_azimuth_ = min(89.9, max_azimuth_);
  min_altitude_ = max(-89.9, min_altitude_);
  max_altitude_ = min(89.9, max_altitude_);

  min_image_x_ = tan(degrees_to_radians(min_azimuth_));
  max_image_x_ = tan(degrees_to_radians(max_azimuth_));
  min_image_z_ = tan(degrees_to_radians(min_altitude_));
  max_image_z_ = tan(degrees_to_radians(max_altitude_));

  max_distance_ = get_parameter("max_distance").as_double();
  min_distance_ = get_parameter("min_distance").as_double();

  // Load a pcap for replay, or start networking for a live sensor
  rclcpp::Parameter captureFile = get_parameter("capture_file");
  if (!captureFile.as_string().empty()) {
    // If running replay, check for flags
    rclcpp::Parameter capturePlayLoop = get_parameter("capture_loop");
    int flag = 0;
    if (capturePlayLoop.get_type() !=
            rclcpp::ParameterType::PARAMETER_NOT_SET &&
        capturePlayLoop.as_bool()) {
      flag |= CEPTON_REPLAY_FLAG_PLAY_LOOPED;
    }
    ret = CeptonReplayLoadPcap(captureFile.as_string().c_str(), flag,
                               &replay_handle);
    check_sdk_error(ret, "CeptonReplayLoadPcap");
  } else {
    // Start listening for UDP data on the specified port. Default port is
    // 8808
    rclcpp::Parameter port = get_parameter("sensor_port");
    int p = 8808;
    if (port.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
      p = port.as_int();
    RCLCPP_DEBUG(this->get_logger(), "Start networking on %d", p);
    CeptonStartNetworkingOnPort(p);
  }

  // Init the expected sensors
  auto p_expected_sensor_ips = get_parameter("expected_sensor_ips");

  // If expected IPs are set, then for each handle, set the last points time
  // to be the current time. The sensor will then time out if we don't get
  // points within the limit. By default, sensors would be discovered lazily.
  // This lets us raise an error message if an expected sensor was not connected
  // properly to the PC.
  if (p_expected_sensor_ips.get_type() !=
      rclcpp::ParameterType::PARAMETER_NOT_SET) {
    RCLCPP_DEBUG(this->get_logger(), "Set expected sensor IPs");
    auto expected_sensor_ips = p_expected_sensor_ips.as_string_array();
    for (auto const &expected_ip : expected_sensor_ips) {
      struct in_addr addr;
      inet_aton(expected_ip.c_str(), &addr);
      // handle is in big-endian. inet_aton returns little endian
      last_points_time_[__bswap_32(addr.s_addr)] = chrono::system_clock::now();
    }
  }
  RCLCPP_DEBUG(this->get_logger(), "Finished constructing");
}

CeptonPublisher::~CeptonPublisher() {
  // Stop the status thread and wait for it to join
  stopping_ = true;
  if (sensor_status_thread.joinable()) sensor_status_thread.join();

  // Tear down the sdk
  CeptonDeinitialize();
}

}  // namespace cepton_ros
