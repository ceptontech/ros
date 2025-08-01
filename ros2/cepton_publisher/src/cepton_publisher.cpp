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

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using CeptonPoints = cepton_messages::msg::CeptonPointData;
using namespace std::chrono_literals;
using namespace std;
namespace cepton_ros {
inline void check_sdk_error(int re, const char *msg) {
  if (re != CEPTON_SUCCESS) {
    cout << "Error: " << re << " " << string(msg) << endl;
  }
}

double degrees_to_radians(double t) { return t * M_PI / 180.0; }

static unordered_map<CeptonSensorHandle, cepton_messages::msg::CeptonPointData>
    cepton_clouds_;
static unordered_map<CeptonSensorHandle, PointCloud2> sensor_clouds_;

/**
 * @brief Internal points callback invoked by SDK. Publishes points in format
 * sensor_msgs::PointCloud2
 *
 * @param handle
 * @param start_timestamp
 * @param n_points
 * @param stride
 * @param points
 * @param node A pointer to the CeptonPublisher
 */
void ceptonFrameCallback(CeptonSensorHandle handle, int64_t start_timestamp,
                         size_t n_points, const struct CeptonPointEx *points,
                         void *user_data) {
  auto *node = reinterpret_cast<CeptonPublisher *>(user_data);

  // Update the sensor status
  {
    lock_guard<mutex> lock(node->status_lock_);
    node->last_points_time_[handle] = chrono::system_clock::now();
  }

  if (cepton_clouds_.count(handle) == 0) {
    cepton_clouds_[handle] = cepton_messages::msg::CeptonPointData();
  }

  auto &cpts = cepton_clouds_[handle];
  cpts.set__handle(handle);
  cpts.set__n_points(n_points);
  cpts.set__start_timestamp(start_timestamp);

  cpts.points.resize(n_points * sizeof(CeptonPointEx));
  memcpy(cpts.points.data(), points, n_points * sizeof(CeptonPointEx));

  // Publish all points
  node->cep_points_publisher->publish(cpts);

  // Publish by handle - create publisher if not existing
  if (node->use_handle_for_cepp()) {
    node->ensure_cepp_publisher(handle, "cepp_handle_" + to_string(handle),
                                node->handle_cep_points_publisher);
    node->handle_cep_points_publisher[handle]->publish(cpts);
  }

  // Publish by serial number
  if (node->serial_cep_points_publisher.count(handle) &&
      node->use_sn_for_cepp())
    node->serial_cep_points_publisher[handle]->publish(cpts);
}

void sensorFrameCallback(CeptonSensorHandle handle, int64_t start_timestamp,
                         size_t n_points, const struct CeptonPointEx *points,
                         void *user_data) {
  CeptonPublisher *node = reinterpret_cast<CeptonPublisher *>(user_data);

  // Update the sensor status
  {
    lock_guard<mutex> lock(node->status_lock_);

    node->last_points_time_[handle] = chrono::system_clock::now();
  }

  // Make sure the points buffer exists
  {
    std::lock_guard<std::mutex> guard(node->handle_to_points_mutex_);
    if (!node->handle_to_points.count(handle))
      node->handle_to_points[handle] = vector<uint8_t>();

    auto &ref = node->handle_to_points[handle];

    // If we are publishing every frame, or if the last merge was published for
    // half-frequency mode, then start the buffer from scratch
    node->handle_to_start_timestamp[handle] = start_timestamp;
    ref.resize(n_points * sizeof(CeptonPointEx));
    memcpy(ref.data(), points, n_points * sizeof(CeptonPointEx));
  }

  node->publish_async(handle);
}

void CeptonPublisher::publish_async(CeptonSensorHandle handle) {
  if (pub_fut_.valid()) pub_fut_.wait();
  pub_fut_ = std::async(std::launch::async, [this, handle]() {
    // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h
    if (sensor_clouds_.count(handle) == 0) {
      sensor_clouds_[handle] = PointCloud2();
    }

    int64_t timestamp = handle_to_start_timestamp[handle];

    auto &cloud = sensor_clouds_[handle];
    cloud.height = 1;
    cloud.width = 4;
    cloud.header.stamp.sec = timestamp / 1'000'000;
    // nanosec is the timestamp portion that is truncated from the sec. portion.
    cloud.header.stamp.nanosec = (timestamp % 1'000'000) * 1'000;

    cloud.header.frame_id = "cepton3";

    sensor_msgs::PointCloud2Modifier mod(cloud);

    mod.setPointCloud2Fields(
        11,  // num fields
             // clang-format off
              "x", 1, PointField::FLOAT32,
              "y", 1, PointField::FLOAT32,
              "z", 1, PointField::FLOAT32,
              "intensity", 1, PointField::FLOAT32,
              "timestamp_s", 1, PointField::INT32,
              "timestamp_us", 1, PointField::INT32,
              "flags", 1, PointField::UINT16,
              "channel_id", 1, PointField::UINT16,
              "azimuth", 1, PointField::FLOAT32,
              "elevation", 1, PointField::FLOAT32,
              "range", 1, PointField::FLOAT32
             // clang-format on
    );

    // resizing should be done before the iters are declared, otw seg faults
    mod.resize(handle_to_points[handle].size());

    // Populate the cloud fields
    // Each iterator is optional, and depends on whether the field was added to
    // the cloud.
    auto x_iter = make_optional_iter<float>(cloud, "x");
    auto y_iter = make_optional_iter<float>(cloud, "y");
    auto z_iter = make_optional_iter<float>(cloud, "z");
    auto i_iter = make_optional_iter<float>(cloud, "intensity");
    auto t_iter = make_optional_iter<int32_t>(cloud, "timestamp_us");
    auto t_us_iter = make_optional_iter<int32_t>(cloud, "timestamp_us");
    auto f_iter = make_optional_iter<uint16_t>(cloud, "flags");
    auto c_iter = make_optional_iter<uint16_t>(cloud, "channel_id");
    auto azim_iter = make_optional_iter<double>(cloud, "azimuth");
    auto elev_iter = make_optional_iter<double>(cloud, "elevation");
    auto dist_iter = make_optional_iter<float>(cloud, "range");

    // Set the fields that contain values
    auto set_fields = [&](float x, float y, float z, float i, int32_t t,
                          int32_t t_us, uint16_t f, uint16_t c, double azim,
                          double elev, float dist) {
      // If not using the cepton coordinate system, then swap to using X-forward
      // coordinate system, where X is forward, Y is left, Z is up
      if (!using_cepton_coordinate_system_) {
        auto tx = y;
        auto ty = -x;
        auto tz = z;
        x = tx;
        y = ty;
        z = tz;
      }

      if (x_iter.has_value()) *(x_iter.value()) = x;
      if (y_iter.has_value()) *(y_iter.value()) = y;
      if (z_iter.has_value()) *(z_iter.value()) = z;
      if (i_iter.has_value()) *(i_iter.value()) = i;
      if (t_iter.has_value()) *(t_iter.value()) = t;
      if (t_us_iter.has_value()) *(t_us_iter.value()) = t_us;
      if (f_iter.has_value()) *(f_iter.value()) = f;
      if (c_iter.has_value()) *(c_iter.value()) = c;
      if (azim_iter.has_value()) *(azim_iter.value()) = azim;
      if (elev_iter.has_value()) *(elev_iter.value()) = elev;
      if (dist_iter.has_value()) *(dist_iter.value()) = dist;
    };

    // Move the iterators that contain values
    auto move_iters = [&]() {
      if (x_iter.has_value()) ++x_iter.value();
      if (y_iter.has_value()) ++y_iter.value();
      if (z_iter.has_value()) ++z_iter.value();
      if (i_iter.has_value()) ++i_iter.value();
      if (t_iter.has_value()) ++t_iter.value();
      if (t_us_iter.has_value()) ++t_us_iter.value();
      if (f_iter.has_value()) ++f_iter.value();
      if (c_iter.has_value()) ++c_iter.value();
      if (azim_iter.has_value()) ++azim_iter.value();
      if (elev_iter.has_value()) ++elev_iter.value();
      if (dist_iter.has_value()) ++dist_iter.value();
    };

    int kept = 0;

    auto const min_distance_squared = min_distance_ * min_distance_;
    auto const max_distance_squared = max_distance_ * max_distance_;

    // if previous frame is recorded, populate point cloud with this first
    if (handle_to_points.count(handle)) {
      std::lock_guard<std::mutex> guard(handle_to_points_mutex_);
      auto const &ref = handle_to_points[handle];
      auto const num_points = ref.size() / sizeof(CeptonPointEx);
      for (unsigned i = 0; i < num_points; i++) {
        CeptonPointEx const &p0 = *reinterpret_cast<CeptonPointEx const *>(
            ref.data() + i * sizeof(CeptonPointEx));

        timestamp += p0.relative_timestamp;
        // If point has flags that should not be included (specified by the
        // include_flag), continue
        if ((~(include_flag_) & (p0.flags)) != 0) {
          continue;
        }

        const float x = p0.x * 1.0 / 65536.0;  // 0.005;
        const float y = p0.y * 1.0 / 65536.0;  // 0.005;
        const float z = p0.z * 1.0 / 65536.0;  // 0.005;

        const float distance_squared = x * x + y * y + z * z;
        // Filter out points that are labelled ambient but have invalid distance
        // until point flag definitions are finalized (> 500m for now)
        if (distance_squared >= 500 * 500) continue;

        const float image_x = x / y;
        const float image_z = z / y;

        const double azimuth_rad = atan(image_x);
        const double elevation_rad =
            atan2(image_z, sqrt(image_x * image_x + 1));

        if (image_x < min_image_x_ || image_x > max_image_x_ ||
            image_z < min_image_z_ || image_z > max_image_z_ ||
            distance_squared < min_distance_squared ||
            distance_squared > max_distance_squared)
          continue;

        bool const is_valid_return = (p0.flags & CEPTON_POINT_NO_RETURN) == 0;
        float range_meas = is_valid_return ? sqrt(x * x + y * y + z * z) : 0.0;
        set_fields(x, y, z, static_cast<float>(p0.reflectivity) * 0.01,
                   static_cast<int32_t>(timestamp / (int64_t)1e6),
                   static_cast<int32_t>(timestamp % (int64_t)1e6), p0.flags,
                   p0.channel_id, azimuth_rad, elevation_rad, range_meas);

        move_iters();

        kept++;
      }
    } else {
      throw new runtime_error("trying to publish without a points buffer");
    }
    mod.resize(kept);

    // Publish points
    points_publisher->publish(cloud);

    // Publish by handle - create publisher if needed
    if (use_handle_for_pcl2()) {
      ensure_pcl2_publisher(handle, "handle_" + to_string(handle),
                            handle_points_publisher);
      handle_points_publisher[handle]->publish(cloud);
    }

    // If info packet is received to link handle to serial number, publish by
    // serial number
    if (serial_points_publisher.count(handle) && use_sn_for_pcl2())
      serial_points_publisher[handle]->publish(cloud);
  });
}

void sensor_info_callback(CeptonSensorHandle handle,
                          const struct CeptonSensor *info, void *user_data) {
  auto *node = reinterpret_cast<CeptonPublisher *>(user_data);

  // Update the sensor status
  {
    lock_guard<mutex> lock(node->status_lock_);
    node->handle_to_serial_number_[handle] = info->serial_number;
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
  if (node->use_sn_for_cepp())
    node->ensure_cepp_publisher(handle,
                                "cepp_serial_" + to_string(info->serial_number),
                                node->serial_cep_points_publisher);
  if (node->use_sn_for_pcl2())
    node->ensure_pcl2_publisher(handle,
                                "serial_" + to_string(info->serial_number),
                                node->serial_points_publisher);

  // Publish info on the unified topic
  node->info_publisher->publish(msg);

  // Publish info on the per-sensor info topics.
  // Publish on the sensor-specific topic
  node->ensure_info_publisher(handle, "info_handle_" + to_string(handle),
                              node->handle_to_info_publisher);
  node->handle_to_info_publisher[handle]->publish(msg);
}

/// @brief Create a pcl2 publisher and store in the provided map, if not
/// already existing
/// @param topic
/// @param m
void CeptonPublisher::ensure_pcl2_publisher(
    CeptonSensorHandle handle, string const &topic,
    unordered_map<CeptonSensorHandle, PointPublisher> &m) {
  if (!m.count(handle)) m[handle] = create_publisher<PointCloud2>(topic, 50);
}

/// @brief Create a cepp points publisher and store in the provided map, if
/// not already existing
/// @param topic
/// @param m
void CeptonPublisher::ensure_cepp_publisher(
    CeptonSensorHandle handle, string const &topic,
    unordered_map<CeptonSensorHandle, CepPointPublisher> &m) {
  if (!m.count(handle)) m[handle] = create_publisher<CeptonPoints>(topic, 50);
}

void CeptonPublisher::ensure_info_publisher(
    CeptonSensorHandle handle, string const &topic,
    unordered_map<CeptonSensorHandle, CepInfoPublisher> &m) {
  if (!m.count(handle))
    m[handle] =
        create_publisher<cepton_messages::msg::CeptonSensorInfo>(topic, 10);
}

CeptonPublisher::CeptonPublisher() : Node("cepton_publisher") {
  // Allowed options, with default values
  RCLCPP_DEBUG(this->get_logger(), "My log message %d", 4);

  declare_parameter("capture_file", "");
  declare_parameter("capture_loop", false);
  declare_parameter("sensor_port", 8808);
  declare_parameter("half_frequency_mode", false);
  declare_parameter("cepp_output_type",
                    "BOTH");  // "NONE" or "IP" or "SN" or "BOTH"
  declare_parameter("pcl2_output_type",
                    "BOTH");  // "NONE" or "IP" or "SN" or "BOTH"
  declare_parameter("include_saturated_points", true);
  declare_parameter("include_second_return_points", true);
  declare_parameter("include_invalid_points", false);
  declare_parameter("include_noise_points", false);
  declare_parameter("include_blocked_points", true);
  declare_parameter("min_altitude", -90.);
  declare_parameter("max_altitude", 90.);
  declare_parameter("min_azimuth", -180.);
  declare_parameter("max_azimuth", 180.);
  declare_parameter("aggregation_mode", 0);
  declare_parameter("using_cepton_coordinate_system", true);
  declare_parameter("max_distance", numeric_limits<float>::max());
  declare_parameter("min_distance", 0.0);
  declare_parameter("expected_sensor_ips", vector<string>{});

  // Initialize sdk
  int ret = CeptonInitialize(CEPTON_API_VERSION, nullptr);
  check_sdk_error(ret, "CeptonInitialize");

  // Get the aggregation mode
  rclcpp::Parameter p_aggregation_mode = get_parameter("aggregation_mode");
  frame_aggregation_mode_ = (int)p_aggregation_mode.as_int();
  RCLCPP_DEBUG(this->get_logger(), "Using frame aggregation %d",
               frame_aggregation_mode_);

  RCLCPP_DEBUG(this->get_logger(),
               "========= Point Cloud Output Parameters =========");
  // Check whether to output cepp points
  rclcpp::Parameter pOutputCepp = get_parameter("cepp_output_type");
  std::string cepp_output_type = pOutputCepp.as_string();
  if (cepp_output_type == "NONE") {
    use_handle_for_cepp_ = use_sn_for_cepp_ = false;
    RCLCPP_DEBUG(this->get_logger(), "Not publishing CEPP points!");

  } else {
    RCLCPP_DEBUG(this->get_logger(), "Publishing CEPP points with: %s",
                 cepp_output_type.c_str());
    use_handle_for_cepp_ =
        cepp_output_type == "IP" || cepp_output_type == "BOTH";
    RCLCPP_DEBUG(this->get_logger(), "\tPublishing CEPP points with handle: %s",
                 use_handle_for_cepp_ ? "true" : "false");
    use_sn_for_cepp_ = cepp_output_type == "SN" || cepp_output_type == "BOTH";
    RCLCPP_DEBUG(this->get_logger(), "\tPublishing CEPP points with SN: %s",
                 use_sn_for_cepp_ ? "true" : "false");
    cep_points_publisher = create_publisher<CeptonPoints>("cepton_points", 50);

    // Register callback
    ret = CeptonListenFramesEx(frame_aggregation_mode_, ceptonFrameCallback,
                               this);
    check_sdk_error(ret, "CeptonListenFramesEx");
  }

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
    ret = CeptonListenFramesEx(frame_aggregation_mode_, sensorFrameCallback,
                               this);
    check_sdk_error(ret, "CeptonListenFramesEx");
  }
  RCLCPP_DEBUG(this->get_logger(),
               "=================================================");

  // ---------------------------------
  // Set up the sensor info publishing
  {
    // Create publisher
    info_publisher = create_publisher<cepton_messages::msg::CeptonSensorInfo>(
        "cepton_info", 10);
    // Register callback
    ret = CeptonListenSensorInfo(sensor_info_callback, this);
    check_sdk_error(ret, "CeptonListenSensorInfo");
  }

  // -----------------------------------
  // Set up the sensor status publishing
  {
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

  // Check whether to output 2 frames at once
  half_frequency_mode_ = get_parameter("half_frequency_mode").as_bool();

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

  // Coordinate system settings
  using_cepton_coordinate_system_ =
      get_parameter("using_cepton_coordinate_system").as_bool();

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
