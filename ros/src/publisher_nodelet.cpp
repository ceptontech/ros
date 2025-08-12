#include "publisher_nodelet.hpp"

#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <pluginlib/class_list_macros.h>

#include <future>
#include <iostream>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(cepton_ros::PublisherNodelet, nodelet::Nodelet);

inline double degrees_to_radians(double t) { return t * M_PI / 180.0; }

namespace cepton_ros {

/**
 * SDK callback
 */
void on_ex_frame(CeptonSensorHandle handle, int64_t start_timestamp,
                 size_t n_points, const struct CeptonPointEx *points,
                 void *user_data) {
  reinterpret_cast<PublisherNodelet *>(user_data)->publish_points(
      handle, start_timestamp, n_points, points);
}

/**
 * SDK callback
 */
void on_sensor_info(CeptonSensorHandle handle, const struct CeptonSensor *info,
                    void *user_data) {
  reinterpret_cast<PublisherNodelet *>(user_data)->publish_sensor_info(info);
}

PublisherNodelet::~PublisherNodelet() {
  int ret;
  ret = CeptonDeinitialize();
  check_api_error(ret, "CeptonDeinitialize");
}

void PublisherNodelet::check_api_error(int err, char const *api) {
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
  ROS_INFO("PublisherNodeletStarted");

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
  private_node_handle_.param("expected_sensor_ips", expected_sensor_ips_,
                             expected_sensor_ips_);

  // Check for which points should be included based on params for flag bits
  {
    bool include = true;
    include_flag_ = 0;
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

    ROS_INFO("=================================================\n");

    include_flag_ |= CEPTON_POINT_BLOOMING | CEPTON_POINT_FRAME_PARITY |
                     CEPTON_POINT_FRAME_BOUNDARY;
  }

  // Assign publisher for info
  sensor_info_publisher_ =
      node_handle_.advertise<cepton_ros::SensorInformation>(
          "cepton3/sensor_information", 2);

  // All points
  points_publisher_ =
      node_handle_.advertise<cepton_ros::Cloud>("cepton3/points", 50);

  // Initialize SDK
  ret = CeptonInitialize(CEPTON_API_VERSION, nullptr);
  check_api_error(ret, "CeptonInitialize");

  // Start SDK CaptureReplay or Networking
  if (!capture_path.empty()) {
    ret = CeptonReplayLoadPcap(capture_path.c_str(), replay_flags,
                               &replay_handle_);
    check_api_error(ret, "CeptonReplayLoadPcap");
  } else {
    ret = CeptonStartNetworking();
    check_api_error(ret, "CeptonStartNetworking");
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
        ros::Duration(0.1), [&](const ros::TimerEvent &event) {
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
  for (auto const &expected_ip : expected_sensor_ips_) {
    NODELET_INFO("[%s] Adding expected IP %s\n", getName().c_str(),
                 expected_ip.c_str());
    struct in_addr addr;
    inet_aton(expected_ip.c_str(), &addr);
    // handle is in big-endian. inet_aton returns little endian
    last_points_time_[__bswap_32(addr.s_addr)] =
        std::chrono::system_clock::now();
  }
}

void PublisherNodelet::publish_points(CeptonSensorHandle handle,
                                      int64_t start_timestamp, size_t n_points,
                                      const CeptonPointEx *points) {
  // Buffer to build the output cloud
  static std::unordered_map<CeptonSensorHandle, cepton_ros::Cloud> clouds;

  // Update the sensor status
  {
    std::lock_guard<std::mutex> lock(status_lock_);
    last_points_time_[handle] = std::chrono::system_clock::now();
  }

  // Prep the cloud
  {
    // Modify the same cloud buffer each time to avoid realloc
    auto &cloud = clouds[handle];
    cloud.clear();
    cloud.header.stamp = start_timestamp;
    cloud.header.frame_id = "cepton3";
    cloud.height = 1;
    cloud.width = n_points;
    cloud.reserve(n_points);

    const auto max_distance_squared = max_distance_ * max_distance_;
    const auto min_distance_squared = min_distance_ * min_distance_;

    // Loop and add the points
    int kept = 0;
    int noise_count = 0;
    for (int i = 0; i < n_points; ++i) {
      cepton_ros::Point cp;
      auto const &p = points[i];

      // If point has flags that should not be included (specified by the
      // include_flag), continue
      if ((~include_flag_ & p.flags) != 0) continue;

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

      const float tan_yx = y / x;
      const float tan_zx = z / x;

      const double azimuth_rad = atan(tan_yx);
      const double elevation_rad = atan2(tan_zx, sqrt(tan_yx * tan_yx + 1));

      if (tan_yx < min_image_x_ || tan_yx > max_image_x_ ||
          tan_zx < min_image_z_ || tan_zx > max_image_z_ ||
          distance_squared < min_distance_squared ||
          distance_squared > max_distance_squared)
        continue;

      cp.x = x;
      cp.y = y;
      cp.z = z;
      cp.intensity = p.reflectivity * 0.01;
      cp.relative_timestamp = p.relative_timestamp;
      cp.channel_id = p.channel_id;
      cp.flags = p.flags;
      cp.azimuth = azimuth_rad;
      cp.elevation = elevation_rad;
      cp.valid = !(p.flags & CEPTON_POINT_NO_RETURN);

      cloud.points.push_back(cp);
      kept++;
    }
    // Resize according to number of kept points
    cloud.width = kept;
    if (kept > 0) cloud.points.resize(kept);
  }

  // Publish the point cloud
  {
    // If the last publish is still pending, wait for it to finish
    if (pub_fut_.valid()) pub_fut_.wait();

    // Launch a new process to do the publishing
    pub_fut_ =
        std::async(std::launch::async, [this, handle, start_timestamp]() {
          // Make sure there is an allocated buffer
          if (!clouds.count(handle)) {
            clouds[handle] = cepton_ros::Cloud();
          }

          auto const &cloud = clouds[handle];

          // Publish points
          points_publisher_.publish(cloud);

          // Publish by handle
          if (handle_points_publisher_.count(handle) && output_by_handle_)
            handle_points_publisher_[handle].publish(cloud);

          // Publish by serial number
          if (serial_points_publisher_.count(handle) && output_by_sn_)
            serial_points_publisher_[handle].publish(cloud);
        });
  }
}

void PublisherNodelet::publish_sensor_info(const CeptonSensor *info) {
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
            node_handle_.advertise<cepton_ros::Cloud>(handle_topic_name, 2)));
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
            node_handle_.advertise<cepton_ros::Cloud>(sn_topic_name, 2)));
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
  msg.fault_summary = info->fault_summary;
  std::copy(std::begin(info->fault_entries), std::end(info->fault_entries),
            msg.fault_entries.begin());

  // Publish on the unified info topic
  sensor_info_publisher_.publish(msg);

  // Publish on the sensor-specific topic
  handle_info_publisher_[info->handle].publish(msg);
}

}  // namespace cepton_ros
