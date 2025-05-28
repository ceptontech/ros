#include "publisher_nodelet.hpp"

#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <pluginlib/class_list_macros.h>

#include <future>
#include <iostream>
#include <string>
#include <vector>

#include "panic.h"

PLUGINLIB_EXPORT_CLASS(cepton2_ros::PublisherNodelet, nodelet::Nodelet);
double degrees_to_radians(double t) { return t * M_PI / 180.0; }

using namespace std;

/**
 * @class Helper class to get the publish parity to get crossed scans.
 * Parity is returned based on non-boundary points, so be sure to skip boundary
 * points when checking if a frame should be published when using the result of
 * this calc.
 */
class HalfFrequencyParityCalc {
 public:
  static constexpr auto SIGN_NOT_SET   = 0;
  static constexpr auto PARITY_NOT_SET = -1;

  HalfFrequencyParityCalc() = default;

  /**
   * @brief Return the parity that should trigger publish of half-frequency
   * frames, or -1 if the parity is not yet determined
   */
  int update(uint8_t const *points, size_t num_points, size_t stride) {
    if (publish_parity != PARITY_NOT_SET) {
      // Calc is finished, return the result
      return publish_parity;
    }
    if (last_frame_sign == SIGN_NOT_SET) {
      // If this is the first frame, just get the sign and return
      last_frame_sign = calc_sign(points, num_points, stride);
      return PARITY_NOT_SET;
    }
    // Get the sign of the new incoming frame
    int cur_sign = calc_sign(points, num_points, stride);

    // If the sign changed, then going forward we should publish on this parity
    if (cur_sign != last_frame_sign) {
      // Scan forward to find the first non-boundary parity
      size_t index;
      for (index = 0; index < num_points; index++) {
        CeptonPoint const &point =
            *(CeptonPoint const *)(points + index * stride);
        if ((point.flags & CEPTON_POINT_FRAME_BOUNDARY) == 0) break;
      }
      CeptonPoint const &sel_point =
          *(CeptonPoint const *)(points + index * stride);
      publish_parity =
          static_cast<int>(sel_point.flags & CEPTON_POINT_FRAME_PARITY);
      return publish_parity;
    } else {
      return PARITY_NOT_SET;
    }
  }

 private:
  int last_frame_sign = SIGN_NOT_SET;
  int publish_parity  = PARITY_NOT_SET;

  static int calc_sign(uint8_t const *points, size_t num_points,
                       size_t stride) {
    vector<float> azim;
    vector<float> elev;

    for (size_t i = 0; i < num_points; i++) {
      CeptonPoint const &point = *(CeptonPoint const *)(points + i * stride);
      if (point.channel_id == 0) {
        float ix     = -static_cast<float>(point.x) / point.y;
        float iz     = -static_cast<float>(point.z) / point.y;
        float azim_i = atan(-ix);
        float elev_i = atan2(-iz, sqrt(ix * ix + 1.0));
        azim.push_back(azim_i);
        elev.push_back(elev_i);
      }
    }
    // Calculate the slope
    float x_mu = accumulate(azim.begin(), azim.end(), 0.0) / azim.size();
    float y_mu = accumulate(elev.begin(), elev.end(), 0.0) / elev.size();

    float numer = 0;
    float denom = 0;
    for (size_t i = 0; i < azim.size(); i++) {
      numer += (azim[i] - x_mu) * (elev[i] - y_mu);
      denom += (azim[i] - x_mu) * (azim[i] - x_mu);
    }
    float slope = numer / denom;
    return slope > 0.0 ? 1 : -1;
  }
};

static auto half_freq_parity_calc = HalfFrequencyParityCalc();

namespace cepton2_ros {
// Reflectivity look up table
static const float reflectivity_LUT[256] = {
    0.000f,  0.010f,  0.020f,  0.030f,  0.040f,  0.050f,  0.060f,  0.070f,
    0.080f,  0.090f,  0.100f,  0.110f,  0.120f,  0.130f,  0.140f,  0.150f,
    0.160f,  0.170f,  0.180f,  0.190f,  0.200f,  0.210f,  0.220f,  0.230f,
    0.240f,  0.250f,  0.260f,  0.270f,  0.280f,  0.290f,  0.300f,  0.310f,
    0.320f,  0.330f,  0.340f,  0.350f,  0.360f,  0.370f,  0.380f,  0.390f,
    0.400f,  0.410f,  0.420f,  0.430f,  0.440f,  0.450f,  0.460f,  0.470f,
    0.480f,  0.490f,  0.500f,  0.510f,  0.520f,  0.530f,  0.540f,  0.550f,
    0.560f,  0.570f,  0.580f,  0.590f,  0.600f,  0.610f,  0.620f,  0.630f,
    0.640f,  0.650f,  0.660f,  0.670f,  0.680f,  0.690f,  0.700f,  0.710f,
    0.720f,  0.730f,  0.740f,  0.750f,  0.760f,  0.770f,  0.780f,  0.790f,
    0.800f,  0.810f,  0.820f,  0.830f,  0.840f,  0.850f,  0.860f,  0.870f,
    0.880f,  0.890f,  0.900f,  0.910f,  0.920f,  0.930f,  0.940f,  0.950f,
    0.960f,  0.970f,  0.980f,  0.990f,  1.000f,  1.010f,  1.020f,  1.030f,
    1.040f,  1.050f,  1.060f,  1.070f,  1.080f,  1.090f,  1.100f,  1.110f,
    1.120f,  1.130f,  1.140f,  1.150f,  1.160f,  1.170f,  1.180f,  1.190f,
    1.200f,  1.210f,  1.220f,  1.230f,  1.240f,  1.250f,  1.260f,  1.270f,
    1.307f,  1.345f,  1.384f,  1.424f,  1.466f,  1.509f,  1.553f,  1.598f,
    1.644f,  1.692f,  1.741f,  1.792f,  1.844f,  1.898f,  1.953f,  2.010f,
    2.069f,  2.129f,  2.191f,  2.254f,  2.320f,  2.388f,  2.457f,  2.529f,
    2.602f,  2.678f,  2.756f,  2.836f,  2.919f,  3.004f,  3.091f,  3.181f,
    3.274f,  3.369f,  3.467f,  3.568f,  3.672f,  3.779f,  3.889f,  4.002f,
    4.119f,  4.239f,  4.362f,  4.489f,  4.620f,  4.754f,  4.892f,  5.035f,
    5.181f,  5.332f,  5.488f,  5.647f,  5.812f,  5.981f,  6.155f,  6.334f,
    6.519f,  6.708f,  6.904f,  7.105f,  7.311f,  7.524f,  7.743f,  7.969f,
    8.201f,  8.439f,  8.685f,  8.938f,  9.198f,  9.466f,  9.741f,  10.025f,
    10.317f, 10.617f, 10.926f, 11.244f, 11.572f, 11.909f, 12.255f, 12.612f,
    12.979f, 13.357f, 13.746f, 14.146f, 14.558f, 14.982f, 15.418f, 15.866f,
    16.328f, 16.804f, 17.293f, 17.796f, 18.314f, 18.848f, 19.396f, 19.961f,
    20.542f, 21.140f, 21.755f, 22.389f, 23.040f, 23.711f, 24.401f, 25.112f,
    25.843f, 26.595f, 27.369f, 28.166f, 28.986f, 29.830f, 30.698f, 31.592f,
    32.511f, 33.458f, 34.432f, 35.434f, 36.466f, 37.527f, 38.620f, 39.744f,
    40.901f, 42.092f, 43.317f, 44.578f, 45.876f, 47.211f, 48.586f, 50.000f,
};

PublisherNodelet::~PublisherNodelet() {
  int ret;
  ret = CeptonDeinitialize();
  check_api_error(ret, "CeptonDeinitialize");
}

void PublisherNodelet::check_api_error(int err, char const *api) {
  if (err != CEPTON_SUCCESS) {
    printf("API Error for %s: %s\n", api, CeptonGetErrorCodeName(err));
    exit(1);
  }
}

/**
 * Use for indexing into the voltage field of CeptonPanic::voltage_out_of_range
 */
enum {
  VoltageFault_Komodo_0_8v = 0,
  VoltageFault_Komodo_1_0v,
  VoltageFault_Komodo_1_8v,
  VoltageFault_Komodo_3_3v,
  VoltageFault_Motor_5_3v,
  VoltageFault_OM_5_3v,
  VoltageFault_Laser_32v
};

const auto SENSOR_POINTS_TIMEOUT = std::chrono::seconds(3);

void PublisherNodelet::onInit() {
  ROS_INFO("PublisherNodeletStarted");
  int ret;

  // Get node handle
  node_handle_         = getNodeHandle();
  private_node_handle_ = getPrivateNodeHandle();

  // Get Parameters
  std::string capture_path = "";
  private_node_handle_.param("capture_path", capture_path, capture_path);
  std::cerr << "capture path " << capture_path << std::endl;
  bool capture_loop     = true;
  uint32_t replay_flags = 0;
  private_node_handle_.param("capture_loop", capture_loop, capture_loop);
  printf("Replay loop: %d\n", (int)capture_loop);
  if (capture_loop) {
    replay_flags = CEPTON_REPLAY_FLAG_PLAY_LOOPED;
  }

  private_node_handle_.param("half_frequency_mode", is_half_frequency_mode_,
                             is_half_frequency_mode_);

  private_node_handle_.param("min_altitude", min_altitude_, min_altitude_);
  private_node_handle_.param("max_altitude", max_altitude_, max_altitude_);
  private_node_handle_.param("min_azimuth", min_azimuth_, min_azimuth_);
  private_node_handle_.param("max_azimuth", max_azimuth_, max_azimuth_);

  // Clip so that we can safely take the tangent
  min_azimuth_  = max(-89.9, min_azimuth_);
  max_azimuth_  = min(89.9, max_azimuth_);
  min_altitude_ = max(-89.9, min_altitude_);
  max_altitude_ = min(89.9, max_altitude_);

  min_image_x_ = tan(degrees_to_radians(min_azimuth_));
  max_image_x_ = tan(degrees_to_radians(max_azimuth_));
  min_image_z_ = tan(degrees_to_radians(min_altitude_));
  max_image_z_ = tan(degrees_to_radians(max_altitude_));

  private_node_handle_.param("max_distance", max_distance_, max_distance_);
  private_node_handle_.param("min_distance", min_distance_, min_distance_);

  private_node_handle_.param("output_by_handle", output_by_handle_,
                             output_by_handle_);
  private_node_handle_.param("output_by_sn", output_by_sn_, output_by_sn_);

  private_node_handle_.param("aggregation_mode", frame_aggregation_mode_,
                             frame_aggregation_mode_);

  private_node_handle_.param("using_cepton_coordinate_system",
                             using_cepton_coordinate_system_,
                             using_cepton_coordinate_system_);

  private_node_handle_.param("expected_sensor_ips", expected_sensor_ips_,
                             expected_sensor_ips_);

  // Check for which points should be included based on params for flag bits
  CheckIncludedFlags();

  // Assign publisher for info
  sensor_info_publisher_ =
      node_handle_.advertise<cepton2_ros::SensorInformation>(
          "cepton2/sensor_information", 2);

  // All points
  points_publisher_ = node_handle_.advertise<cepton2_ros::CeptonPointCloud>(
      "cepton2/points", 50);

  // Create Callbacks
  error_callback = [](CeptonSensorHandle handle, int error_code,
                      const char *error_msg, const void *error_data,
                      size_t error_data_size) {
    printf("Got error: %s\n", error_msg);
  };

  // Initialize SDK
  ret = CeptonInitialize(CEPTON_API_VERSION, error_callback);
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
  ret = CeptonListenFrames(frame_aggregation_mode_, FrameCallbackWrapper, this);
  check_api_error(ret, "CeptonListenFrames");

  // Listen for sensor info
  ret = CeptonListenSensorInfo(SensorInfoCallbackWrapper, this);
  check_api_error(ret, "CeptonListenSensorInfo");
  ret = CeptonListenPanic(SensorPanicCallbackWrapper, this);
  check_api_error(ret, "CeptonListenPanic");
  // Loop until replay is finished
  // Start watchdog timer
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
        node_handle_.advertise<cepton2_ros::CeptonSensorStatus>(
            "cepton2/cepton_sensor_status", 2);

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

          for (auto itr = last_points_time_.begin();
               itr != last_points_time_.end(); itr++) {
            auto const handle           = itr->first;
            auto const last_points_time = itr->second;
            if (now - last_points_time > SENSOR_POINTS_TIMEOUT) {
              timed_out_sensors[handle] = handle_to_serial_number_.count(handle)
                                              ? handle_to_serial_number_[handle]
                                              : 0;
            }
          }
          for (auto itr = timed_out_sensors.begin();
               itr != timed_out_sensors.end(); itr++) {
            auto const handle        = itr->first;
            auto const serial_number = itr->second;
            auto msg                 = cepton2_ros::CeptonSensorStatus();
            msg.serial_number        = serial_number;
            msg.handle               = handle;
            msg.status               = SENSOR_TIMED_OUT;
            sensor_status_publisher_.publish(msg);
          }
        }
        this_thread::sleep_for(chrono::seconds(1));
      }
    });
  }

  // Set up the expected IPs
  for (auto const &expected_ip : expected_sensor_ips_) {
    std::cerr << "Adding expected IP " << expected_ip << std::endl;
    struct in_addr addr;
    inet_aton(expected_ip.c_str(), &addr);
    // handle is in big-endian. inet_aton returns little endian
    last_points_time_[__bswap_32(addr.s_addr)] = chrono::system_clock::now();
  }

}  // PublisherNodelet::onInit

void PublisherNodelet::FrameCallbackWrapper(CeptonSensorHandle handle,
                                            int64_t start_timestamp,
                                            size_t n_points, size_t stride,
                                            const uint8_t *points,
                                            void *user_data) {
  reinterpret_cast<PublisherNodelet *>(user_data)->PublishPoints(
      handle, start_timestamp, n_points, stride, points);

}  // PublisherNodelet::FrameCallbackWrapper

void PublisherNodelet::SensorInfoCallbackWrapper(
    CeptonSensorHandle handle, const struct CeptonSensor *info,
    void *user_data) {
  reinterpret_cast<PublisherNodelet *>(user_data)->PublishSensorInformation(
      info);
}  // PublisherNodelet::SensorInfoCallbackWrapper

void PublisherNodelet::SensorPanicCallbackWrapper(
    CeptonSensorHandle handle, const CeptonPanicMessage *panic_message,
    void *user_data) {
  reinterpret_cast<PublisherNodelet *>(user_data)->PublishSensorPanic(
      handle, panic_message);
}  // PublisherNodelet::SensorPanicCallbackWrapper

int get_non_boundary_parity(uint8_t const *points, size_t n_points,
                            size_t stride) {
  for (size_t index = 0; index < n_points; index++) {
    CeptonPoint const &point = *(CeptonPoint const *)(points + index * stride);
    if ((point.flags & CEPTON_POINT_FRAME_BOUNDARY) == 0) {
      return point.flags & CEPTON_POINT_FRAME_PARITY;
    }
  }
  return -1;
}

void PublisherNodelet::PublishPoints(CeptonSensorHandle handle,
                                     int64_t start_timestamp, size_t n_points,
                                     size_t stride, const uint8_t *points) {
  // Update the sensor status
  {
    lock_guard<mutex> lock(status_lock_);
    last_points_time_[handle] = chrono::system_clock::now();
  }

  // Update the half-frequency publish parity calc
  static unordered_map<CeptonSensorHandle, HalfFrequencyParityCalc> pc;
  if (pc.find(handle) == pc.end()) pc[handle] = HalfFrequencyParityCalc();
  auto publish_parity = pc[handle].update(points, n_points, stride);

  // Check if this is a publish frame
  bool const is_publish_frame =
      get_non_boundary_parity(points, n_points, stride) == publish_parity;

  // Make sure the points buffer exists
  if (!handle_to_points_.count(handle))
    handle_to_points_[handle] = vector<uint8_t>();

  auto &ref = handle_to_points_[handle];

  // If we are publishing every frame, or if the last merge was published for
  // half-frequency mode, then start the buffer from scratch
  if (!is_half_frequency_mode_ ||
      (is_half_frequency_mode_ && !is_publish_frame)) {
    handle_to_start_timestamp_[handle] = start_timestamp;
    ref.resize(n_points * stride);
    copy(points, points + n_points * stride, ref.begin());
  } else {
    // If in half frequency mode and publishing, then extend the existing buffer
    auto orig_size = ref.size();
    ref.resize(ref.size() + n_points * stride);
    copy(points, points + n_points * stride, ref.data() + orig_size);
  }

  // Return if not publishing
  if (is_half_frequency_mode_ && !is_publish_frame) return;

  this->publish_async(handle, stride);

}  // PublisherNodelet::PublishPoints

void PublisherNodelet::publish_async(CeptonSensorHandle handle, size_t stride) {
  // If the last publish is still pending, wait for it to finish
  if (pub_fut_.valid()) pub_fut_.wait();

  pub_fut_ = std::async(std::launch::async, [this, stride, handle]() {
    // Check for cached output points
    if (!rostopic_point_clouds_.count(handle)) {
      rostopic_point_clouds_[handle] = CeptonPointCloud();
    }

    auto const &cepp_points = handle_to_points_[handle];
    auto const *points      = cepp_points.data();

    auto max_num_points = cepp_points.size() / stride;

    // Modify the same cloud buffer each time to avoid realloc
    auto &cloud = rostopic_point_clouds_[handle];
    cloud.clear();
    cloud.header.stamp    = handle_to_start_timestamp_[handle];
    cloud.header.frame_id = "cepton2";
    cloud.height          = 1;
    cloud.width           = max_num_points;
    cloud.reserve(max_num_points);

    const auto max_distance_squared = max_distance_ * max_distance_;
    const auto min_distance_squared = min_distance_ * min_distance_;

    // Loop and add the points
    int kept = 0;
    cepton2_ros::CustomCeptonPoint cp;
    int noise_count = 0;
    for (int i = 0; i < max_num_points; ++i) {
      CeptonPoint const &p =
          *reinterpret_cast<CeptonPoint const *>(points + i * stride);
      // If point has flags that should not be included (specified by the
      // include_flag), continue
      if ((~include_flag_ & p.flags) != 0) continue;

      float x = p.x * 0.005;
      float y = p.y * 0.005;
      float z = p.z * 0.005;

      const float distance_squared = x * x + y * y + z * z;

      const float image_x = x / y;
      const float image_z = z / y;

      const double azimuth_rad   = atan(image_x);
      const double elevation_rad = atan2(image_z, sqrt(image_x * image_x + 1));

      if (image_x < min_image_x_ || image_x > max_image_x_ ||
          image_z < min_image_z_ || image_z > max_image_z_ ||
          distance_squared < min_distance_squared ||
          distance_squared > max_distance_squared)
        continue;

      // If not using the cepton coordinate system, then swap to using X-forward
      // coordinate system, where X is forward, Y is left, Z is up
      if (!using_cepton_coordinate_system_) {
        auto tx = y;
        auto ty = -x;
        auto tz = z;
        x       = tx;
        y       = ty;
        z       = tz;
      }

      cp.x                  = x;
      cp.y                  = y;
      cp.z                  = z;
      cp.reflectivity       = reflectivity_LUT[p.reflectivity];
      cp.relative_timestamp = p.relative_timestamp;
      cp.channel_id         = p.channel_id;
      cp.flags              = p.flags;
      cp.azimuth            = azimuth_rad;
      cp.elevation          = elevation_rad;
      cp.valid              = !(p.flags & CEPTON_POINT_NO_RETURN);

      cloud.points.push_back(cp);
      kept++;
    }
    cloud.width = kept;
    if (kept > 0) cloud.points.resize(kept);

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

class FaultMonitor {
 public:
  struct FaultEntries {
    uint8_t voltage_out_range[8];
    uint8_t temperature_out_range_komodo;
    uint8_t temperature_out_range_iox;
    uint8_t temperature_out_range_iguana;
    uint8_t temperature_out_range_gecko;
    uint8_t iguana_iox_comm_crc_error;
    uint8_t iox_gecko_comm_crc_error;
    uint8_t gecko_komodo_comm_crc_error;
    uint8_t iguana_boot_failure;
    uint8_t phy_sync_failure;
    uint8_t eth_link_down;
    uint8_t ptp_expiration;
    uint8_t thermal_shut_down;
    uint8_t lasers_partial_block;
    uint8_t lasers_full_block;
    uint8_t mmt_interlock;
    uint8_t iguana_failure;
    uint8_t fpga_runtime_fail;
    uint8_t mmt_timeout;
    uint8_t gecko_boot_failure;
    uint8_t rsv[5];
  };

 private:
  static const uint32_t MONITORED_FIELD_COUNT = 32;

  static_assert(sizeof(FaultEntries) == MONITORED_FIELD_COUNT);

  FaultEntries fault_entries_;

 public:
  enum { FAULT_ACTIVE = 0x3 };

  /**
   * Update the fault entries based on the INFZ
   */
  cepton2_ros::SensorPanic update(CeptonSensor const *info, int64_t timestamp,
                                  CeptonSensorHandle handle) {
    // Copy the fault entries
    fault_entries_ =
        *reinterpret_cast<FaultEntries const *>(info->fault_entries);

    // Return the updated message
    return get_msg(timestamp, handle);
  }

  /**
   * Update the fault entries based on the panic packet
   */
  cepton2_ros::SensorPanic update(CeptonPanicMessage const *panic,
                                  int64_t timestamp,
                                  CeptonSensorHandle handle) {
    switch (panic->fault_identity) {
      case VOLTAGE_KOMODO_0V8:  // Voltage out-of-range 0
        fault_entries_.voltage_out_range[VoltageFault_Komodo_0_8v] =
            FAULT_ACTIVE;
        break;
      case VOLTAGE_KOMODO_1V0:  // Voltage out-of-range 1
        fault_entries_.voltage_out_range[VoltageFault_Komodo_1_0v] =
            FAULT_ACTIVE;
        break;
      case VOLTAGE_KOMODO_1V8:  // Voltage out-of-range 2
        fault_entries_.voltage_out_range[VoltageFault_Komodo_1_8v] =
            FAULT_ACTIVE;
        break;
      case VOLTAGE_KOMODO_3V3:  // Voltage out-of-range 3
        fault_entries_.voltage_out_range[VoltageFault_Komodo_3_3v] =
            FAULT_ACTIVE;
        break;
      case VOLTAGE_MOTOR_5V3:  // Voltage out-of-range 4
        fault_entries_.voltage_out_range[VoltageFault_Motor_5_3v] =
            FAULT_ACTIVE;
        break;
      case VOLTAGE_OM_5V3:  // Voltage out-of-range 5
        fault_entries_.voltage_out_range[VoltageFault_OM_5_3v] = FAULT_ACTIVE;
        break;
      case VOLTAGE_LASER_32V:  // Voltage out-of-range 6
        fault_entries_.voltage_out_range[VoltageFault_Laser_32v] = FAULT_ACTIVE;
        break;
      case TEMPERATURE_KOMODO_ERROR:  // Komodo temperature out of range
        fault_entries_.temperature_out_range_komodo = FAULT_ACTIVE;
        break;
      case TEMPERATURE_IOX_ERROR:  // IOX temperature out of range
        fault_entries_.temperature_out_range_iox = FAULT_ACTIVE;
        break;
      case TEMPERATURE_IGUANA_ERROR:  // Iguana temperature out of range
        fault_entries_.temperature_out_range_iguana = FAULT_ACTIVE;
        break;
      case TEMPERATURE_GECKO_ERROR:  // Gecko temperature out of range
        fault_entries_.temperature_out_range_gecko = FAULT_ACTIVE;
        break;
      case CHECKSUM_IGUANA_IOX_COMM_ERROR:  // Iguana/IOX comm crc error
        fault_entries_.iguana_iox_comm_crc_error = FAULT_ACTIVE;
        break;
      case CHECKSUM_IOX_GECKO_COMM_ERROR:  // IOX/Gecko comm crc error
        fault_entries_.iox_gecko_comm_crc_error = FAULT_ACTIVE;
        break;
      case CHECKSUM_GECKO_KOMODO_COMM_ERROR:  // Gecko/Komodo comm crc error
        fault_entries_.gecko_komodo_comm_crc_error = FAULT_ACTIVE;
        break;
      case BOOT_IGUANA_FAILURE:  // Iguana boot failure
        fault_entries_.iguana_boot_failure = FAULT_ACTIVE;
        break;
      case ETH_COMM_PTP_SYNC_ERROR:  // PHY sync failure
        fault_entries_.phy_sync_failure = FAULT_ACTIVE;
        break;
      case ETH_LINK_DOWN:  // Ethernet link down
        fault_entries_.eth_link_down = FAULT_ACTIVE;
        break;
      case ETH_COMM_PTP_EXPIRATION:  // PTP master expired
        fault_entries_.ptp_expiration = FAULT_ACTIVE;
        break;
      case TEMPERATURE_THERMAL_SHUTDOWN:  // Thermal shut down
        fault_entries_.thermal_shut_down = FAULT_ACTIVE;
        break;
      case LASERS_PARTIAL_BLOCKAGE:  // Partial blockage
        fault_entries_.lasers_partial_block = FAULT_ACTIVE;
        break;
      case LASERS_FULL_BLOCKAGE:  // Full blockage
        fault_entries_.lasers_full_block = FAULT_ACTIVE;
        break;
      case MMT_MOTION_INTERLOCK:  // MMT interlock
        fault_entries_.mmt_interlock = FAULT_ACTIVE;
        break;
        /*
        Placeholder: Iguana failure
        */
      case FPGA_RUNTIME_FAILURE:  // FPGA runtime failure
        fault_entries_.fpga_runtime_fail = FAULT_ACTIVE;
        break;
      case MMT_TIMEOUT:  // MMT encoder timeout
        fault_entries_.mmt_timeout = FAULT_ACTIVE;
        break;
      case BOOT_GECKO_FAILURE:  // Gecko boot failure
        fault_entries_.gecko_boot_failure = FAULT_ACTIVE;
        break;
    }
    return get_msg(timestamp, handle);
  }

  cepton2_ros::SensorPanic get_msg(int64_t timestamp,
                                   CeptonSensorHandle handle) {
    auto msg              = cepton2_ros::SensorPanic();
    msg.eth_link_down     = fault_entries_.eth_link_down;
    msg.fpga_runtime_fail = fault_entries_.fpga_runtime_fail;
    msg.gecko_komodo_comm_crc_error =
        fault_entries_.gecko_komodo_comm_crc_error;
    msg.iguana_boot_failure       = fault_entries_.iguana_boot_failure;
    msg.iguana_failure            = fault_entries_.iguana_failure;
    msg.iguana_iox_comm_crc_error = fault_entries_.iguana_iox_comm_crc_error;
    msg.iox_gecko_comm_crc_error  = fault_entries_.iox_gecko_comm_crc_error;
    msg.lasers_full_block         = fault_entries_.lasers_full_block;
    msg.lasers_partial_block      = fault_entries_.lasers_partial_block;
    msg.mmt_interlock             = fault_entries_.mmt_interlock;
    msg.mmt_timeout               = fault_entries_.mmt_timeout;
    msg.phy_sync_failure          = fault_entries_.phy_sync_failure;
    msg.ptp_expiration            = fault_entries_.ptp_expiration;
    msg.temperature_out_range_gecko =
        fault_entries_.temperature_out_range_gecko;
    msg.temperature_out_range_iguana =
        fault_entries_.temperature_out_range_iguana;
    msg.temperature_out_range_iox = fault_entries_.temperature_out_range_iox;
    msg.temperature_out_range_komodo =
        fault_entries_.temperature_out_range_komodo;
    msg.thermal_shut_down  = fault_entries_.thermal_shut_down;
    msg.gecko_boot_failure = fault_entries_.gecko_boot_failure;

    msg.voltage_out_range =
        vector<uint8_t>(fault_entries_.voltage_out_range,
                        std::end(fault_entries_.voltage_out_range));

    msg.timestamp = timestamp;
    msg.handle    = handle;
    return msg;
  }
};
static unordered_map<CeptonSensorHandle, FaultMonitor> fault_monitors_;

void PublisherNodelet::PublishSensorPanic(
    CeptonSensorHandle handle, const CeptonPanicMessage *panic_message) {
  // Create a panic publisher by handle, if not existing
  if (handle_panic_publisher_.find(handle) == handle_panic_publisher_.end()) {
    fault_monitors_[handle] = FaultMonitor();
    auto panic_topic_name   = "cepton2/panic_handle_" + std::to_string(handle);
    handle_panic_publisher_.insert(
        std::pair<CeptonSensorHandle, ros::Publisher>(
            handle, node_handle_.advertise<cepton2_ros::SensorPanic>(
                        panic_topic_name, 10)));
  }
  // Publish the updated panic
  auto msg = fault_monitors_[handle].update(
      panic_message, static_cast<int64_t>(panic_message->ptp_timestamp * 1e-3),
      handle);
  handle_panic_publisher_[handle].publish(msg);
}  // PublisherNodelet::PublishSensorPanic

void PublisherNodelet::PublishSensorInformation(const CeptonSensor *info) {
  {
    lock_guard<mutex> lock(status_lock_);
    handle_to_serial_number_[info->handle] = info->serial_number;
  }

  // Create a points publisher by handle
  if (handle_points_publisher_.find(info->handle) ==
          handle_points_publisher_.end() &&
      output_by_handle_) {
    auto handle_topic_name =
        "cepton2/cepp_handle_" + std::to_string(info->handle);
    handle_points_publisher_.insert(
        std::pair<CeptonSensorHandle, ros::Publisher>(
            info->handle,
            node_handle_.advertise<CeptonPointCloud>(handle_topic_name, 2)));
  }

  // Create a points publisher by serial number
  if (serial_points_publisher_.find(info->handle) ==
          serial_points_publisher_.end() &&
      output_by_sn_) {
    auto sn_topic_name =
        "cepton2/cepp_serial_" + std::to_string(info->serial_number);
    serial_points_publisher_.insert(
        std::pair<CeptonSensorHandle, ros::Publisher>(
            info->handle,
            node_handle_.advertise<CeptonPointCloud>(sn_topic_name, 2)));
  }

  // Create an info publisher by handle
  if (handle_info_publisher_.find(info->handle) ==
      handle_info_publisher_.end()) {
    auto info_topic_name =
        "cepton2/info_handle_" + std::to_string(info->serial_number);
    handle_info_publisher_.insert(std::pair<CeptonSensorHandle, ros::Publisher>(
        info->handle, node_handle_.advertise<cepton2_ros::SensorInformation>(
                          info_topic_name, 2)));
  }

  cepton2_ros::SensorInformation msg;
  msg.header.stamp = ros::Time::now();

  msg.handle             = info->handle;
  msg.serial_number      = info->serial_number;
  msg.model_name         = reinterpret_cast<char const *>(info->model_name);
  msg.model              = info->model;
  msg.part_number        = info->part_number;
  msg.firmware_version   = info->firmware_version;
  msg.power_up_timestamp = info->power_up_timestamp;
  msg.time_sync_offset   = info->time_sync_offset;
  msg.time_sync_drift    = info->time_sync_drift;
  msg.return_count       = info->return_count;
  msg.channel_count      = info->channel_count;
  msg.status_flags       = info->status_flags;
  msg.fault_summary      = info->fault_summary;
  copy(begin(info->fault_entries), end(info->fault_entries),
       msg.fault_entries.begin());

  // Publish on the unified info topic
  sensor_info_publisher_.publish(msg);

  // Publish on the sensor-specific topic
  handle_info_publisher_[info->handle].publish(msg);

  // Create a panic publisher by handle, if not existing
  if (handle_panic_publisher_.find(info->handle) ==
      handle_panic_publisher_.end()) {
    fault_monitors_[info->handle] = FaultMonitor();
    auto panic_topic_name =
        "cepton2/panic_handle_" + std::to_string(info->handle);
    handle_panic_publisher_.insert(
        std::pair<CeptonSensorHandle, ros::Publisher>(
            info->handle, node_handle_.advertise<cepton2_ros::SensorPanic>(
                              panic_topic_name, 10)));
  }
  // Publish the updated panic
  auto panic_message = fault_monitors_[info->handle].update(
      info, info->power_up_timestamp - info->time_sync_offset, info->handle);
  handle_panic_publisher_[info->handle].publish(panic_message);

}  // PublisherNodelet::PublishSensorInformation

void PublisherNodelet::CheckIncludedFlags() {
  bool include  = true;
  include_flag_ = 0;
  printf("============= Point Flag Parameters =============\n");

  private_node_handle_.param("include_saturated_points", include, true);
  include_flag_ |= (include ? CEPTON_POINT_SATURATED : 0);
  printf("Including Saturated points: %s\n", include ? "true" : "false");

  // keep frame low_snr/parity/boundary as internal for now

  private_node_handle_.param("include_second_return_points", include, true);
  include_flag_ |= (include ? CEPTON_POINT_SECOND_RETURN : 0);
  printf("Including Second Return  points: %s\n", include ? "true" : "false");

  private_node_handle_.param("include_invalid_points", include, false);
  include_flag_ |= (include ? CEPTON_POINT_NO_RETURN : 0);
  printf("Including Invalid (No Return) points: %s\n",
         include ? "true" : "false");

  private_node_handle_.param("include_noise_points", include, false);
  include_flag_ |= (include ? CEPTON_POINT_NOISE : 0);
  printf("Including Noise points: %s\n", include ? "true" : "false");

  private_node_handle_.param("include_blocked_points", include, false);
  include_flag_ |= (include ? CEPTON_POINT_BLOCKED : 0);
  printf("Including Blocked points: %s\n", include ? "true" : "false");

  printf("=================================================\n");

  include_flag_ |= CEPTON_POINT_BLOOMING | CEPTON_POINT_FRAME_PARITY |
                   CEPTON_POINT_FRAME_BOUNDARY;

}  // PublisherNodelet::CheckIfInclude
}  // namespace cepton2_ros
