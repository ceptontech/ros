#include "cepton_publisher.h"

#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>

#include <cstddef> 
#include <algorithm>
#include <iostream>
#include <set>
#include <mutex>

#include "cepton_messages/cepton_messages.h"
#include "panic.h"
#include "sdk_proxy.h"

using PointCloud2  = sensor_msgs::msg::PointCloud2;
using PointField   = sensor_msgs::msg::PointField;
using CeptonPoints = cepton_messages::msg::CeptonPointData;
using namespace std::chrono_literals;
using namespace std;
namespace cepton_ros {
inline void check_sdk_error(int re, const char *msg) {
  if (re != CEPTON_SUCCESS) {
    cout << "Error: " << re << " " << string(msg) << endl;
  }
}

static SdkProxy sdk;

double degrees_to_radians(double t) { return t * M_PI / 180.0; }

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
  cepton_messages::msg::CeptonPanic update(CeptonSensor const *info,
                                           int64_t timestamp,
                                           CeptonSensorHandle handle,
                                           CeptonPublisher *const) {
    // Copy the fault entries
    fault_entries_ =
        *reinterpret_cast<FaultEntries const *>(info->fault_entries);

    // Return the updated message
    return get_msg(timestamp, handle);
  }

  /**
   * Update the fault entries based on the panic packet
   */
  cepton_messages::msg::CeptonPanic update(CeptonPanicMessage const *panic,
                                           int64_t timestamp,
                                           CeptonSensorHandle handle,
                                           CeptonPublisher *const node) {
    RCLCPP_DEBUG(node->get_logger(), "Received fault identity 0x%08x",
                 panic->fault_identity);
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

  cepton_messages::msg::CeptonPanic get_msg(int64_t timestamp,
                                            CeptonSensorHandle handle) {
    auto msg              = cepton_messages::msg::CeptonPanic();
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
  int update(const struct CeptonPointEx *points, size_t num_points) {  
    if (publish_parity != PARITY_NOT_SET) {
      // Calc is finished, return the result
      return publish_parity;
    }
    if (last_frame_sign == SIGN_NOT_SET) {
      // If this is the first frame, just get the sign and return
      last_frame_sign = calc_sign(points, num_points); 
      return PARITY_NOT_SET;
    }
    // Get the sign of the new incoming frame
    int cur_sign = calc_sign(points, num_points); 

    // If the sign changed, then going forward we should publish on this parity
    if (cur_sign != last_frame_sign) {
      // Scan forward to find the first non-boundary parity
      size_t index;
      for (index = 0; index < num_points; index++) {
        CeptonPointEx const &point = points[index];
        if ((point.flags & CEPTON_POINT_FRAME_BOUNDARY) == 0) break;
      }
      CeptonPointEx const &sel_point = points[index];
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

  static int calc_sign(const struct CeptonPointEx *points, size_t num_points) { 
    vector<float> azim;
    vector<float> elev;

    for (size_t i = 0; i < num_points; i++) {
      CeptonPointEx const &point = points[i]; 
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

int get_non_boundary_parity(const struct CeptonPointEx *points, size_t n_points) { 
  for (size_t index = 0; index < n_points; index++) {
    CeptonPointEx const &point = points[index];
    if ((point.flags & CEPTON_POINT_FRAME_BOUNDARY) == 0) {
      return point.flags & CEPTON_POINT_FRAME_PARITY;
    }
  }
  return -1;
}

static unordered_map<CeptonSensorHandle, cepton_messages::msg::CeptonPointData>
    cepton_clouds_;
static unordered_map<CeptonSensorHandle, PointCloud2> sensor_clouds_;
static unordered_map<CeptonSensorHandle, FaultMonitor> fault_monitors_;

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
void ceptonFrameCallback(CeptonSensorHandle handle,
  int64_t start_timestamp,
  size_t n_points,
  const struct CeptonPointEx *points,
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
  memcpy(cpts.points.data(), points, n_points*sizeof(CeptonPointEx)); 

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

void sensorFrameCallback(CeptonSensorHandle handle,
  int64_t start_timestamp,
  size_t n_points,
  const struct CeptonPointEx *points,
  void *user_data) {
  CeptonPublisher *node = reinterpret_cast<CeptonPublisher *>(user_data);

  // Update the sensor status
  {
    lock_guard<mutex> lock(node->status_lock_);

    node->last_points_time_[handle] = chrono::system_clock::now();
  }

  // Update the half-frequency publish parity calc
  static unordered_map<CeptonSensorHandle, HalfFrequencyParityCalc> pc;
  if (pc.find(handle) == pc.end()) pc[handle] = HalfFrequencyParityCalc();
  auto publish_parity = pc[handle].update(points, n_points);  

  auto half_frequency_mode = node->half_frequency_mode();

  // Check if this is a publish frame
  auto const is_publish_frame = 
      get_non_boundary_parity(points, n_points) == publish_parity; 

  // Make sure the points buffer exists
  {
    std::lock_guard<std::mutex> guard(node->handle_to_points_mutex_);
    if (!node->handle_to_points.count(handle))
    node->handle_to_points[handle] = vector<uint8_t>();         

    auto &ref = node->handle_to_points[handle];

    // If we are publishing every frame, or if the last merge was published for
    // half-frequency mode, then start the buffer from scratch
    if (!half_frequency_mode || (half_frequency_mode && !is_publish_frame)) { 
    node->handle_to_start_timestamp[handle] = start_timestamp;
    ref.resize(n_points * sizeof(CeptonPointEx)); 
    memcpy(ref.data(), points, n_points * sizeof(CeptonPointEx)); 
    } else {
    // If in half frequency mode and publishing, then extend the existing buffer
    auto orig_size = ref.size();
    ref.resize(ref.size() + n_points * sizeof(CeptonPointEx)); 
    memcpy(ref.data() + orig_size, points, n_points * sizeof(CeptonPointEx));
    }
  }

  // Return if not publishing
  if (half_frequency_mode && !is_publish_frame) return;

  node->publish_async(handle);  
}

std::mutex data_mutex;
void CeptonPublisher::publish_async(CeptonSensorHandle handle) { 
  if (pub_fut_.valid()) pub_fut_.wait();
  pub_fut_ = std::async(std::launch::async, [this, handle]() { 
    // https://github.com/ros/common_msgs/blob/jade-devel/sensor_msgs/include/sensor_msgs/point_cloud2_iterator.h
    if (sensor_clouds_.count(handle) == 0) {
      sensor_clouds_[handle] = PointCloud2();
    }

    int64_t timestamp = handle_to_start_timestamp[handle];

    auto &cloud            = sensor_clouds_[handle];
    cloud.height           = 1;
    cloud.width            = 4;
    cloud.header.stamp.sec = timestamp / 1'000'000;
    // nanosec is the timestamp portion that is truncated from the sec. portion.
    cloud.header.stamp.nanosec = (timestamp % 1'000'000) * 1'000;

    cloud.header.frame_id = "cepton2";

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
    auto x_iter    = make_optional_iter<float>(cloud, "x");
    auto y_iter    = make_optional_iter<float>(cloud, "y");
    auto z_iter    = make_optional_iter<float>(cloud, "z");
    auto i_iter    = make_optional_iter<float>(cloud, "intensity");
    auto t_iter    = make_optional_iter<int32_t>(cloud, "timestamp_us");
    auto t_us_iter = make_optional_iter<int32_t>(cloud, "timestamp_us");
    auto f_iter    = make_optional_iter<uint16_t>(cloud, "flags");
    auto c_iter    = make_optional_iter<uint16_t>(cloud, "channel_id");
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
        x       = tx;
        y       = ty;
        z       = tz;
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
      auto const &ref       = handle_to_points[handle];
      auto const num_points = ref.size() / sizeof(CeptonPointEx);
      for (unsigned i = 0; i < num_points; i++) {
        CeptonPointEx const &p0 = 
               *reinterpret_cast<CeptonPointEx const *>(ref.data() + i * sizeof(CeptonPointEx)); 
        
        timestamp += p0.relative_timestamp;
        // If point has flags that should not be included (specified by the
        // include_flag), continue
        if ((~(include_flag_) & (p0.flags)) != 0) {
          continue;
        }

        const float x = p0.x * 1.0/65536.0; //0.005; 
        const float y = p0.y * 1.0/65536.0; //0.005; 
        const float z = p0.z * 1.0/65536.0; //0.005; 

        const float distance_squared = x * x + y * y + z * z;
        // Filter out points that are labelled ambient but have invalid distance until point flag definitions are finalized (> 500m for now)
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
        set_fields(x, y, z, reflectivity_LUT[p0.reflectivity],
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

  auto msg               = cepton_messages::msg::CeptonSensorInfo();
  msg.serial_number      = info->serial_number;
  msg.handle             = handle;
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
  msg.temperature        = info->temperature;
  msg.fault_summary      = info->fault_summary;
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

  // Update the panic based on INFZ, and publish
  node->ensure_panic_publisher(handle, "panic_handle_" + to_string(handle));
  auto panic_message = fault_monitors_[handle].update(
      info, info->power_up_timestamp - info->time_sync_offset, handle, node);
  node->handle_to_panic_publisher[handle]->publish(panic_message);
}

void sensor_panic_callback(CeptonSensorHandle handle,
                           const CeptonPanicMessage *panic_message,
                           void *user_data) {
  auto *node = reinterpret_cast<CeptonPublisher *>(user_data);
  node->ensure_panic_publisher(handle, "panic_handle_" + to_string(handle));

  // Publish the per-sensor panics
  auto msg = fault_monitors_[handle].update(
      panic_message,
      static_cast<int64_t>(panic_message->ptp_timestamp *
                           /* nsec to usec */ 1e-3),
      handle, node);

  // Panics are reported immediately when they come in
  node->handle_to_panic_publisher[handle]->publish(msg);
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

void CeptonPublisher::ensure_panic_publisher(CeptonSensorHandle handle,
                                             string const &topic) {
  if (!handle_to_panic_publisher.count(handle)) {
    fault_monitors_[handle] = FaultMonitor();
    handle_to_panic_publisher[handle] =
        create_publisher<cepton_messages::msg::CeptonPanic>(topic, 10);
  }
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

  // Load the SDK shared object library
  try {
    sdk.LoadModule("cepton_sdk2");
  } catch (exception &e) {
    cerr << "LoadModule received exception: " << e.what() << endl;
  }

  // Initialize sdk
  int ret = sdk.Initialize();
  check_sdk_error(ret, "CeptonInitialize");

  // Get the aggregation mode
  rclcpp::Parameter p_aggregation_mode = get_parameter("aggregation_mode");
  frame_aggregation_mode_              = (int)p_aggregation_mode.as_int();
  RCLCPP_DEBUG(this->get_logger(), "Using frame aggregation %d",
               frame_aggregation_mode_);

  RCLCPP_DEBUG(this->get_logger(),
               "========= Point Cloud Output Parameters =========");
  // Check whether to output cepp points
  rclcpp::Parameter pOutputCepp = get_parameter("cepp_output_type");
  std::string cepp_output_type  = pOutputCepp.as_string();
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
    ret = sdk.ListenFrames(frame_aggregation_mode_, ceptonFrameCallback, this);
    check_sdk_error(ret, "CeptonListenFramesEx");
  }

  // Check whether to output pcl2 points
  rclcpp::Parameter pOutputPcl2 = get_parameter("pcl2_output_type");
  std::string pcl2_output_type  = pOutputPcl2.as_string();
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
    ret = sdk.ListenFrames(frame_aggregation_mode_, sensorFrameCallback, this);
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
    ret = sdk.ListenInfo(sensor_info_callback, this);
    check_sdk_error(ret, "CeptonListenSensorInfo");
  }

  // ---------------------------------
  // Set up the sensor panic publishing
  {
    // Register callback
    ret = sdk.ListenPanic(sensor_panic_callback, this);
    check_sdk_error(ret, "CeptonListenSensorPanic");
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
          auto msg          = cepton_messages::msg::CeptonSensorStatus();
          msg.serial_number = serial_number;
          msg.handle        = handle;
          msg.status        = SENSOR_TIMED_OUT;
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
  min_azimuth_  = get_parameter("min_azimuth").as_double();
  max_azimuth_  = get_parameter("max_azimuth").as_double();

  // Clip so that we can safely take the tangent
  min_azimuth_  = max(-89.9, min_azimuth_);
  max_azimuth_  = min(89.9, max_azimuth_);
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
    int flag                          = 0;
    if (capturePlayLoop.get_type() !=
            rclcpp::ParameterType::PARAMETER_NOT_SET &&
        capturePlayLoop.as_bool()) {
      flag |= CEPTON_REPLAY_FLAG_PLAY_LOOPED;
    }
    ret = sdk.ReplayLoadPcap(captureFile.as_string().c_str(), flag,
                             &replay_handle);
    check_sdk_error(ret, "CeptonReplayLoadPcap");
  } else {
    // Start listening for UDP data on the specified port. Default port is
    // 8808
    rclcpp::Parameter port = get_parameter("sensor_port");
    int p                  = 8808;
    if (port.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
      p = port.as_int();
    RCLCPP_DEBUG(this->get_logger(), "Start networking on %d", p);
    sdk.StartNetworkingOnPort(p);
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
  sdk.Deinitialize();
}

}  // namespace cepton_ros
