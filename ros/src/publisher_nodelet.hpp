// Include ROS
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// Include PCL
#include <pcl_ros/point_cloud.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "cepton_ros/CeptonSensorStatus.h"
#include "cepton_ros/SensorInformation.h"
#include "cepton_ros/cepton_ros.hpp"
#include "cepton_sdk3.h"

enum SensorStatusFlags : uint32_t { SENSOR_TIMED_OUT = 1 << 0 };

namespace cepton_ros {

// update this when making changes, will display in terminal running publisher
const std::string VERSION = "v2.1.0";

/**
 * Cepton SDK nodelet. Publishes sensor point topics.
 **/
class PublisherNodelet : public nodelet::Nodelet {
  using PublisherMap = std::unordered_map<CeptonSensorHandle, ros::Publisher>;
  using TimerMap =
      std::unordered_map<CeptonSensorHandle,
                         std::chrono::time_point<std::chrono::system_clock>>;
  using SerialNumberMap = std::unordered_map<CeptonSensorHandle, uint32_t>;

 public:
  ~PublisherNodelet();

  void check_api_error(int err, char const* api);

  void publish_points(CeptonSensorHandle handle, int64_t start_timestamp,
                      size_t n_points, const CeptonPointEx* points);

  void publish_sensor_info(const CeptonSensor* info);

 protected:
  void onInit() override;

 private:
  /** ROS node handle for public fields */
  ros::NodeHandle node_handle_;

  /** ROS node handle for private fields */
  ros::NodeHandle private_node_handle_;

  /** Timer to watch for replay shutdown condition */
  ros::Timer watchdog_timer_;

  /** Timer to report sensors which stopped producing points */
  ros::Timer sensor_status_timer_;

  /** Publisher for sending points of all lidars */
  ros::Publisher points_publisher_;

  /** Publisher for sending points of all sensor infos */
  ros::Publisher sensor_info_publisher_;

  /** Publisher map for the per-sensor points (named by sensor handle) */
  PublisherMap handle_points_publisher_;

  /** Publisher map for the per-sensor points (named by serial number) */
  PublisherMap serial_points_publisher_;

  /** Publisher map for the per-sensor points (named by sensor handle) */
  PublisherMap handle_info_publisher_;

  /** Publisher for the status messages */
  ros::Publisher sensor_status_publisher_;

  /**
   * Flag that is populated by settings, telling the nodelet which points should
   * be included in the output messages.
   *
   * Based on config params; bit is flipped set as 1 for point flags which
   * should be included, or 0 if excluded. The pre-included flags are "ignored",
   * either because it is for internal use only, or because it is deprecated.
   */
  uint16_t include_flag_ = CEPTON_POINT_BLOOMING | CEPTON_POINT_FRAME_PARITY |
                           CEPTON_POINT_FRAME_BOUNDARY;

  CeptonReplayHandle replay_handle_{0};

  /**
   * Store network source information (ip, port, multicast_group) for cleanup
   * on shutdown
   */
  struct NetworkSource {
    std::string ip;
    uint16_t port;
    std::string multicast_group;
  };

  struct PointFilter {
    float min_distance_squared{0.0F};
    float max_distance_squared{std::numeric_limits<float>::max()};
    float min_image_x{0.0F};
    float max_image_x{0.0F};
    float min_image_z{0.0F};
    float max_image_z{0.0F};
    uint16_t include_flags{0};
  };

  struct SensorCloudState {
    cepton_ros::Cloud cloud;
    bool first_frame{true};
  };

  std::vector<NetworkSource> networking_sources_;

  /** If set to true, the nodelet will advertise topics by sensor handle */
  bool output_by_handle_{true};

  /** If set to true, the nodelet will advertise topics by serial number */
  bool output_by_sn_{true};

  /** Altitude filter, in degrees */
  double min_altitude_{-90.};
  double max_altitude_{90.};

  /** Azimuth filter, in degrees */
  double min_azimuth_ = {-90.0};
  double max_azimuth_{90.};

  /** The minimum point distance to keep (meters) */
  float min_distance_{0.0};
  /** The maximum point distance to keep (meters) */
  float max_distance_{std::numeric_limits<float>::max()};

  bool aggregate_frames_{false};

  /** Optional set of expected IPs. Useful for detecting time-out */
  std::vector<std::string> expected_sensor_ips_;

  PointFilter point_filter_;

  std::unordered_map<CeptonSensorHandle, SensorCloudState> sensor_clouds_;
  std::mutex sensor_data_lock_;

  void load_parameters(std::string& capture_path, bool& capture_loop,
                       std::vector<std::string>& sensor_network_sources);
  void configure_point_flags();
  void initialize_publishers();
  void initialize_sdk();
  void configure_input(const std::string& capture_path, bool capture_loop,
                       const std::vector<std::string>& sensor_network_sources);
  void register_sdk_callbacks();
  void start_replay_watchdog(const std::string& capture_path);
  void start_status_monitor();
  void register_expected_sensors();
  void report_timed_out_sensors(const ros::TimerEvent&);

  static bool parse_network_source(const std::string& source,
                                   NetworkSource& result);
  bool add_networking_source(const NetworkSource& source);
  void extend_from_points(cepton_ros::Cloud& cloud, int64_t start_timestamp,
                          size_t n_points, const CeptonPointEx* points,
                          bool first) const;

  std::mutex status_lock_;

  /** Monitor the starting timestamp of each frame */
  TimerMap last_points_time_;

  /** Store the handle to serial number mappings */
  SerialNumberMap handle_to_serial_number_;
};

}  // namespace cepton_ros
