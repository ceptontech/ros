// Include ROS
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// Include PCL
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <mutex>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include "cepton_ros/CeptonSensorStatus.h"
#include "cepton_ros/SensorInformation.h"
#include "cepton_ros/SensorPanic.h"
#include "cepton_ros/cepton_ros.hpp"
#include "cepton_sdk3.h"

enum SensorStatusFlags : uint32_t { SENSOR_TIMED_OUT = 1 << 0 };

namespace cepton_ros {

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

  void check_api_error(int err, char const *api);

  void publish_points(CeptonSensorHandle handle, int64_t start_timestamp,
                      size_t n_points, const CeptonPointEx *points);

  void publish_sensor_info(const CeptonSensor *info);

 protected:
  void onInit() override;

 private:
  /** ROS node handle for public fields */
  ros::NodeHandle node_handle_;

  /** ROS node handle for private fields */
  ros::NodeHandle private_node_handle_;

  /** Timer to watch for shutdown condition */
  ros::Timer watchdog_timer_;

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
   * Flag that is populated by settings, telling the nodelet which points
   * should be included in the output messages
   */
  // uint8_t include_flag_;
  /* Based on config params, bit is flipped as 1 for point flags that should be
  included, while 0 if excluded The pre-included flags are "ignored," either
  because it is for internal use only (hence no config to include them) or
  because it is deprecated.

  Ambient point information exists for all points, the corresponding flag bit
  for ambient point is (1 << 15), hence the inclusion to include_flag_*/
  uint16_t include_flag_ = CEPTON_POINT_BLOOMING | CEPTON_POINT_FRAME_PARITY |
                           CEPTON_POINT_FRAME_BOUNDARY | (1 << 15);
  CeptonReplayHandle replay_handle_{0};

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

  /** Tangent limits (derived from azimuth and altitude filter) */
  double min_image_x_{0.0};
  double max_image_x_{0.0};
  double min_image_z_{0.0};
  double max_image_z_{0.0};

  double min_distance_{0.0};
  double max_distance_{std::numeric_limits<float>::max()};

  /** Optional set of expected IPs. Useful for detecting time-out */
  std::vector<std::string> expected_sensor_ips_;

  /** Future for the publish process */
  std::future<void> pub_fut_;

  void publish_async(CeptonSensorHandle handle);

  std::mutex status_lock_;

  /** Monitor the starting timestamp of each frame */
  TimerMap last_points_time_;

  /** Status monitor thread */
  std::thread sensor_status_thread;

  /** Store the handle to serial number mappings */
  SerialNumberMap handle_to_serial_number_;

  bool stopping_{false};
};

}  // namespace cepton_ros
