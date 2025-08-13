#include <cepton_sdk3.h>
#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "cepton_messages/msg/cepton_point_data.hpp"
#include "cepton_messages/msg/cepton_sensor_info.hpp"
#include "cepton_messages/msg/cepton_sensor_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

namespace cepton_ros {

enum SensorStatusFlags : uint32_t { SENSOR_TIMED_OUT = 1 << 0 };

/**
 * @brief Class definition for the CeptonPublisher component
 */
class CeptonPublisher : public rclcpp::Node {
 public:
  using CepPointPublisher =
      rclcpp::Publisher<cepton_messages::msg::CeptonPointData>::SharedPtr;
  using PointPublisher =
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
  using PointPublisherMap =
      std::unordered_map<CeptonSensorHandle, PointPublisher>;
  using CepPointPublisherMap =
      std::unordered_map<CeptonSensorHandle, CepPointPublisher>;
  using InfoPublisher =
      rclcpp::Publisher<cepton_messages::msg::CeptonSensorInfo>::SharedPtr;
  using InfoPublisherMap =
      std::unordered_map<CeptonSensorHandle, InfoPublisher>;
  using StatusPublisher =
      rclcpp::Publisher<cepton_messages::msg::CeptonSensorStatus>::SharedPtr;
  using SerialNumberMap = std::unordered_map<CeptonSensorHandle, uint32_t>;

  CeptonPublisher();
  ~CeptonPublisher();

 private:
  CeptonReplayHandle replay_handle = 0;

  // Edited
  friend void sensorFrameCallback(CeptonSensorHandle handle,
                                  int64_t start_timestamp, size_t n_points,
                                  const struct CeptonPointEx *points,
                                  void *user_data);
  /**
   * @brief Internal points callback invoked by SDK. Publishes points in format
   * CeptonPointDataEx (see cepton_sdk2.h)
   *
   * @param handle
   * @param start_timestamp
   * @param n_points The number of points we need to copy
   * @param stride
   * @param points
   * @param node
   */

  // Edited
  friend void ceptonFrameCallback(CeptonSensorHandle handle,
                                  int64_t start_timestamp, size_t n_points,
                                  const struct CeptonPointEx *points,
                                  void *user_data);

  /**
   * @brief SDK info callback
   *
   * @param handle
   * @param info
   * @param node
   */
  friend void sensor_info_callback(CeptonSensorHandle handle,
                                   const struct CeptonSensor *info, void *node);

  /**
   * @brief Responsible for publishing the points received from the SDK
   * callback functions. Publish points in format msg::PointCloud2
   *
   */
  PointPublisher points_publisher;
  std::unordered_map<CeptonSensorHandle, PointPublisher>
      handle_points_publisher;
  std::unordered_map<CeptonSensorHandle, PointPublisher>
      serial_points_publisher;

  std::unordered_map<CeptonSensorHandle, std::vector<uint8_t>> handle_to_points;
  std::mutex handle_to_points_mutex_;
  std::unordered_map<CeptonSensorHandle, int64_t> handle_to_start_timestamp;

  /**
   * @brief Responsible for publishing the points received from the SDK callback
   * functions. Publish points in format cepton_messages::msg::CeptonPointData
   *
   */
  CepPointPublisher cep_points_publisher;
  CepPointPublisherMap handle_cep_points_publisher;
  CepPointPublisherMap serial_cep_points_publisher;

  /**
   * @brief Responsible for publishing the sensor info packets
   */
  InfoPublisher info_publisher;

  /**
   * @brief Responsible for publishing info messages on per-sensor topics
   */
  InfoPublisherMap handle_to_info_publisher;

  /**
   * @brief Publishes sensor status messages
   */
  StatusPublisher sensor_status_publisher;

  std::thread sensor_status_thread;

  /**
   * @brief Store the last time each sensor received some points.
   */
  std::unordered_map<CeptonSensorHandle,
                     std::chrono::time_point<std::chrono::system_clock>>
      last_points_time_;

  SerialNumberMap handle_to_serial_number_;

  std::mutex status_lock_;

  bool stopping_{false};

  static constexpr auto SENSOR_POINTS_TIMEOUT = std::chrono::seconds(3);

  const rclcpp::NodeOptions options;

  /* Based on config params, bit is flipped as 1 for point flags that should be
    included, while 0 if excluded The pre-included flags are "ignored," either
    because it is for internal use only (hence no config to include them) or
    because it is deprecated.

    Ambient point information exists for all points, the corresponding flag bit
    for ambient point is (1 << 15), hence the inclusion to include_flag_*/
  uint16_t include_flag_ = CEPTON_POINT_BLOOMING | CEPTON_POINT_FRAME_PARITY |
                           CEPTON_POINT_FRAME_BOUNDARY | (1 << 15);

  /** If true, publish cepx points by sensor handle */
  bool use_handle_for_cepx_{true};

  /** If true, publish cepx points by serial number */
  bool use_sn_for_cepx_{true};

  /** If true, publish pcl2 by sensor handle */
  bool use_handle_for_pcl2_{true};

  /** If true, publish pcl2 by serial number */
  bool use_sn_for_pcl2_{true};

  double min_altitude_{-90.};
  double max_altitude_{90.};
  double min_azimuth_{-90.};
  double max_azimuth_{90.};

  double min_image_x_{0.0};
  double max_image_x_{0.0};
  double min_image_z_{0.0};
  double max_image_z_{0.0};

  double min_distance_{0.0};
  double max_distance_{std::numeric_limits<float>::max()};

  void ensure_pcl2_publisher(CeptonSensorHandle handle,
                             std::string const &topic, PointPublisherMap &m);
  void ensure_cepx_publisher(CeptonSensorHandle handle,
                             std::string const &topic, CepPointPublisherMap &m);
  void ensure_info_publisher(CeptonSensorHandle handle,
                             std::string const &topic, InfoPublisherMap &m);
  std::future<void> pub_fut_;

 public:
  void publish_points(CeptonSensorHandle handle);
};

}  // namespace cepton_ros
