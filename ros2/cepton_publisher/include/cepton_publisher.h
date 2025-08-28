#define _USE_MATH_DEFINES

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

#include "cepton_messages/msg/cepton_panic.hpp"
#include "cepton_messages/msg/cepton_point_data.hpp"
#include "cepton_messages/msg/cepton_sensor_info.hpp"
#include "cepton_messages/msg/cepton_sensor_status.hpp"
#include "cepton_sdk2.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

enum PointFieldOptions : uint16_t {
  CARTESIAN = 1 << 0,
  SPHERICAL = 1 << 1,
  TIMESTAMP = 1 << 2,
  INTENSITY = 1 << 3,
  ALL       = 0xFFFF
};

namespace cepton_ros {

enum SensorStatusFlags : uint32_t { SENSOR_TIMED_OUT = 1 << 0 };

/**
 * @brief Class definition for the CeptonPublisher component
 */
class CeptonPublisher : public rclcpp::Node {
 public:
  typedef rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      PointPublisher;
  typedef rclcpp::Publisher<cepton_messages::msg::CeptonPointData>::SharedPtr
      CepPointPublisher;
  typedef rclcpp::Publisher<cepton_messages::msg::CeptonSensorInfo>::SharedPtr
      CepInfoPublisher;
  typedef rclcpp::Publisher<cepton_messages::msg::CeptonPanic>::SharedPtr
      PanicPublisher;

  CeptonPublisher();
  ~CeptonPublisher();
  bool use_handle_for_cepp() { return use_handle_for_cepp_; }
  bool use_sn_for_cepp() { return use_sn_for_cepp_; }
  bool use_handle_for_pcl2() { return use_handle_for_pcl2_; }
  bool use_sn_for_pcl2() { return use_sn_for_pcl2_; }
  uint8_t include_flag() { return include_flag_; }
  bool half_frequency_mode() { return half_frequency_mode_; }
  double min_altitude() { return min_altitude_; }
  double max_altitude() { return max_altitude_; }
  double min_azimuth() { return min_azimuth_; }
  double max_azimuth() { return max_azimuth_; }

 private:
  CeptonReplayHandle replay_handle = 0;

  friend void sensorFrameCallback(CeptonSensorHandle handle,
                                  int64_t start_timestamp, size_t n_points,
                                  size_t stride, const uint8_t *points,
                                  void *node);
  /**
   * @brief Internal points callback invoked by SDK. Publishes points in format
   * CeptonPointData (see cepton_sdk2.h)
   *
   * @param handle
   * @param start_timestamp
   * @param n_points
   * @param stride
   * @param points
   * @param node
   */
  friend void ceptonFrameCallback(CeptonSensorHandle handle,
                                  int64_t start_timestamp, size_t n_points,
                                  size_t stride, const uint8_t *points,
                                  void *node);

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
   * @brief SDK info callback
   *
   * @param handle
   * @param panic_message
   * @param node
   */
  friend void sensor_panic_callback(CeptonSensorHandle handle,
                                    const CeptonPanicMessage *panic_message,
                                    void *node);
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
  std::unordered_map<CeptonSensorHandle, int64_t> handle_to_start_timestamp;

  /**
   * @brief Responsible for publishing the points received from the SDK callback
   * functions. Publish points in format cepton_messages::msg::CeptonPointData
   *
   */
  CepPointPublisher cep_points_publisher;
  std::unordered_map<CeptonSensorHandle, CepPointPublisher>
      handle_cep_points_publisher;
  std::unordered_map<CeptonSensorHandle, CepPointPublisher>
      serial_cep_points_publisher;

  /**
   * @brief Responsible for publishing the sensor info packets
   */
  CepInfoPublisher info_publisher;

  /**
   * @brief Responsible for publishing info messages on per-sensor topics
   */
  std::unordered_map<CeptonSensorHandle, CepInfoPublisher>
      handle_to_info_publisher;

  /**
   * @brief Responsible for publishing the per-sensor panic packets
   */
  std::unordered_map<CeptonSensorHandle, PanicPublisher>
      handle_to_panic_publisher;

  /**
   * @brief Publishes sensor status messages
   */
  rclcpp::Publisher<cepton_messages::msg::CeptonSensorStatus>::SharedPtr
      sensor_status_publisher;

  std::thread sensor_status_thread;

  /**
   * @brief Store the last time each sensor received some points.
   */
  std::unordered_map<CeptonSensorHandle,
                     std::chrono::time_point<std::chrono::system_clock>>
      last_points_time_;

  std::unordered_map<CeptonSensorHandle, uint32_t> handle_to_serial_number_;

  std::mutex status_lock_;

  bool stopping_{false};

  static constexpr auto SENSOR_POINTS_TIMEOUT = std::chrono::seconds(3);

  const rclcpp::NodeOptions options;

  /* Based on config params, bit is flipped as 1 for point flags that should be
    included, while 0 if excluded The pre-included flags are "ignored," either
    because it is for internal use only (hence no config to include them) or
    because it is deprecated.*/
  uint8_t include_flag_ = CEPTON_POINT_BLOOMING | CEPTON_POINT_FRAME_PARITY |
                          CEPTON_POINT_FRAME_BOUNDARY;
  bool use_handle_for_cepp_{true};
  bool use_sn_for_cepp_{true};
  bool use_handle_for_pcl2_{true};
  bool use_sn_for_pcl2_{true};

  bool half_frequency_mode_{false};

  int frame_aggregation_mode_{CEPTON_AGGREGATION_MODE_NATURAL};

  double min_altitude_{-90};
  double max_altitude_{90};
  double min_azimuth_{-180};
  double max_azimuth_{180};

  double min_image_x_{0.0};
  double max_image_x_{0.0};
  double min_image_z_{0.0};
  double max_image_z_{0.0};

  double min_distance_{0.0};
  double max_distance_{std::numeric_limits<float>::max()};

  void ensure_pcl2_publisher(
      CeptonSensorHandle handle, std::string const &topic,
      std::unordered_map<CeptonSensorHandle, PointPublisher> &m);
  void ensure_cepp_publisher(
      CeptonSensorHandle handle, std::string const &topic,
      std::unordered_map<CeptonSensorHandle, CepPointPublisher> &m);
  void ensure_info_publisher(
      CeptonSensorHandle handle, std::string const &topic,
      std::unordered_map<CeptonSensorHandle, CepInfoPublisher> &m);
  void ensure_panic_publisher(CeptonSensorHandle handle,
                              std::string const &topic);
  bool using_cepton_coordinate_system_{true};

 private:
  /**
   * Not yet fully implemented; leaving the std::optional code in place
   * because x120 ultra will have much higher-volume data and we may want to
   * reduce the packet size, or use only certain fields.
   */
  uint32_t point_field_options_ = ALL;

 public:
  /**
   * @brief Helper method - create an optional iterator for a field.
   * The optional will be nullopt if the flag isn't set in the point field
   * options
   */
  template <typename T>
  std::optional<sensor_msgs::PointCloud2Iterator<T>> make_optional_iter(
      sensor_msgs::msg::PointCloud2 &cloud, std::string const &name) {
    // Store a cached value for each field name
    static std::unordered_map<std::string, bool> lookup_cache;

    // If we've not checked for this field yet, search the cloud names
    if (!lookup_cache.count(name))
      lookup_cache[name] =
          cloud.fields.end() !=
          std::find_if(cloud.fields.begin(), cloud.fields.end(),
                       [&name](const sensor_msgs::msg::PointField &field) {
                         return field.name == name;
                       });

    // Return a value if the field was found, else return nullopt
    return lookup_cache[name]
               ? std::make_optional(
                     sensor_msgs::PointCloud2Iterator<T>(cloud, name))
               : std::nullopt;
  }

  void publish_async(CeptonSensorHandle handle, size_t stride);

  std::future<void> pub_fut_;
};

}  // namespace cepton_ros
