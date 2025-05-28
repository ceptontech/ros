// Include ROS
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// Include PCL
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

// Include CeptonSDK2
#include <algorithm>
#include <chrono>
#include <future>
#include <mutex>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include "cepton2_ros/CeptonSensorStatus.h"
#include "cepton2_ros/SensorInformation.h"
#include "cepton2_ros/SensorPanic.h"
#include "cepton2_ros/common.hpp"
#include "cepton2_ros/point.hpp"
#include "cepton_sdk2.h"

enum SensorStatusFlags : uint32_t { SENSOR_TIMED_OUT = 1 << 0 };

namespace cepton2_ros {
/**
 * CEPTON_SDK2 nodelet. Publishes sensor point topics.
 **/
class PublisherNodelet : public nodelet::Nodelet {
 public:
  ~PublisherNodelet();

  static void FrameCallbackWrapper(CeptonSensorHandle handle,
                                   int64_t start_timestamp, size_t n_points,
                                   size_t stride, const uint8_t *points,
                                   void *user_data);

  static void SensorInfoCallbackWrapper(CeptonSensorHandle handle,
                                        const struct CeptonSensor *info,
                                        void *user_data);

  static void SensorPanicCallbackWrapper(
      CeptonSensorHandle handle, const CeptonPanicMessage *panic_message,
      void *user_data);

  void check_api_error(int err, char const *api);

  void PublishPoints(CeptonSensorHandle handle, int64_t start_timestamp,
                     size_t n_points, size_t stride, const uint8_t *points);

  void PublishSensorInformation(const CeptonSensor *info);
  void PublishSensorPanic(CeptonSensorHandle handle,
                          const CeptonPanicMessage *panic_message);

  bool half_frequency_mode() const { return is_half_frequency_mode_; }

 protected:
  void onInit() override;

 private:
  void CheckIncludedFlags();
  CeptonSensorErrorCallback error_callback;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Timer watchdog_timer_;
  ros::Publisher points_publisher_;
  ros::Publisher sensor_info_publisher_;

  std::unordered_map<CeptonSensorHandle, ros::Publisher>
      handle_points_publisher_;
  std::unordered_map<CeptonSensorHandle, ros::Publisher>
      serial_points_publisher_;
  std::unordered_map<CeptonSensorHandle, ros::Publisher> handle_info_publisher_;
  std::unordered_map<CeptonSensorHandle, ros::Publisher>
      handle_panic_publisher_;

  ros::Publisher sensor_status_publisher_;

  uint8_t include_flag_;
  CeptonReplayHandle replay_handle_{0};

  bool is_half_frequency_mode_{false};
  bool output_by_handle_{true};
  bool output_by_sn_{true};

  double min_altitude_{-90.};
  double max_altitude_{90.};
  double min_azimuth_ = {-180};
  double max_azimuth_{180.};

  double min_image_x_{0.0};
  double max_image_x_{0.0};
  double min_image_z_{0.0};
  double max_image_z_{0.0};

  double min_distance_{0.0};
  double max_distance_{std::numeric_limits<float>::max()};

  bool using_cepton_coordinate_system_{true};

  std::vector<std::string> expected_sensor_ips_;

  int half_frequency_publish_parity_{-1};
  int frame_aggregation_mode_{CEPTON_AGGREGATION_MODE_NATURAL};
  std::future<void> pub_fut_;

  void publish_async(CeptonSensorHandle handle, size_t stride);

  std::mutex status_lock_;
  std::unordered_map<CeptonSensorHandle, int64_t> handle_to_start_timestamp_;
  std::unordered_map<CeptonSensorHandle, std::vector<uint8_t>>
      handle_to_points_;
  std::unordered_map<CeptonSensorHandle, CeptonPointCloud>
      rostopic_point_clouds_;
  std::unordered_map<CeptonSensorHandle,
                     std::chrono::time_point<std::chrono::system_clock>>
      last_points_time_;
  std::thread sensor_status_thread;
  std::unordered_map<CeptonSensorHandle, uint32_t> handle_to_serial_number_;
  bool stopping_{false};
};

}  // namespace cepton2_ros
