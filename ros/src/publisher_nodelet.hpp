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

 public:
  ~PublisherNodelet();

  void check_api_error(int err, char const *api);

  void publish_points(CeptonSensorHandle handle, int64_t start_timestamp,
                      size_t n_points, const CeptonPointEx *points);

  void publish_sensor_info(const CeptonSensor *info);

 protected:
  void onInit() override;

 private:
  CeptonSensorErrorCallback error_callback;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Timer watchdog_timer_;
  ros::Publisher points_publisher_;
  ros::Publisher sensor_info_publisher_;

  PublisherMap handle_points_publisher_;
  PublisherMap serial_points_publisher_;
  PublisherMap handle_info_publisher_;
  PublisherMap handle_panic_publisher_;

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

  void publish_async(CeptonSensorHandle handle);

  std::mutex status_lock_;
  std::unordered_map<CeptonSensorHandle, int64_t> handle_to_start_timestamp_;
  std::unordered_map<CeptonSensorHandle, std::vector<CeptonPointEx>>
      handle_to_points_;
  std::unordered_map<CeptonSensorHandle, cepton_ros::Cloud>
      rostopic_point_clouds_;
  std::unordered_map<CeptonSensorHandle,
                     std::chrono::time_point<std::chrono::system_clock>>
      last_points_time_;
  std::thread sensor_status_thread;
  std::unordered_map<CeptonSensorHandle, uint32_t> handle_to_serial_number_;
  bool stopping_{false};
};

}  // namespace cepton_ros
