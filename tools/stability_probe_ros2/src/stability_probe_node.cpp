// Lightweight arrival-time probe for PointCloud2 topics (ROS2).
//
// Subscribes to the given topics and appends one CSV line per message:
//   arrival_sec (steady clock), stamp_sec (header.stamp), width
// The point payload is never touched beyond the (already deserialized)
// message header, so the probe keeps up with multi-Gbps point cloud
// traffic and does not apply backpressure to the publisher under test.
//
// Usage:
//   ros2 run stability_probe stability_probe --ros-args \
//     -p output_dir:=/path/to/out -p "topics:=[/topic_a,/topic_b]"

#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

double steady_now_sec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string sanitize(const std::string& topic) {
  std::string s = topic;
  while (!s.empty() && s.front() == '/') s.erase(s.begin());
  for (auto& c : s)
    if (c == '/') c = '_';
  return s;
}

}  // namespace

class StabilityProbe : public rclcpp::Node {
 public:
  StabilityProbe() : Node("stability_probe") {
    declare_parameter<std::vector<std::string>>("topics",
                                                std::vector<std::string>{});
    declare_parameter<std::string>("output_dir", ".");

    const auto topics = get_parameter("topics").as_string_array();
    const auto out_dir = get_parameter("output_dir").as_string();
    if (topics.empty()) {
      RCLCPP_FATAL(get_logger(),
                   "parameter 'topics' must list at least one topic");
      throw std::runtime_error("no topics given");
    }

    for (const auto& topic : topics) {
      const std::string path = out_dir + "/sensor_" + sanitize(topic) + ".csv";
      std::FILE* file = std::fopen(path.c_str(), "w");
      if (!file) {
        RCLCPP_FATAL(get_logger(), "cannot open %s", path.c_str());
        throw std::runtime_error("cannot open output file");
      }
      std::setvbuf(file, nullptr, _IOFBF, 1 << 20);
      std::fprintf(file, "arrival_sec,stamp_sec,width\n");
      files_.push_back(file);

      subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
          topic, rclcpp::QoS(100),
          [file](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            // Take the arrival time first, before any other work.
            const double arrival = steady_now_sec();
            const double stamp = msg->header.stamp.sec +
                                 msg->header.stamp.nanosec * 1e-9;
            std::fprintf(file, "%.9f,%.9f,%u\n", arrival, stamp, msg->width);
          }));
      RCLCPP_INFO(get_logger(), "probing %s -> %s", topic.c_str(),
                  path.c_str());
    }

    // Flush once per second so data survives an abrupt stop.
    flush_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      for (auto* f : files_) std::fflush(f);
    });
  }

  ~StabilityProbe() override {
    for (auto* f : files_) std::fclose(f);
  }

 private:
  std::vector<std::FILE*> files_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      subs_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StabilityProbe>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
