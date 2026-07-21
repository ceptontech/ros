// Lightweight arrival-time probe for PointCloud2 topics (ROS1).
//
// Subscribes to the given topics and appends one CSV line per message:
//   arrival_sec (steady clock), stamp_sec (header.stamp), width
// The point payload is never touched beyond the (already deserialized)
// message header, so the probe keeps up with multi-Gbps point cloud
// traffic and does not apply backpressure to the publisher under test.
//
// Usage:
//   rosrun stability_probe stability_probe_node \
//     _output_dir:=/path/to/out /topic_a /topic_b ...

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

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

class TopicRecorder {
 public:
  TopicRecorder(ros::NodeHandle& nh, const std::string& topic,
                const std::string& out_dir) {
    const std::string path = out_dir + "/sensor_" + sanitize(topic) + ".csv";
    file_ = std::fopen(path.c_str(), "w");
    if (!file_) {
      ROS_FATAL("cannot open %s", path.c_str());
      ros::shutdown();
      return;
    }
    std::setvbuf(file_, nullptr, _IOFBF, 1 << 20);
    std::fprintf(file_, "arrival_sec,stamp_sec,width\n");
    sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
        topic, 100, &TopicRecorder::callback, this,
        ros::TransportHints().tcpNoDelay());
    ROS_INFO("probing %s -> %s", topic.c_str(), path.c_str());
  }

  ~TopicRecorder() {
    if (file_) std::fclose(file_);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Take the arrival time first, before any other work.
    const double arrival = steady_now_sec();
    std::fprintf(file_, "%.9f,%.9f,%u\n", arrival, msg->header.stamp.toSec(),
                 msg->width);
  }

  void flush() {
    if (file_) std::fflush(file_);
  }

 private:
  std::FILE* file_ = nullptr;
  ros::Subscriber sub_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "stability_probe");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string output_dir;
  pnh.param<std::string>("output_dir", output_dir, ".");

  // Topics are the remaining non-ROS command line arguments.
  std::vector<std::string> args;
  ros::removeROSArgs(argc, argv, args);

  std::vector<std::unique_ptr<TopicRecorder>> recorders;
  for (size_t i = 1; i < args.size(); ++i)
    recorders.emplace_back(new TopicRecorder(nh, args[i], output_dir));
  if (recorders.empty()) {
    ROS_FATAL(
        "usage: stability_probe_node _output_dir:=DIR TOPIC [TOPIC ...]");
    return 2;
  }

  // Flush once per second so data survives an abrupt stop.
  ros::Timer flush_timer =
      nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent&) {
        for (auto& r : recorders) r->flush();
      });

  // One spinner thread per topic so a burst on one topic does not delay the
  // arrival timestamps of the others. (Callbacks of the same subscription
  // are still serialized by roscpp.)
  ros::AsyncSpinner spinner(static_cast<uint32_t>(recorders.size()));
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
