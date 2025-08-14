#include "cepton_subscriber.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iostream>

#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std;
using namespace std::chrono;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

#pragma pack(push, 1)
struct CeptonPointEx {
  int32_t x;
  int32_t y;
  int32_t z;
  uint16_t reflectivity;
  uint16_t relative_timestamp;
  uint16_t flags;
  uint16_t channel_id;
};
#pragma pack(pop)

CeptonSubscriber::CeptonSubscriber() : Node("cepton_subscriber") {
  bool subscribePcl2 = false;
  bool subscribeCeptonInfo = false;
  bool subscribeCeptonPanic = false;

  declare_parameter("subscribe_pcl2", subscribePcl2);
  declare_parameter("subscribe_cepton_info", subscribeCeptonInfo);
  declare_parameter("subscribe_cepton_panic", subscribeCeptonPanic);
  declare_parameter("export_to_csv", export_to_csv_);

  // Check parameter overrides
  rclcpp::Parameter pSubscribePcl2 = get_parameter("subscribe_pcl2");
  if (pSubscribePcl2.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    subscribePcl2 = pSubscribePcl2.as_bool();

  rclcpp::Parameter pSubscribeCeptonInfo =
      get_parameter("subscribe_cepton_info");
  if (pSubscribeCeptonInfo.get_type() !=
      rclcpp::ParameterType::PARAMETER_NOT_SET)
    subscribeCeptonInfo = pSubscribeCeptonInfo.as_bool();

  rclcpp::Parameter pExportCsv = get_parameter("export_to_csv");
  if (pExportCsv.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    export_to_csv_ = pExportCsv.as_bool();

  // Subscribe to PointCloud2
  if (subscribePcl2)
    pointsSubscriber = create_subscription<PointCloud2>(
        "cepton_pcl2", 10,
        bind(&CeptonSubscriber::recv_points, this, placeholders::_1));

  // Subscribe to info messages
  if (subscribeCeptonInfo)
    infoSubscriber =
        create_subscription<cepton_messages::msg::CeptonSensorInfo>(
            "cepton_info", 10,
            bind(&CeptonSubscriber::recv_info, this, placeholders::_1));
}

void CeptonSubscriber::recv_info(
    const cepton_messages::msg::CeptonSensorInfo::SharedPtr info) {
  printf("Model: %s\tHandle: %u\n", info->model_name.c_str(),
         (uint32_t)info->handle);
}

void CeptonSubscriber::recv_points(
    const sensor_msgs::msg::PointCloud2::SharedPtr points) {
  sensor_msgs::PointCloud2Modifier mod(*points);
  int n = mod.size();

  auto p = *points;
  sensor_msgs::PointCloud2ConstIterator<float> x_iter(p, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_iter(p, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_iter(p, "z");
  sensor_msgs::PointCloud2ConstIterator<float> i_iter(p, "intensity");

  // You can add additional iterators depending on which fields the publisher
  // was compiled with

  // Check if directory exists
  const string dir = "frames";
  if (export_to_csv_) {
    struct stat info;
    if (stat(dir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR))
      mkdir(dir.c_str(), S_IRWXU);
  }

  if (export_to_csv_) {
    static int frameNum = 0;
    cout << "Exporting to " << frameNum << endl;
    auto f = dir + "/" + to_string(frameNum++) + ".csv";
    ofstream s(f);
    s << "x,y,z,intensity" << endl;

    for (int i = 0; i < n; ++i, ++x_iter, ++y_iter, ++z_iter, ++i_iter) {
      s << *x_iter << "," << *y_iter << "," << *z_iter << "," << *i_iter
        << endl;
    }
  } else {
    for (int i = 0; i < n; ++i, ++x_iter, ++y_iter, ++z_iter, ++i_iter) {
      printf("X: %f\tY: %f\tZ: %f\tIntensity: %f\n", *x_iter, *y_iter, *z_iter,
             *i_iter);
    }
  }
}
