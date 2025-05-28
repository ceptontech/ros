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
  bool subscribePcl2         = false;
  bool subscribeCeptonPoints = false;
  bool subscribeCeptonInfo   = false;
  bool subscribeCeptonPanic  = false;

  declare_parameter("subscribe_pcl2", subscribePcl2);
  declare_parameter("subscribe_cepton_points", subscribeCeptonPoints);
  declare_parameter("subscribe_cepton_info", subscribeCeptonInfo);
  declare_parameter("subscribe_cepton_panic", subscribeCeptonPanic);
  declare_parameter("export_to_csv", export_to_csv_);

  // Check parameter overrides
  rclcpp::Parameter pSubscribePcl2 = get_parameter("subscribe_pcl2");
  if (pSubscribePcl2.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    subscribePcl2 = pSubscribePcl2.as_bool();

  rclcpp::Parameter pSubscribeCeptonPoints =
      get_parameter("subscribe_cepton_points");
  if (pSubscribeCeptonPoints.get_type() !=
      rclcpp::ParameterType::PARAMETER_NOT_SET)
    subscribeCeptonPoints = pSubscribeCeptonPoints.as_bool();

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

  // Subscribe to CeptonPointData
  if (subscribeCeptonPoints)
    ceptonPointsSubscriber =
        create_subscription<cepton_messages::msg::CeptonPointData>(
            "cepton_points", 50,
            bind(&CeptonSubscriber::recv_cep_points, this, placeholders::_1));

  // Subscribe to info messages
  if (subscribeCeptonInfo)
    infoSubscriber =
        create_subscription<cepton_messages::msg::CeptonSensorInfo>(
            "cepton_info", 10,
            bind(&CeptonSubscriber::recv_info, this, placeholders::_1));

  // Subscribe to panic messages
  if (subscribeCeptonPanic)
    panicSubscriber = create_subscription<cepton_messages::msg::CeptonPanic>(
        "cepton_panic", 10,
        bind(&CeptonSubscriber::recv_panic, this, placeholders::_1));
}

void CeptonSubscriber::recv_info(
    const cepton_messages::msg::CeptonSensorInfo::SharedPtr info) {
  printf("Model: %s\tHandle: %u\n", info->model_name.c_str(),
         (uint32_t)info->handle);
}

void CeptonSubscriber::recv_panic(
    const cepton_messages::msg::CeptonPanic::SharedPtr panic) {
  printf("Got panic\n");
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
  sensor_msgs::PointCloud2ConstIterator<int32_t> t_iter(p, "timestamp_s");
  sensor_msgs::PointCloud2ConstIterator<int32_t> t_us_iter(p, "timestamp_us");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> f_iter(p, "flags");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> c_iter(p, "channel_id");
  sensor_msgs::PointCloud2ConstIterator<float> azim_iter(p, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> elev_iter(p, "elevation");
  sensor_msgs::PointCloud2ConstIterator<float> dist_iter(p, "distance");

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
    s << "timestamp,x,y,z,c,flag,azimuth,elevation,distance" << endl;

    for (int i = 0; i < n; ++i, ++x_iter, ++y_iter, ++z_iter, ++i_iter,
             ++t_iter, ++t_us_iter, ++c_iter, ++f_iter, ++azim_iter,
             ++elev_iter, ++dist_iter) {
      s << (*t_iter * 1e6 + *t_us_iter) << "," << *x_iter << "," << *y_iter
        << "," << *z_iter << "," << static_cast<int>(*c_iter) << ","
        << static_cast<int>(*f_iter) << "," << *azim_iter << "," << *elev_iter
        << "," << *dist_iter << endl;
    }
  } else {
    for (int i = 0; i < n; ++i, ++x_iter, ++y_iter, ++z_iter, ++i_iter,
             ++t_iter, ++t_us_iter, ++c_iter) {
      printf(
          "Timestamp(sec): %u\tChannel: %d\tX: %f\tY: %f\tZ: %f\tIntensity: "
          "%f\n",
          *t_iter, *c_iter, *x_iter, *y_iter, *z_iter, *i_iter);
    }
  }
}

// Sample of exporting the published points
void export_csv(cepton_messages::msg::CeptonPointData::SharedPtr pPoints) {
  static int cepFrameNum = 0;

  // Check if directory exists
  const string dir = "cep_frames";
  struct stat info;
  if (stat(dir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR))
    mkdir(dir.c_str(), S_IRWXU);

  auto f = dir + "/" + to_string(cepFrameNum++) + ".csv";
  ofstream s(f);
  s << "timestamp,x,y,z,c,r,flag,azimuth,elevation,distance" << endl;

  int64_t t             = pPoints->start_timestamp;
  const uint8_t* points = pPoints->points.data();
  for (size_t i = 0; i < pPoints->n_points; i++) {
    const struct CeptonPoint* pt = (CeptonPoint*)(points + i * pPoints->stride);
    t += pt->relative_timestamp;
    s << t << "," << pt->x << "," << pt->y << "," << pt->z << ","
      << (int)pt->channel_id << "," << reflectivity_LUT[pt->reflectivity] << ","
      << (int)pt->flags << endl;
  }
}

void CeptonSubscriber::recv_cep_points(
    const cepton_messages::msg::CeptonPointData::SharedPtr points) {
  printf("Handle:%ld\tNPts:%d\tStride:%d\n", points->handle, points->n_points,
         points->stride);

  if (export_to_csv_) export_csv(points);
}
