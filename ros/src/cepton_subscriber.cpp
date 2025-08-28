#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <fstream>

#include "cepton2_ros/point.hpp"
#include "cepton_sdk2.h"
#include "ros/ros.h"
const bool EXPORT_TO_CSV = true;
int cepFrameNum          = 0;
using namespace std;
void cloudCallback(const cepton2_ros::CeptonPointCloud::ConstPtr& pPoints) {
  // Check if directory exists
  const string dir = "cep_frames";
  struct stat info;
  if (stat(dir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR))
    mkdir(dir.c_str(), S_IRWXU);

  auto f = dir + "/" + to_string(cepFrameNum++) + ".csv";
  ofstream s(f);
  s << "timestamp,x,y,z,c,flag,azimuth,elevation,distance" << endl;

  int64_t t = pPoints->header.stamp;
  for (size_t i = 0; i < pPoints->points.size(); i++) {
    const auto pt  = pPoints->points[i];
    t             += pt.relative_timestamp;
    s << t << "," << pt.x << "," << pt.y << "," << pt.z << ","
      << (int)pt.channel_id << "," << (int)pt.flags << endl;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cepton_subscriber");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber sub = n.subscribe<cepton2_ros::CeptonPointCloud>(
      "cepton2/points", 10, cloudCallback);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
