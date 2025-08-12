#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <fstream>

#include "cepton_ros/cepton_ros.hpp"
#include "ros/ros.h"

const bool EXPORT_TO_CSV = true;
int cepFrameNum = 0;
using namespace std;

void on_cloud(const cepton_ros::Cloud::ConstPtr& pPoints) {
  // Check if directory exists
  cout << "Got " << pPoints->size() << " points" << endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cepton_subscriber");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber sub =
      n.subscribe<cepton_ros::Cloud>("cepton3/points", 10, on_cloud);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
