#include "odometry/ackermann.h"
using namespace odometry_ackermann;

int main(int argc, char** argv){
  ros::init(argc, argv, "Odometry_Publisher");
  ros::NodeHandle nh;

  Odometry odom;
  ros::Time now = ros::Time::now();
  odom.init(now);

  ROS_INFO("Subscribe START!");
  ros::Subscriber sub = nh.subscribe("raw/platform_rx", 100, &Odometry::callback, &odom);
  ROS_INFO("Subscribe FINISHED!");
  
  ros::spin();
  return 0;
}