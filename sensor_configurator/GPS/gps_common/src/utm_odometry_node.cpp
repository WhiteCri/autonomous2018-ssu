/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

using namespace gps_common;
static double base_longitude = 126.7689;
static double base_latitude = 37.2323;
static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
static double rot_cov;
//
static double easting_shift, northing_shift;
//

void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  ros::NodeHandle node;
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;
//  bool parameter;
//  node.getParam("GPS_Odometry_initial",parameter);

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

//
    odom.child_frame_id = child_frame_id;
    node.getParam("/gps/easting_shift", easting_shift);   // easting_shift 됨값 세팅해주면 됨
    node.getParam("/gps/northing_shift", northing_shift); // northing_shift 값 세팅해주면 됨
    odom.pose.pose.position.x = easting - easting_shift;
    odom.pose.pose.position.y = northing - northing_shift;
//


//  if(parameter)
//  { odom.pose.pose.position.x = easting;
//    odom.pose.pose.position.y = northing;}
//  else
//  { odom.pose.pose.position.x = easting - 319000;
//    odom.pose.pose.position.y = northing - 4151000; }

    odom.pose.pose.position.z = fix->altitude;
    
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    odom_pub.publish(odom);
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle priv_node("~");
  ros::NodeHandle node;
  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
  odom_pub = node.advertise<nav_msgs::Odometry>("vo", 10);

  ros::Subscriber fix_sub = node.subscribe("fix", 10, callback);

  ros::spin();
}

