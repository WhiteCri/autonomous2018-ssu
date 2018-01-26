//Header for Ros
#include"ros/ros.h"
#include"sensor_msgs/NavSatFix.h"
#include"geometry_msgs/Pose.h"
//Use String 함수
#include<string>
#include<vector>

void msgCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
 
    ros::NodeHandle nh;
    ros::Publisher caculate_publisher = nh.advertise<geometry_msgs::Pose>("Pose",100);
    geometry_msgs::Pose mypose;

    double mylongitude = msg->longitude;
    double mylatitude = msg->latitude;

    // Publish하기 위한 변수선언
    ROS_INFO("%lf",mylongitude);
    
    return;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "GPS_transformer");
    ros::NodeHandle nh;
    ros::Subscriber nmea_subscriber = nh.subscribe("fix",100,msgCallback);
    ros::Publisher caculate_publisher = nh.advertise<geometry_msgs::Pose>("Pose",100);
  
    ros::spin();
         
    return 0;
}