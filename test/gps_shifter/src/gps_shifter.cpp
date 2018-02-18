#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
static double easting_shift, northing_shift;
static ros::Publisher pub_shifted_vo;


//callback function
void Callback(const nav_msgs::Odometry& vo){
    ros::NodeHandle node;
    nav_msgs::Odometry vo_shifted;
    vo_shifted = vo;
    node.getParam("/gps/easting_shift", easting_shift);   // easting_shift 됨값 세팅해주면 됨
    node.getParam("/gps/northing_shift", northing_shift); // northing_shift 값 세팅해주면 됨
    vo_shifted.child_frame_id = "map";
    vo_shifted.pose.pose.position.x -= easting_shift;
    vo_shifted.pose.pose.position.y -= northing_shift;
    vo_shifted.pose.pose.position.x = - vo_shifted.pose.pose.position.x;
    vo_shifted.pose.pose.position.y = - vo_shifted.pose.pose.position.y;
    vo_shifted.pose.pose.position.z = 0;
    pub_shifted_vo.publish(vo_shifted);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "gps_shifter");
    ros::NodeHandle nh;
    pub_shifted_vo = nh.advertise<nav_msgs::Odometry>("vo", 10);
    ros::Subscriber sub = nh.subscribe("vo_raw", 100, Callback);

    ros::spin();
}
