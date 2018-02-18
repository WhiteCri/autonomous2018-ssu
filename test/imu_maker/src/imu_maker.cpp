#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include "imu/imu_msgs.h"

//"base_frame";
//"camera_right_frame";
#define PI 3.141592
#define R_COVAR 99999.0
#define P_COVAR 99999.0
#define Y_COVAR 1000.0
#define DEG2RAD PI/180.0

static ros::Publisher pub_imu;


//callback function
void Callback(const imu::imu_msgs& imu_raw){
    ros::NodeHandle node;
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw( - imu_raw.yaw * DEG2RAD );

    boost::array<double, 9> covariance = {{
    R_COVAR, 0, 0,
    0, P_COVAR, 0,
    0, 0, Y_COVAR
    }};

    sensor_msgs::Imu imu;
    imu.header.frame_id = "base_link";
    imu.header.stamp = ros::Time::now();
    imu.orientation = quaternion;
    imu.orientation_covariance = covariance;
    pub_imu.publish(imu);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_maker");
    ros::NodeHandle nh;
    pub_imu = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::Subscriber sub = nh.subscribe("/raw/imu", 100, Callback);

    ros::spin();
}
