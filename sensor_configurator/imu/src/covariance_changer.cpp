#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
sensor_msgs::Imu imu_msg;

void imuSubCallback(sensor_msgs::Imu::ConstPtr& ptr){
    imu_msg = *ptr;
}

int main(){
    ros::init(argc, argv, "imu_main");
    ros::NodeHandle nh;
    ros::Subscriber sub_imu = nh.advertise<sensor_msgs::Imu>("raw/imu",100);
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("test_imu",100);
    while(ros::ok()){
        ros::spinOnce();
        nh.getParam("imu_test_yaw_cov",imu_msg.orientation_covariance[8]);
        pub_imu.publish(imu_msg);
    }
}