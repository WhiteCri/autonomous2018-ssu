#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
sensor_msgs::Imu imu_msg;

void imuSubCallback(const sensor_msgs::Imu::ConstPtr& ptr){
    imu_msg = *ptr;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "imu_main");
    ros::NodeHandle nh;
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("raw/imu",100,&imuSubCallback);
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("test_imu",100);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        double yaw_cov;
        nh.getParam("imu_test_yaw_cov",imu_msg.orientation_covariance[8]);
        ROS_INFO("yaw_cov : %.2lf",imu_msg.orientation_covariance[8]);
        pub_imu.publish(imu_msg);
        loop_rate.sleep();
    }
}