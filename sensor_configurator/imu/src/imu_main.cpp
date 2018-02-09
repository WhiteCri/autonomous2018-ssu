#include <ros/ros.h>
#include <iostream>
#include "IMUConfigurator.h"
#include "imu/imu_msgs.h"
int main(int argc, char **argv){
    ros::init(argc, argv, "imu_main");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<imu::imu_msgs>("raw/imu",100);
    
    IMUConfigurator imu;

    if(argc < 3){
        ROS_ERROR("give me [path] [frequency]");
        return -1;
    }
    if(!imu.serialCommucation(argv[1])){
        ROS_ERROR("Serial port error");
        return -1;
    }

    int fre = atoi(argv[2]);
    ros::Rate loop_rate(fre);
    imu::imu_msgs msg;

    while(ros::ok()){
        msg = imu.RPY(imu.parse(), nh);
        pub.publish(msg);
        loop_rate.sleep();
    }

}