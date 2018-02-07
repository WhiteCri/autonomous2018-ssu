#include <ros/ros.h>
#include <iostream>
#include "IMUConfigurator.h"
#include "imu/IMU_data.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_main");
    ros::NodeHandle nh;

    ros::Publisher imu_data_pub = nh.advertise<imu::IMU_data>("raw/imu", 100);

    ros::Rate loop_rate(IMUFRE);

    IMUConfigurator imu;

    imu.serialCommucation(argv[1]);

    while(ros::ok()){
        imu.RPY(imu.parse());

        imu_data_pub.publish(imu.msg);

        loop_rate.sleep();
    }

}