#include <ros/ros.h>
#include <iostream>
#include "IMUConfigurator.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_main");
    ros::NodeHandle nh;


    ros::Rate loop_rate(IMUFRE);

    IMUConfigurator imu;


    imu.serialCommucation(argv[1]);

    while(ros::ok()){
        imu.RPY(imu.parse(), nh);


        loop_rate.sleep();
    }

}