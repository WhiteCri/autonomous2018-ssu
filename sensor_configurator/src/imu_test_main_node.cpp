#include <ros/ros.h>
#include "IMUConfigurator.h"


int main (int argc, char** argv){
    ros::init(argc, argv, IMU_NODE_NAME);
    ros::NodeHandle nh;

    IMUConfigurator imu;

    ros::Rate loop_rate(IMU_LOOP_RATE);

    while(ros::ok()){
        imu.setParameterData(nh);
        loop_rate.sleep();
    }
}
