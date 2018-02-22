#include <serial/serial.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include "imu/imu_msgs.h"

#define PI 3.141592
#define R_COVAR 99999.0
#define P_COVAR 99999.0
#define Y_COVAR 0.5
#define DEG2RAD PI/180.0

static uint32_t m = 0;

class IMUConfigurator
{
public:
    std::string parse();
    bool serialCommucation(char* path_);
    imu::imu_msgs RPY(std::string parse,ros::NodeHandle nh);
    sensor_msgs::Imu transform(double yaw);
    
private:
    serial::Serial ser;
    sensor_msgs::Imu imu_msg;

};
