#include <serial/serial.h>
#include <string>
#include "sensor_msgs/Imu.h"
#include "imu/imu_msgs.h"

static uint32_t m = 0;

class IMUConfigurator
{
public:
    std::string parse();
    bool serialCommucation(char* path_);
    imu::imu_msgs RPY(std::string parse,ros::NodeHandle nh);

private:
    serial::Serial ser;
    sensor_msgs::Imu imu_msg;

};
