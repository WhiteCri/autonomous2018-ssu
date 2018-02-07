#include <serial/serial.h>
#include "sensor_msgs/Imu.h"

#define IMUFRE 5
    
static uint32_t m = 0;

class IMUConfigurator
{
public:
    std::string parse();
    int serialCommucation(char* path_);
    void RPY(std::string parse,ros::NodeHandle nh);
    sensor_msgs::Imu imu_msg;

private:
    serial::Serial ser;

};
