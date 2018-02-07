#include <serial/serial.h>
#include "imu/IMU_data.h"

#define IMUFRE 5

class IMUConfigurator
{
public:
    std::string parse();
    int serialCommucation(char* path_);
    void RPY(std::string parse);
    imu::IMU_data msg;

private:
    serial::Serial ser;

};
