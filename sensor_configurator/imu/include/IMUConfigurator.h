#include <serial/serial.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

#define PI 3.141592
#define R_COVAR 99999.0
#define P_COVAR 99999.0
#define Y_COVAR 0.5
#define DEG2RAD PI/180.0

static uint32_t m = 0;

class IMUConfigurator
{
public:
    IMUConfigurator(ros::NodeHandle& nh);
    std::string parse();
    bool serialCommucation(char* path_);
    void RPY(std::string parse);
    sensor_msgs::Imu transform();
    
private:
    serial::Serial ser;
    sensor_msgs::Imu imu_msg;
    ros::NodeHandle *nhPtr_;
    double yaw;
    double shift;
    double cov;
    bool debugingFlag;
    boost::array<double, 9> covariance;
};
