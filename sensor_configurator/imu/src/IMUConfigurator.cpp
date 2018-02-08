#include <ros/ros.h>
#include <string>
#include <sstream>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include "IMUConfigurator.h"

std::string IMUConfigurator::parse()
{
    std::string raw = ser.read(ser.available());

    std::stringstream ss(raw);
    std::string now = "";
    std::getline(ss,now);
    std::getline(ss,now);

    return now;
}

bool IMUConfigurator::serialCommucation(char* path_)
{

    try{
        ser.setPort(path_);///dev/ttyUSB0
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        //0 is error num, but i don't know each number point which err. i just put it to
        //complete function call
    }
    catch(serial::IOException& e){
        ROS_INFO("erro : %s",e.what());
        return false;
    }
    if(!ser.isOpen()) {
        ROS_INFO("Serial is not open!");
        return false;
    }
    return true;
}

imu::imu_msgs IMUConfigurator::RPY(std::string parse, ros::NodeHandle nh)
{
        ROS_INFO("%s",parse.c_str());

        std::stringstream cc(parse);

        std::string trash;

        std::string R="0";
        std::string P="0";
        std::string Y="0";
       
        std::getline(cc,trash,'*');
        std::getline(cc,R,',');
        std::getline(cc,P,',');
        std::getline(cc,Y,',');
        
        //imu_msg.Roll = std::stod(R);
        //imu_msg.Pitch = std::stod(P);
        double yaw = std::stod(Y);
        //qt.setRPY(0,0,yaw);
        //imu_msg.orientation = qt;
            
        //ROS_INFO("%.2f",imu_msg.Roll);
        //ROS_INFO("%.2f",imu_msg.Pitch);
        ROS_INFO("%.2f",yaw);

        ROS_INFO("---------");  
        imu::imu_msgs msg;
        msg.yaw = yaw;
        return msg;
}