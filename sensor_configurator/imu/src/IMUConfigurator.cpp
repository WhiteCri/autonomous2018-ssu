#include <ros/ros.h>
#include <string>
#include <sstream>
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

int IMUConfigurator::serialCommucation(char* path_)
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
        return -1;
    }
    if(!ser.isOpen()) {
        ROS_INFO("Serial is not open!");
        return -1;
    }
}

void IMUConfigurator::RPY(std::string parse)
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
        
        msg.Roll = std::stod(R);
        msg.Pitch = std::stod(P);
        msg.Yaw = std::stod(Y);
                
        ROS_INFO("%.2f",msg.Roll);
        ROS_INFO("%.2f",msg.Pitch);
        ROS_INFO("%.2f",msg.Yaw);

        ROS_INFO("---------");

}