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

void IMUConfigurator::RPY(std::string parse,ros::NodeHandle nh)
{
        ros::Publisher imu_data_pub = nh.advertise<sensor_msgs::Imu>("raw/imu", 100);

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
        
        double Roll = std::stod(R);
        double Pitch = std::stod(P);
        double Yaw = std::stod(Y);

    //Roll Pitch Yaw 값을 이용해 연산하여 x,y,z,w 값에 넣어줘야함
    //covariance 에 0 , -1 값을 적절히 넣어줘야함 

    imu_msg.header.seq = m%0xffffffff;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu";

    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 0;
    imu_msg.orientation_covariance[0] = 0;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0;

    imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = 0;
    imu_msg.angular_velocity_covariance[0] = 0;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0;

    imu_msg.linear_acceleration.x = 0;
    imu_msg.linear_acceleration.y = 0;
    imu_msg.linear_acceleration.z = 0;
    imu_msg.linear_acceleration_covariance[0] = 0;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0;

    ++m;
    

////

                
        ROS_INFO("%.2f",Roll);
        ROS_INFO("%.2f",Pitch);
        ROS_INFO("%.2f",Yaw);

        ROS_INFO("---------");


        imu_data_pub.publish(imu_msg);

}