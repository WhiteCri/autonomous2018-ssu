
#include "IMUConfigurator.h"

IMUConfigurator::IMUConfigurator(ros::NodeHandle& nh) 
    : nhPtr_(&nh), yaw(0), angle_alignment(0), debugingFlag(false)
{
    //getParam
    nhPtr_->getParam("angle_alignment", angle_alignment);
    nhPtr_->getParam("debugingFlag", debugingFlag);
    std::vector<double> covVector(9);
    nhPtr_->getParam("imu_covariance",covVector);
    for(int i = 0 ; i < 9; ++i)
        covariance[i] = covVector[i];

    //show state
    ROS_INFO("angle_alignment : %.2lf",angle_alignment);
    std::string temp = "DebugingFlag : ";
    if(debugingFlag == true) temp += "true";
    else temp += "false";
    ROS_INFO("%s",temp.c_str());
    ROS_INFO("[%5.2lf %5.2lf %5.2lf",covariance[0], covariance[1], covariance[2]);
    ROS_INFO("[%5.2lf %5.2lf %5.2lf",covariance[3], covariance[4], covariance[5]);
    ROS_INFO("[%5.2lf %5.2lf %5.2lf",covariance[6], covariance[7], covariance[8]);
}

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

void IMUConfigurator::RPY(std::string parse)
{
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
     yaw = std::stod(Y);
     yaw = -yaw;
     yaw += angle_alignment;
     if(debugingFlag == true){
        ROS_INFO("get : %.2lf(degree)",yaw);
        ROS_INFO("[%5.2lf %5.2lf %5.2lf",covariance[0], covariance[1], covariance[2]);
        ROS_INFO("[%5.2lf %5.2lf %5.2lf",covariance[3], covariance[4], covariance[5]);
        ROS_INFO("[%5.2lf %5.2lf %5.2lf",covariance[6], covariance[7], covariance[8]);
    }
}
sensor_msgs::Imu IMUConfigurator::transform()
{ 
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(yaw  * DEG2RAD);
    nhPtr_->getParam("imu_yaw_covariance",covariance[8]);
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "base_link";
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.orientation = quaternion;
    imu_msg.orientation_covariance = covariance;
    return imu_msg;
}