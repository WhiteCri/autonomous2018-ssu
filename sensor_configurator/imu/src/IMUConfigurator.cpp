
#include "IMUConfigurator.h"

IMUConfigurator::IMUConfigurator(ros::NodeHandle& nh) 
    : nhPtr_(&nh), yaw(0), cov(0), shift(0), debugingFlag(false)
{
    //getParam

    nhPtr_->param<double>("imu/covariance",cov, 0.5);
    nhPtr_->param<double>("imu/shift", shift, 0);
    nhPtr_->param<bool>("imu/debugingFlag", debugingFlag, false);


    //show state
    ROS_INFO("shift : %.2lf",shift);
    std::string temp = "DebugingFlag : ";
    if(debugingFlag == true) temp += "true";
    else temp += "false";
    ROS_INFO("%s",temp.c_str());
    ROS_INFO("covariance: %lf", cov);
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

     nhPtr_->getParam("imu/debugingFlag", debugingFlag);
    
     std::getline(cc,trash,'*');
     std::getline(cc,R,',');
     std::getline(cc,P,',');
     std::getline(cc,Y,',');
     
     //imu_msg.Roll = std::stod(R);
     //imu_msg.Pitch = std::stod(P);
     yaw = std::stod(Y);
     yaw = -yaw;
     yaw += shift;
     if(yaw >= 180.0) yaw -= 360;
     if(yaw <= -180.0) yaw += 360;
     if(debugingFlag == true)
        ROS_INFO("get : %.2lf(degree)",yaw);
}

sensor_msgs::Imu IMUConfigurator::transform()
{ 
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(yaw  * DEG2RAD);

    sensor_msgs::Imu imu_msg;
    
    nhPtr_->getParam("imu/covariance", cov);
    
    nhPtr_->getParam("imu/covariance",cov);
    nhPtr_->getParam("imu/shift", shift);


    boost::array<double, 9> covariance = {{
    cov, 0, 0,
    0, cov, 0,
    0, 0, cov,
    }};

    imu_msg.header.frame_id = "base_link";
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.orientation = quaternion;
    imu_msg.orientation_covariance = covariance;


    return imu_msg;
}