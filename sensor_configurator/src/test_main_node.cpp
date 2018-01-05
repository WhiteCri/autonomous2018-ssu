#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "SerialCommunicator.h"


void write_callback(const std_msgs::String::ConstPtr& msg){
    //ROS_INFO_STREAM("Writing to serial port" << msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "test_main_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    SerialCommunicator *ser = new SerialCommunicator("/dev/ttyUSB0");
    if(!ser->isValid()){
      ROS_INFO("serial not valid : %s",ser->getErrorMsg().c_str());
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){
        ros::spinOnce();
        if(ser->isValid()){
            ROS_INFO("Reading from serial port");
            std_msgs::String result;
            result.data = ser->read();
            ROS_INFO_STREAM("Read: \n" << result.data);
            read_pub.publish(result);
        }
        else {
          ROS_ERROR("Serial error!");
          break;
        }
        loop_rate.sleep();
    }
}
