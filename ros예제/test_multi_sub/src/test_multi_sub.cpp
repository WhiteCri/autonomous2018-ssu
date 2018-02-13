#include "ros/ros.h"                            // ROS Default Header File
#include "test_multi_sub/message_1.h"    // MsgTutorial Message File Header. The header file is automatically created when building the package.
#include "test_multi_sub/message_2.h"
#include "test_multi_sub/message_pub.h"
#include <ros/time.h>

static double a, b, sum;




// Message callback function. This is a function is called when a topic
// message named 'ros_tutorial_msg' is received. As an input message,
// the 'MsgTutorial' message of the 'ros_tutorials_topic' package is received.
void msgCallback_1(const test_multi_sub::message_1::ConstPtr& msg)
{
    ROS_INFO("msgCallback_1 STARTS!");
    a = msg->data;
    ROS_INFO("msgCallback_1 FINISHED!");
}

void msgCallback_2(const test_multi_sub::message_2::ConstPtr& msg)
{
    ROS_INFO("msgCallback_2 STARTS!");
    ros::NodeHandle nh;
    
    b = msg->data;

    sum = a+b;

    test_multi_sub::message_pub msg_pub;
    msg_pub.data = sum;
  
    ros::Publisher pub = nh.advertise<test_multi_sub::message_pub>("Topic_3", 100);
    pub.publish(msg_pub);
    ROS_INFO("msgCallback_2 FINISHED!");
}


int main(int argc, char **argv)                 // Node Main Function
{
  ros::init(argc, argv, "Multi_Sub_Pub");       // Initializes Node Name
  ros::NodeHandle nh;                           // Node handle declaration for communication with ROS system
  ros::Publisher pub = nh.advertise<test_multi_sub::message_pub>("Topic_3", 100);


  ros::Subscriber sub_1 = nh.subscribe("Topic_1", 100, msgCallback_1);
  ros::Subscriber sub_2 = nh.subscribe("Topic_2", 100, msgCallback_2);

  ros::spin();
  
  return 0;
}
