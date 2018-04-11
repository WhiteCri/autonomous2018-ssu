#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>


#define FRE 10
int main(int argc, char *argv[]){
    ros::init(argc, argv, "test_cmd_pub");
    ros::NodeHandle nh;
    ros::Publisher ackermann_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::Rate loop_rate(FRE);

    geometry_msgs::Twist msg;
    
    bool flag = true;
    while(ros::ok()){ 
        int test;
        nh.getParam("test", test);
        msg.linear.x = test;

        //if(flag) msg.drive.speed = -msg.drive.speed;
        //flag = !flag;
        ackermann_publisher.publish(msg);
        loop_rate.sleep();
    }
}