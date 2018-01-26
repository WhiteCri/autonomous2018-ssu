#include "ros/ros.h"
#include "sensor_configurator/DriveIssue.h"

#define FRE 50

static int setSpeed = 0;
static int setSteer = 0;
static int setBrake = 1;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_publisher");
    ros::NodeHandle nh;

    ros::Publisher drive_pub = nh.advertise<sensor_configurator::DriveIssue>("drive_msg", 100);

    ros::Rate loop_rate(FRE);
   
    sensor_configurator::DriveIssue msg;
    

    nh.setParam("drive_speed", 0);
    nh.setParam("drive_steer", 0);
    nh.setParam("drive_brake", 1);
    msg.speed = setSpeed;
    msg.steer = setSteer;
    msg.brake = setBrake;
    
    while(ros::ok()){
        nh.getParam("drive_speed", setSpeed);
        nh.getParam("drive_steer", setSteer);
        nh.getParam("drive_brake", setBrake);

        if(setSpeed <= 20 && setSpeed >= 0) {// km/h : 0 ~ 20
            msg.speed = setSpeed;
            ROS_INFO("speed = %d",msg.speed);
        }
        else
            ROS_INFO("speed = %d - > Out of Range, Reset 0 ~ 20",msg.speed);
        
        if(setSteer <= 28 && setSteer >= -28) { // dgree : -28 ~ 28
            msg.steer = setSteer;
            ROS_INFO("steer = %d",msg.steer);
        }
        else
            ROS_INFO("steer = %d - > Out of Range, Reset -28 ~ 28",msg.steer);
        
        if(setBrake <= 33 && setBrake >= 1) { // brake : 1 ~ 33
            msg.brake = setBrake;
            ROS_INFO("brake = %d",msg.brake);
        }
        else
            ROS_INFO("brake = %d - > Out of Range, Reset 1 ~ 33",msg.brake);

        drive_pub.publish(msg);

        loop_rate.sleep();
    }

    return 0;
}