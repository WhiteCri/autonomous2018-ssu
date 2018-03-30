#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


#define FRE 5
int main(int argc, char *argv[]){
    ros::init(argc, argv, "ackermann_pub");
    ros::NodeHandle nh;
    ros::Publisher ackermann_publisher =
        nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd",100);
    ros::Rate loop_rate(FRE);
    uint32_t seq = 0;

    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.frame_id = "whereeeeee";
    
    bool flag = true;
    while(ros::ok()){
        msg.header.seq = seq++;
        msg.header.stamp = ros::Time::now();
        msg.drive.steering_angle = (20)*(3.141592/180.0);
        msg.drive.speed = 0.5;
        nh.getParam("txSpeed",msg.drive.speed);
        //if(flag) msg.drive.speed = -msg.drive.speed;
        //flag = !flag;
        ackermann_publisher.publish(msg);
        loop_rate.sleep();
    }
}