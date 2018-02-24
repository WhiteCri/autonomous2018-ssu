#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


#define FRE 10
int main(int argc, char *argv[]){
    ros::init(argc, argv, "ackermann_pub");
    ros::NodeHandle nh;
    ros::Publisher ackermann_publisher =
        nh.advertise<ackermann_msgs::AckermannDriveStamped>("platform_tx_test",100);
    ros::Rate loop_rate(FRE);
    uint32_t seq = 0;

    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.frame_id = "whereeeeee";
            
    while(ros::ok()){
        msg.header.seq = seq++;
        msg.header.stamp = ros::Time::now();
        msg.drive.steering_angle = 10;
        msg.drive.speed = 0;
        ackermann_publisher.publish(msg);
        loop_rate.sleep();
    }
}