#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

static bool dynamic_obstacle, u_turn;

void CallScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO("%d",scan->ranges[1]);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "improve_lidar");
    ros::NodeHandle Node;
    ros::NodeHandle n("~");
    n.param<bool>("dynamic_obstacle",dynamic_obstacle,false);
    n.param<bool>("u_turn",u_turn,false);

    ros::Subscriber scan_sub = Node.subscribe("scan",100,CallScan);

    ros::spin();

    return 0;
}