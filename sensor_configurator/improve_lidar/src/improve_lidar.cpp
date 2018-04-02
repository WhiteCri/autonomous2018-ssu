#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>

#define Laser_Filter_start 110
#define Laser_Filter_end 430
#define check_dynamic 5
#define check_UTurn 2
#define dynamic_distance_point 0.5
#define UTurn_distance_point 0.5
#define frequency 1

#define test_dynamic
#define test_uturn

static bool dynamic_obstacle, u_turn;

bool checkdynamic(std::vector<float> ranges, int number)
{
    int offset;
    for(offset=1; offset<=check_dynamic; offset++)
    {
        if( (ranges[number] > 0 ) && (ranges[number] < 5) && (fabs(ranges[number]-ranges[number+offset])<dynamic_distance_point) && (fabs(ranges[number]-ranges[number-offset])<dynamic_distance_point) )
            continue;
        else
            return false;
    }
    return true;

}

bool checkUTurn(std::vector<float>ranges, int number)
{
    int offset;
    for(offset=1; offset<=check_UTurn; offset++)
    {
        if((ranges[number]>100) && (fabs(ranges[number]-ranges[number+offset])<0.5) && (fabs(ranges[number]-ranges[number-offset])<0.5) )
            continue;
        else
            return false;
    } 
    return true;
}

void CallScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ros::NodeHandle Node;
    
    for(int i=Laser_Filter_start; i<=Laser_Filter_end;i++)
    {
        if(checkdynamic(scan->ranges,i))
        {
            Node.setParam("dynamic_obstacle",true);
            
            #ifdef test_dynamic
            Node.getParam("dynamic_obstacle",dynamic_obstacle);
            if(dynamic_obstacle)
            {
                 ROS_INFO("range[%d] : %lf",i, scan->ranges[i]);
                 ROS_INFO("Detected Dynamic Obstacle"); 
            }    
            #endif
        }
        else
        {
            Node.setParam("dynamic_obstacle",false);
        }
        
        if(checkUTurn(scan->ranges,i))
        {
            Node.setParam("u_turn",true);

            #ifdef test_uturn
            Node.getParam("u_turn",u_turn);
            if(u_turn)
            {
                ROS_INFO("range[%d] : %lf",i, scan->ranges[i]);
                ROS_INFO("Detected u_turn");
            }   
            #endif
        }
        else
        {
            Node.setParam("u_turn",false);
        }
    }


}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "improve_lidar");
    ros::NodeHandle Node;
    ros::NodeHandle n("~");
    n.param<bool>("dynamic_obstacle",dynamic_obstacle,false);
    n.param<bool>("u_turn",u_turn,false);

    ros::Subscriber scan_sub = Node.subscribe("scan",100,CallScan);
    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();

    return 0;
}