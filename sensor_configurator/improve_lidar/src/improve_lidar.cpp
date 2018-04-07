#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <assert.h>


#define Laser_Filter_start 200
#define Laser_Filter_end 300
#define check_dynamic 5
#define check_UTurn 2
#define dynamic_distance_point 0.2
#define UTurn_distance_point 0.1
#define frequency 0.8

#define test_dynamic

static double inf = std::numeric_limits<double>::infinity();
static bool dynamic_obstacle, u_turn;
static int before_detect = 0;

bool checkdynamic(std::vector<float> ranges, int number)
{
    int max_range = check_dynamic + 1;
    /* detect size 크기 조절 */
    if( (ranges[number+max_range] == inf || ranges[number+max_range] == -inf) &&
        (ranges[number-max_range] == inf || ranges[number-max_range] == -inf) )
        {}
    else
        return false;

    int offset;
    for(offset=1; offset<=check_dynamic; offset++)
    {
        if((fabs(ranges[number]-ranges[number+offset])<dynamic_distance_point) &&
          (fabs(ranges[number]-ranges[number-offset])<dynamic_distance_point) )
            continue;
        else
            return false;
    }
    return true;

}

bool checkUTurn(std::vector<float>ranges, int number)
{
    int offset;
    int max_range = check_UTurn + 1;
    /* detect size 크기 조절 */
    if( (ranges[number+max_range] == inf || ranges[number+max_range] == -inf) &&
        (ranges[number-max_range] == inf || ranges[number-max_range] == -inf) )
        {}
    else
        return false;
    /* detection */
    for(offset=1; offset<=check_UTurn; offset++)
    {
        if( (ranges[number+offset] != inf) && (ranges[number-offset] != inf) &&
            (fabs(ranges[number]-ranges[number+offset]) < UTurn_distance_point) &&
           (fabs(ranges[number]-ranges[number-offset]) < UTurn_distance_point))
                continue;
            
        else
            return false;
    }
    /* 이전 상태와 range값 비교해서 최대한 붙어 있게 설정 */
    if(before_detect == 0)
        before_detect = number;
    if((fabs(ranges[number]-ranges[before_detect]) > 0.2))
        return false;
    before_detect = number;
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

    ros::Subscriber scan_sub = Node.subscribe("uturn_scan",100,CallScan);
    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();

    return 0;
}