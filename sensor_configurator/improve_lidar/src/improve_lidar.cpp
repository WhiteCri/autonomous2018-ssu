#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <obstacle_detector/Obstacles.h>
#include <cmath>
#include <thread>
#include <vector>


/* U_turn check */
#define Laser_Filter_start 200
#define Laser_Filter_end 300
#define frequency 10

#define check_dynamic 1
#define check_UTurn 3
#define dynamic_distance_point 0.2
#define UTurn_distance_point 0.1

/* dynamic check 
#define size_N 4

#define min_y_distance 0.5
#define max_y_distance 2

#define min_x_obstacle 5
#define max_x_distance 0.015

#define min_delta_y 0.0035
#define max_delta_y 0.08
*/
#define test_uturn
#define test_dynamic

static double inf = std::numeric_limits<double>::infinity();
static bool dynamic_obstacle, u_turn;
static int before_detect = 0;

struct priv{
    std::vector<double> priv_data_first_y;
    std::vector<double> delta_y;
};
struct dynamic{
    int size_N;
    double min_y_distance;
    double max_y_distance;
    double min_x_obstacle;
    double max_x_distance;
    double min_delta_y;
    double max_delta_y;
};

static dynamic dynamic_param;
static priv priv;

using namespace obstacle_detector;

void obstaclecheck(const obstacle_detector::Obstacles::ConstPtr &object)
{

    if(object->segments.empty())
        return;
    
    ros::NodeHandle Node;
    std::vector<std::vector<double>> dataarry;
    double delta_y;
    bool dynamic_obstacle = false;

    for(int i=0; i<object->segments.size(); i++)
    {
        std::vector<double> element;
        element.resize(4);
        dataarry.push_back(element);
    }
    /* 내 vector로 모든 data추출 */
    for(int i=0; i<object->segments.size(); i++)
    {
        dataarry[i][0] = object->segments.at(i).first_point.x;
        dataarry[i][1] = object->segments.at(i).first_point.y;
        dataarry[i][2] = object->segments.at(i).last_point.x;
        dataarry[i][3] = object->segments.at(i).last_point.y;
    }
    /* find arry num */
    for(int i=0; i<object->segments.size(); i++)
    {
        if( (fabs(dataarry[i][1] - dataarry[i][3]) > dynamic_param.min_y_distance)  && (fabs(dataarry[i][1] - dataarry[i][3]) < dynamic_param.max_y_distance) &&
            (fabs(dataarry[i][0] - dataarry[i][2]) < dynamic_param.max_x_distance) && (fabs(dataarry[i][0]) > dynamic_param.min_x_obstacle) 
            && (fabs(dataarry[i][2]) > dynamic_param.min_x_obstacle) )
        {
            priv.priv_data_first_y.push_back(dataarry[i][1]);
        }
    }

    /* 이전 data와 비교해서 delta_y 추출 */
    if( priv.priv_data_first_y.size() == dynamic_param.size_N )
    {
        for(std::vector<double>::size_type i = 0; i < priv.priv_data_first_y.size()-1; i++)
        {
            delta_y = fabs(priv.priv_data_first_y[i] - priv.priv_data_first_y[i+1]);
            priv.delta_y.push_back(delta_y);
        }   
        priv.priv_data_first_y.clear();
    }
    /* delta y에 대한 샘플링 시작 */
    if(priv.delta_y.size() == (dynamic_param.size_N - 1)) 
    {
        for(std::vector<double>::size_type i = 0; i < priv.delta_y.size(); i++)
        {
            ROS_INFO("delta[%ld] y : %lf",i,priv.delta_y[i]);
            if(priv.delta_y[i] > dynamic_param.min_delta_y && priv.delta_y[i] < dynamic_param.max_delta_y)
                dynamic_obstacle = true;
            else
            {
                dynamic_obstacle = false;
                break;
            }
        }
        priv.delta_y.clear();
    }

    Node.setParam("hl_controller/movingobj",dynamic_obstacle);

    #ifdef test_dynamic
        if(dynamic_obstacle)
          {  ROS_INFO("detected_dynamic");
          }
    #endif


}

bool checkdynamic(std::vector<float> ranges, int number)
{
    ros::NodeHandle Node;

   
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
    int min_number = number - 1;
    /* detect size 크기 조절 */
    if( (ranges[number+max_range] == inf || ranges[number+max_range] == -inf) &&
        (ranges[min_number] == inf || ranges[min_number] == -inf) )
        {}
    else
        return false;
    /* detection */
    for(offset=1; offset<=check_UTurn; offset++)
    {
        if( (ranges[number+offset] != inf) && 
            (fabs(ranges[number]-ranges[number+offset]) < UTurn_distance_point) )
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

void uturncall(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ros::NodeHandle Node;
    
    for(int i=Laser_Filter_start; i<=Laser_Filter_end;i++)
    {/*
        if(checkdynamic(scan->ranges,i))
        {
            dynamic_obstacle = true;

            #ifdef test_dynamic
            if(dynamic_obstacle)
            {
                 ROS_INFO("range[%d] : %lf",i, scan->ranges[i]);
                 ROS_INFO("Detected Dynamic Obstacle"); 
            }    
            #endif
        }
        else
        {
            dynamic_obstacle = false;
        }
        */
        if(checkUTurn(scan->ranges,i))
        {      
            u_turn = true;   

            #ifdef test_uturn
            if(u_turn)
            {
                ROS_INFO("range[%d] : %lf",i, scan->ranges[i]);
                ROS_INFO("Detected u_turn");
            }   
            #endif
        }
        else
        {
            u_turn = false;
        }
    }
    Node.setParam("hl_controller/Uturn",u_turn);


}

void subscribeuturn()
{
    ros::NodeHandle Node;
    ros::Subscriber scan_sub = Node.subscribe("uturn_scan",100,uturncall);
    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "improve_lidar");
    ros::NodeHandle Node;
    ros::NodeHandle n("~");

    n.param<bool>("hl_controller/movingobj",dynamic_obstacle,false);
    n.param<bool>("hl_controller/Uturn",u_turn,false);
    n.getParam("size_N",dynamic_param.size_N);
    n.getParam("min_y_distance",dynamic_param.min_y_distance);
    n.getParam("max_y_distance",dynamic_param.max_y_distance);
    n.getParam("min_x_obstacle",dynamic_param.min_x_obstacle);
    n.getParam("max_x_distance",dynamic_param.max_x_distance);
    n.getParam("min_delta_y",dynamic_param.min_delta_y);
    n.getParam("max_delta_y",dynamic_param.max_delta_y);
    
    ros::Subscriber sub = Node.subscribe("raw_obstacles",100,obstaclecheck);

//    std::thread dynamicnode(subscribeuturn);
 //   dynamicnode.detach();

    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}