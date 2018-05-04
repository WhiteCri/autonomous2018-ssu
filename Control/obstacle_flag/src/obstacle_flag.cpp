/*
 * obstacle_flag.cpp
 *
 *  Created on: 2018-05-03
 *  Author: HwiJin Hong
 *
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <obstacle_detector/Obstacles.h>
#include "obstacle_flag/data.h"
#include <cmath>
#include <thread>
#include <vector>

/* U_turn check */
#define Laser_Filter_start 195
#define Laser_Filter_end 330
#define frequency 30

#define test_uturn
#define test_dynamic
#define test_park

static double inf = std::numeric_limits<double>::infinity();
static bool dynamic_obstacle, u_turn;
static bool is_park;

static active active;
static dynamic dynamic_param;
static priv priv;
static u_turn_param u_turn_param;
static park park;

using namespace obstacle_detector;

void obstaclecheck(const obstacle_detector::Obstacles::ConstPtr &object)
{
    ros::NodeHandle Node;
    Node.getParam("obstacle_flag/active/dynamic",active.active_dynamic);
    if(!active.active_dynamic)
        return;

    if(object->segments.empty())
        return;
    
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
            (fabs(dataarry[i][0] - dataarry[i][2]) < dynamic_param.max_x_width) && (fabs(dataarry[i][0]) > dynamic_param.min_x_obstacle) 
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

bool checkUTurn(std::vector<float>ranges, int number)
{
    int offset;
    int min_number = number - u_turn_param.check_UTurn;
   /*  detect size 크기 조절 */
    if( /*ranges[number+max_range] == inf || ranges[number-max_range] == inf &&*/
        (ranges[min_number] == inf) )
        {}  
    else
        return false; 
    /* detection */
    for(offset=0; offset<=u_turn_param.check_UTurn; offset++)
    {
        if( (ranges[number+offset] != inf) && 
            (fabs(ranges[number]-ranges[number+offset]) < u_turn_param.UTurn_distance_point) )
                continue;
            
        else
            return false;
    }
    /* 이전 상태와 range값 비교해서 최대한 붙어 있게 설정 */
    if(u_turn_param.before_detect == 0)
        u_turn_param.before_detect = number;
    if((fabs(ranges[number]-ranges[u_turn_param.before_detect]) > 0.1))
        return false;
    u_turn_param.before_detect = number;
    return true;
}

void uturncall(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ros::NodeHandle Node;
    Node.getParam("obstacle_flag/active/u_turn",active.active_uturn);
    if(!active.active_uturn)
        return;
    int count=0;

    for(int i=Laser_Filter_start; i<=Laser_Filter_end;i++)
    {
        if(checkUTurn(scan->ranges,i))
        {      
            u_turn = true;   
            count++;
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
    if(count>=u_turn_param.number_of_uturn)
        u_turn = true;
    else
        u_turn = false;

    #ifdef test_uturn
        ROS_INFO("count : %d",count);
    #endif

    Node.setParam("hl_controller/uturn",u_turn);
}

void parkcall(const obstacle_detector::Obstacles::ConstPtr &object)
{
    ros::NodeHandle Node;
    Node.getParam("obstacle_flag/active/park",active.active_park);

    if(!active.active_park)
        return;

    if(object->segments.empty() && object->segments.size() == 1)
        return;
    
    std::vector<int> index;
    is_park = false;
    double x,y;
    /* park area checking */
    for(std::vector<double>::size_type i=0; i<object->segments.size(); i++)
    {
        if( object->segments.at(i).first_point.x > park.min_x && object->segments.at(i).first_point.x < park.max_x &&
            object->segments.at(i).last_point.x > park.min_x && object->segments.at(i).last_point.x < park.max_x && 
            object->segments.at(i).first_point.y > park.min_y && object->segments.at(i).first_point.y < park.max_y &&
            object->segments.at(i).last_point.y > park.min_y && object->segments.at(i).last_point.y < park.max_y )
              index.push_back(i);
    }
    /* no data detect then return */
    if(index.size() == 0)
        return;

    /* park track checking */
    for(std::vector<int>::size_type i=0; i<index.size(); i++)
    {
        if( fabs(object->segments.at(index.at(i)).last_point.x - object->segments.at(index.at(i)+1).first_point.x) <= park.obj_2_distance &&
            fabs(object->segments.at(index.at(i)).last_point.y - object->segments.at(index.at(i)+1).first_point.y) <= park.obj_2_distance    )
           { 
               is_park = true;
               x = object->segments.at(index.at(i)).last_point.x;
               y = object->segments.at(index.at(i)).last_point.y;
           }

    }
    #ifdef test_park
        if(is_park)
         {
                ROS_INFO("detected_park");
                ROS_INFO("first_x : %lf , first_y : %lf",x,y);
         }

    #endif
    Node.setParam("h1_controller/parking",is_park);

}

void subscribepark()
{
    ros::NodeHandle Node;
    ros::NodeHandle n("~");

    n.getParam("park/min_x",park.min_x);
    n.getParam("park/min_y",park.min_y);
    n.getParam("park/max_x",park.max_x);
    n.getParam("park/max_y",park.max_y);
    n.getParam("park/obj_2_distance",park.obj_2_distance);

    ros::Subscriber scan_sub = Node.subscribe("park_obstacle",100,parkcall);
    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void subscribeuturn()
{
    ros::NodeHandle Node;
    ros::NodeHandle n("~");

    n.getParam("u_turn/UTurn_distance_point",u_turn_param.UTurn_distance_point);
    n.getParam("u_turn/check_UTurn",u_turn_param.check_UTurn);
    n.getParam("u_turn/number_of_uturn",u_turn_param.number_of_uturn);
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
    ros::init(argc, argv, "obstacle_flag");
    ros::NodeHandle Node;
    ros::NodeHandle n("~");

    n.param<bool>("hl_controller/movingobj",dynamic_obstacle,false);
    n.param<bool>("hl_controller/uturn",u_turn,false);
    n.param<bool>("h1_controoller/parking",is_park,false);


    n.getParam("dynamic/size_N",dynamic_param.size_N);
    n.getParam("dynamic/min_y_distance",dynamic_param.min_y_distance);
    n.getParam("dynamic/max_y_distance",dynamic_param.max_y_distance);
    n.getParam("dynamic/min_x_obstacle",dynamic_param.min_x_obstacle);
    n.getParam("dynamic/max_x_width",dynamic_param.max_x_width);
    n.getParam("dynamic/min_delta_y",dynamic_param.min_delta_y);
    n.getParam("dynamic/max_delta_y",dynamic_param.max_delta_y);

    
    ros::Subscriber sub = Node.subscribe("dynamic_obstacle",100,obstaclecheck);

    std::thread dynamicnode(subscribeuturn);
    dynamicnode.detach();

    std::thread parking(subscribepark);
    parking.detach();

    ros::Rate loop_rate(frequency);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}