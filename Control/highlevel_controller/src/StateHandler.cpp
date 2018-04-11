#include "highlevel_controller/StateHandler.h"
#include "highlevel_controller/base_parameter.h"
#include "highlevel_controller/goalSender.h"
#include <ros/ros.h>

extern Parameters* param_ptr;
extern GoalSender* goalSender_ptr;

inline void showGoal(double x, double y, double yaw, 
    const std::string& str){
    ROS_INFO("set new goal : %lf, %lf, %lf(degree), %s",
        x, y, yaw, str.c_str());
}

void init(){
    double x = param_ptr->x_goal.back();
    double y = param_ptr->y_goal.back();
    double yaw = param_ptr->yaw_goal.back();
    std::string goal_type = param_ptr->goal_type.back();

    goalSender_ptr->setGoal(x, y, yaw);
    goalSender_ptr->sendGoal();
    goalSender_ptr->sendGoal();
    showGoal(x, y, yaw, goal_type);
}

void toward_goal(){
    typedef GoalSender::GoalStates GoalStates;
    param_ptr->nh.setParam("hl_controller/tx_stop", false);

    auto state = goalSender_ptr->getState();
    if (state == GoalStates::STATE_SUCCEEDED){
        ROS_INFO("SUCCEEDED...");
       param_ptr->x_goal.pop_back();
       param_ptr->y_goal.pop_back();
       param_ptr->yaw_goal.pop_back();
       param_ptr->goal_type.pop_back();

       if(param_ptr->goal_type.size() == 0){
           param_ptr->nh.setParam("hl_controller/reached_goal", true);
           return;
       }
       
       double x = param_ptr->x_goal.back();
       double y = param_ptr->y_goal.back();
       double yaw = param_ptr->yaw_goal.back();
       std::string goal_type = param_ptr->goal_type.back();
       
       goalSender_ptr->setGoal(x, y, yaw);
       goalSender_ptr->sendGoal();
       showGoal(x, y, yaw, goal_type);
    }
    else if (state == GoalStates::STATE_LOST){
        ROS_INFO("LOST...");
        double x = param_ptr->x_goal.back();
        double y = param_ptr->y_goal.back();
        double yaw = param_ptr->yaw_goal.back();
        std::string goal_type = param_ptr->goal_type.back();
       
        goalSender_ptr->setGoal(x, y, yaw);
        showGoal(x, y, yaw, goal_type);
        goalSender_ptr->sendGoal();
    }
    else if (state == GoalStates::STATE_ACTIVE){}
    else if (state == GoalStates::STATE_PENDING) {
        ROS_INFO("PENDING...");
    }
    else {
        ROS_INFO("why control reaches here...");
    } 
}

void process_crosswalk(){
    ROS_INFO("crosswalk start");

    //maintaining car's status
    double crosswalk_driving_duration = param_ptr->crosswalk_driving_duration;
    if (crosswalk_driving_duration > 0){
        ROS_INFO("maintaining it's status for %lf seconds...", param_ptr->crosswalk_driving_duration);
        ros::Rate(param_ptr->crosswalk_driving_duration).sleep();
    }

    //take car to stop
    ROS_INFO("stop...");
    param_ptr->nh.setParam("hl_controller/tx_stop", true);

    //take car to wait
    ros::Rate(1 / param_ptr->crosswalk_stop_duration).sleep();

    //check that process_crosswalk had been done.
    param_ptr->nh.setParam("hl_controller/crosswalk_onetime_flag",true);

    ROS_INFO("crosswalk done");    
}

void process_movingobj(){
    ROS_INFO("movingobj start");

    //maintaining car's status
    double movingobj_driving_duration = param_ptr->movingobj_driving_duration;
    if (movingobj_driving_duration > 0){
        ROS_INFO("maintaining it's status for %lf seconds...", param_ptr->movingobj_driving_duration);
        ros::Rate(param_ptr->movingobj_driving_duration).sleep();
    }
    //take car to stop
    ROS_INFO("stop...");
    param_ptr->nh.setParam("hl_controller/tx_stop", true);

    //take car to wait
    ros::Rate(1 / param_ptr->movingobj_stop_duration).sleep();
    
    //check that process_movingobj had been done.
    param_ptr->nh.setParam("hl_controller/movingobj_onetime_flag",true);

    ROS_INFO("movingobj done");
}

void process_parking(){
    ROS_INFO("process parking start");

    ROS_INFO("finding parking point...");
    /* 
        I haven't find the solution to select the goal. So temporarily, just select the near goal.
        this code should be modified until the contest
    */
    double parking_point_x   = param_ptr->parking_near_arrive_point_x;
    double parking_point_y   = param_ptr->parking_near_arrive_point_y;
    double parking_point_yaw = 0;
    double backing_point_x   = param_ptr->parking_near_back_point_x;
    double backing_point_y   = param_ptr->parking_near_back_point_y;
    double backing_point_yaw = 0;
    ROS_INFO("set near point as a goal...");
    
    //set goal to parking point
    ROS_INFO("to the parking point...");
    goalSender_ptr->setGoal(
        parking_point_x,
        parking_point_y,
        parking_point_yaw
    );
    goalSender_ptr->sendGoal();

    //wait until arrive.
    while(true){
        auto state = goalSender_ptr->getState();
        if (state == GoalSender::GoalStates::STATE_SUCCEEDED) break;
        ros::Rate(param_ptr->frequency).sleep();
        ROS_INFO("parking point has not been arrived...");
    }
    ROS_INFO("Arrived parking point...");

    //take car to stop
    ROS_INFO("stop...");
    param_ptr->nh.setParam("hl_controller/tx_stop", true);

    //take car to wait
    ros::Rate(1 / param_ptr->parking_stop_duration).sleep();

    //set goal to backing point
    ROS_INFO("set tx_stop false");
    param_ptr->nh.setParam("hl_controller/tx_stop", false);

    ROS_INFO("to the backing point...");
    goalSender_ptr->setGoal(
        backing_point_x,
        backing_point_y,
        backing_point_yaw
    );
    goalSender_ptr->sendGoal();

    //wait until arrive.
    while(true){
        auto state = goalSender_ptr->getState();
        if (state == GoalSender::GoalStates::STATE_SUCCEEDED) break;
        ros::Rate(param_ptr->frequency).sleep();
        ROS_INFO("backing point has not been arrived...");
    }
    ROS_INFO("Arrived backing point...");

    param_ptr->nh.setParam("hl_controller/parking_onetime_flag",true);
}

void process_recovery(){
    param_ptr->nh.setParam("hl_controller/recovery",false);
}

void done(){
    ROS_INFO("ALL GOAL had been processed");
}