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
    param_ptr->nh.setParam("hl_controller/crosswalk_onetime_flag",true);    
}

void process_movingobj(){
    param_ptr->nh.setParam("hl_controller/movingobj_onetime_flag",true);
}

void process_parking(){
    param_ptr->nh.setParam("hl_controller/parking_onetime_flag",true);
}

void process_recovery(){
}

void done(){
}