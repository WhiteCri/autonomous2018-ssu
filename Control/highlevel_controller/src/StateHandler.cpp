#include "highlevel_controller/StateHandler.h"
#include "highlevel_controller/base_parameter.h"
#include "highlevel_controller/goalSender.h"
#include <ros/ros.h>

static GoalSender goalSender();

extern Parameters* param_ptr;

void init(){
    double x = param_ptr->x_goal.front();
    double y = param_ptr->y_goal.front();
    double yaw = param_ptr->yaw_goal.front();
    //goalSender.setGoal(
}

void toward_goal(){

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