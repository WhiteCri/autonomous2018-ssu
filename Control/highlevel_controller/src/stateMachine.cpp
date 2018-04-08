#include "highlevel_controller/stateMachine.h"
#include "highlevel_controller/base_parameter.h"
#include <ros/ros.h>

extern Parameters* param_ptr;

inline void display_state(const std::string status){
    ROS_WARN(" ");
    ROS_WARN("STATUES : %s", status.c_str());
}



void init(){
    display_state("init");
}

void toward_goal(){
    display_state("toward_goal");
}

void process_crosswalk(){
    display_state("process_crosswalk");
    param_ptr->nh.setParam("hl_controller/crosswalk_onetime_flag",true);    
}

void process_movingobj(){
    display_state("movingobj");
    param_ptr->nh.setParam("hl_controller/movingobj_onetime_flag",true);
}

void process_parking(){
    display_state("parking");
    param_ptr->nh.setParam("hl_controller/parking_onetime_flag",true);
}

void process_recovery(){
    display_state("recovery");
}

void done(){
    display_state("done");
}