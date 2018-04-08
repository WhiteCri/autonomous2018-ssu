#include "highlevel_controller/stateMachine.h"
#include <ros/ros.h>

extern Parameters* param_ptr;

inline void display_state(const std::string status){
    ROS_WARN("");
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
}

void process_movingobj(){
    display_state("movingobj");
}

void process_parking(){
    display_state("parking");
}

void process_recovery(){
    display_state("recovery");
}

void done(){
    display_state("done");
}