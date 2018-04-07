#include <ros/ros.h>
#include <iostream>
#include "highlevel_controller/base_parameter.h"

Parameters* param_ptr;

int main(int argc, char *argv[]){

    ros::init(argc, argv, "highlevel_controller");
    ros::NodeHandle nh;

    //do not relocate this code. it must locate under the declaration of nodeHandle
    param_ptr = Parameters::getInstance(nh);
    ros::Rate loop_rate(param_ptr->frequency);
    while(ros::ok()){
        param_ptr->load_param(nh);
        loop_rate.sleep();    
    }
}