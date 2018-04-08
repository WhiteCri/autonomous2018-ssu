#include <ros/ros.h>
#include <iostream>
#include "highlevel_controller/base_parameter.h"

Parameters* param_ptr;

int main(int argc, char *argv[]){

    ros::init(argc, argv, "highlevel_controller");

    //do not relocate this code. it must locate under the declaration of nodeHandle
    param_ptr = Parameters::getInstance();
    ros::Rate loop_rate(param_ptr->frequency);
    while(ros::ok()){
        param_ptr->load_param();
        loop_rate.sleep();    
    }
}