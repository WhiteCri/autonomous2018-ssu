#include <ros/ros.h>


int main(int argc, char *argv[]){
    ros::init(argc, argv, "fake_param_setter");
    ros::NodeHandle nh;

    ros::Rate(0.1).sleep();
    nh.setParam("hl_controller/crosswalk", true);
}