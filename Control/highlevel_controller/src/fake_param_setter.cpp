#include <ros/ros.h>


int main(int argc, char *argv[]){
    if (argc <= 1) return -1;
    ros::init(argc, argv, "fake_param_setter");
    ros::NodeHandle nh;

    ros::Rate(0.2).sleep();
    nh.setParam(argv[1], true);
}