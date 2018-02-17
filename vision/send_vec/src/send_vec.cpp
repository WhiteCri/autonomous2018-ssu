#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int32MultiArray.h"

#define MAX_I 3

int main(int argc, char** argv){
    ros::init(argc,argv,"send_vec");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("/cam0/lane",100);


    ros::Rate loop_rate(5);
    while(ros::ok()){
        std_msgs::Int32MultiArray array;
        array.data.clear();


        array.data.push_back(MAX_I * 2); // size
        for(int i=0; i<MAX_I; i++)
        {
            array.data.push_back(527);
            array.data.push_back(101+50*i);
            array.data.push_back(527);
            array.data.push_back(713+50*i);
        }   // while문 써서 x, y 좌표 번갈아가면서 넣어주기

        pub.publish(array);

        ros::spinOnce();
    }
    
        
    
}