#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "lane_detection/MsgLane.h"

void createExample();
std::vector<cv::Vec3f> distData;

class CalDistance{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
public:
    CalDistance()
    {   
        //sub_=nh_.subscribe("/cam0/lane",1,&laneCb,this);
        //pub_ = nh_.advertise("/cam0/lane/dist",1);

    }
    void laneCb(const lane_detection::MsgLane::ConstPtr& msg);
    void calYdist();
    void calXgap50();
    void calXdist();

private:
    double xGap;
    double xDist;
    double yGap;
    double yDist;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "cal_distance");
    CalDistance calDist;
    ros::spin();

    return 0;
}

// void laneCb(const lane_detection::MsgLane::ConstPtr& msg){

// }


void CalDistance::calYdist(){
    yDist = 0.4819*std::exp(0.0066*yGap);
}

void CalDistance::calXgap50(){ 
    xGap50 = 0.0146*std::pow(yDist, 6) - 0.3673*std::pow(yDist, 5) + 3.8374*std::pow(yDist, 4) 
        - 21.814*std::pow(yDist, 3) + 74.873*std::pow(yDist, 2) - 167.26*yDist + 271.67;
}

void CalDistance::calXdist(){
    xDist = 0.5*xGap / xGap50;
}

void createExample(){
  distData.push_back(cv::Vec3f(1,0,0));
  distData.push_back(cv::Vec3f(1,1,0));
  distData.push_back(cv::Vec3f(1,2,0));
  distData.push_back(cv::Vec3f(1,3,0));
  distData.push_back(cv::Vec3f(1,4,0));
  distData.push_back(cv::Vec3f(3,0,0));
  distData.push_back(cv::Vec3f(3,1,0));
  distData.push_back(cv::Vec3f(3,2,0));
  distData.push_back(cv::Vec3f(3,3,0));
  distData.push_back(cv::Vec3f(3,4,0));
}

