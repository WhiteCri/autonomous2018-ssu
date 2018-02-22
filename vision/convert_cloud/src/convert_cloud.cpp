

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <vector>
#include <ctime>
//#include "my_pkg/my_msg.h"

static const bool DEBUG = true; // 디버깅 스위치

class ConvertCloud{
  ros::NodeHandle nh_;
  ros::Subscriber dist_sub_;
  ros::Publisher point_pub_;

public:
  ConvertCloud();
  //void distCb(const my_pkg::my_msg::ConstPtr& msg);  
  void deSend();
  void deConvert();
  void createExample();
  void convert();

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc;
  sensor_msgs::PointCloud2 pc_out;
  std::vector<cv::Vec3f> distData;

};

int main(int argc, char** argv){
  ros::init(argc,argv,"convert_cloud");
  cv::startWindowThread();
  ConvertCloud cvtCloud;
  // if(!DEBUG){
  //   ros::spin();
  // }
  // else{
    cvtCloud.deSend();
  // }

  return 0;
}





//## 구현부자

// 생성자
ConvertCloud::ConvertCloud(){
  //if(!DEBUG) dist_sub_ = nh_.subscribe("/cam0/lane/dist",1, &ConvertCloud::distCb, this);
  point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cam0/point_cloud",10);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pc = ref_pc;
}


// 메세지 콜백
// void ConvertCloud::distCb(const my_pkg::my_msg::ConstPtr& msg){
//   distData.clear();    

//   // 1안
//   distData = msg->distVec;

//   // 2안
//   // std::vector<cv::Vec3f>::iterator msg_iter = msg->distVec.begin();
//   // while(msg_iter != msg->distVec.end()){
//   //   distData.p백ush_back(*msg_iter);
//   // }
    
//   if(!distData.empty()){
//     convert();
//     point_pub_.publish(pc_out);
//   }    
// }


// 클라우드 변환
void ConvertCloud::convert(){
  pc->clear();

  // Create header
  std::string frame_id("convert_cloud");
  pc->header.frame_id = frame_id;
  pc->header.seq = ros::Time::now().toNSec()/1e3;
  //pc->header.stamp = ros::Time();

  pcl::PointCloud<pcl::PointXYZ>::iterator pc_iter = pc->begin();
  std::vector<cv::Vec3f>::iterator data_iter = distData.begin();
    
  int count = 0;
  while(data_iter != distData.end()){
    pcl::PointXYZ inputPoint((*data_iter)[0],(*data_iter)[1],(*data_iter)[2]);
    pc_iter = pc->insert(pc_iter, inputPoint);
    ++data_iter;
  }

  pcl::toROSMsg(*pc, pc_out);
  std::cout<<"make cloud done"<<std::endl;
  
  pc->clear();
}


// 디버깅 센드
void ConvertCloud::deSend(){
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    //ros::spinOnce();
    createExample();
    deConvert();
    point_pub_.publish(pc_out);
  }
}


// 디버깅 변환
void ConvertCloud::deConvert(){
  pc->clear();

  // Create header
  std::string frame_id("convert_cloud");
  pc->header.frame_id = frame_id;
  pc->header.seq = ros::Time::now().toNSec()/1e3;
  //pc->header.stamp = ros::Time();

  pcl::PointCloud<pcl::PointXYZ>::iterator pc_iter = pc->begin();
  std::vector<cv::Vec3f>::iterator data_iter = distData.begin();
    
  int count = 0;
  while(data_iter != distData.end()){
    pcl::PointXYZ inputPoint((*data_iter)[0],(*data_iter)[1],(*data_iter)[2]);
    pc_iter = pc->insert(pc_iter, inputPoint);
    ++data_iter;
  }

  pcl::toROSMsg(*pc, pc_out);
  std::cout<<"!!!make cloud done"<<std::endl;
    

  pc->clear();
}


// 디버깅 데이터 생성
void ConvertCloud::createExample(){
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
