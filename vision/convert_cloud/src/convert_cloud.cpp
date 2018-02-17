

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
#include "opencv2/opencv.hpp"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <vector>
#include <ctime>

static const bool DEBUG = true;


class ConvertCloud{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

public:
    ConvertCloud(){
        sub_ = nh_.subscribe("/cam0/dist",100,&ConvertCloud::distCb,this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cam0/point_cloud",10);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pc = ref_pc;
    }
  void distCb(const std_msgs::Float32MultiArray::ConstPtr& distData);
  void parseVec();
  void convert();


private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc;
  sensor_msgs::PointCloud2 pc_out;
  std::vector<cv::Vec3f> distVec;
  std::vector<float> distXdata;
  std::vector<float> distYdata;
  int size;


};

int main(int argc, char** argv){
  ros::init(argc,argv,"convert_cloud");
  ConvertCloud cvtCloud;
  ros::spin();

  return 0;
}

void ConvertCloud::distCb(const std_msgs::Float32MultiArray::ConstPtr& distData){    

    if(DEBUG) std::cout<<"start call back"<<std::endl;
    
    distXdata.clear();
    distYdata.clear();

    std::vector<float>::const_iterator it;

    if(DEBUG) std::cout<<"before push_back"<<std::endl;
    
    it = distData->data.begin();

    size = (int)(*it);
    ++it;

    while(it != distData->data.end()){
        try{
            distXdata.push_back(*it);
            ++it;

            distYdata.push_back(*it);
            ++it;       
        }
        catch(std::exception& e){
            break;
        }
    }

    if(DEBUG) {    
        std::cout<<"size : "<<size<<std::endl;
        for(int i=0; i<distXdata.size(); i++){
            std::cout<<"X : "<<distXdata[i]<<" / Y : "<<distYdata[i]<<std::endl;
        }
    }

    parseVec();
    convert();
}


void ConvertCloud::parseVec(){
    distVec.clear();
    std::vector<float>::iterator itX = distXdata.begin();
    std::vector<float>::iterator itY = distYdata.begin();
    while( (itX!=distYdata.end())&&(itY!=distYdata.end()) ){
        if(DEBUG){
           std::cout<<"parse X : "<<(*itX)<<" / parse Y : "<<(*itY)<<std::endl;
        }
        distVec.push_back(cv::Vec3f( (*itX),(*itY),0) );
        ++itX;
        ++itY;
    }
    if(DEBUG){
        std::cout<<"parse done"<<std::endl;
    }
}


// 클라우드 변환
void ConvertCloud::convert(){
  pc->clear();

  // Create header
  std::string frame_id("convert_cloud");
  pc->header.frame_id = frame_id;
  pc->header.seq = ros::Time::now().toNSec()/1e3;
  //pc->header.stamp = ros::Time();

  pcl::PointCloud<pcl::PointXYZ>::iterator pc_iter = pc->begin();
  std::vector<cv::Vec3f>::iterator data_iter = distVec.begin();
    
  int count = 0;
  while(data_iter != distVec.end()){
    pcl::PointXYZ inputPoint((*data_iter)[0],(*data_iter)[1],(*data_iter)[2]);
    pc_iter = pc->insert(pc_iter, inputPoint);
    ++data_iter;
  }

  pcl::toROSMsg(*pc, pc_out);
  if(DEBUG) std::cout<<"make cloud done"<<std::endl;
  
  pc->clear();
  pub_.publish(pc_out);

}

