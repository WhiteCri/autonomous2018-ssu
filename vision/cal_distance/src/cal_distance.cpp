#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "coordinate.h"
#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

static const bool DEBUG = false;
static std::string groupName;


/*
  1. 픽셀 파일 읽기 => 2차원 백터
  1'. 변환 함수(pixel -> 실좌표)
  struct pos
  2. 무한루프
*/

using std::vector;

class Transformer{
public:
    Transformer(std::string pixelName, std::string lineName){
      //save Center
      parsePixel(pixelName, fileVec, center);
      parseLine(lineName, hlines, vlines);
      ROS_INFO("load lines end...");

      this->realVec.resize(fileVec.size());
      realVec[0].emplace_back(center.x, center.y); // set top-left realcoordinate
      for (size_t i = 1 ; i < fileVec[0].size(); ++i)
        realVec[0].emplace_back(
          realVec[0][i-1].x ,
          realVec[0][i-1].y - 0.5
        );
      for(size_t i = 1 ; i < fileVec.size(); ++i){
        for( size_t j = 0 ; j < fileVec[0].size(); ++j){
          realVec[i].emplace_back(
            realVec[i-1][j].x - 0.5,
            realVec[i-1][j].y
          );
        }
      }

      ROS_INFO("setting real-coordinates end...");

      //debuging code
      {
        //make string from fileVec, realVec, linevec

        //filevec
        std::stringstream ss;
        for (auto& posVec: fileVec){
          for (auto& pos : posVec){
            ss << "(" << pos.x << ',' << pos.y << ")";
          }
          ss << std::endl;
        }
        ROS_INFO("---filevec---  \n%s", ss.str().c_str());

        //realvec
        ss.str("");
        for(auto& posVec : realVec){
          for(auto& pos : posVec){
            ss << "(" << pos.x << ',' << pos.y << ")";
          }
          ss << std::endl;
        }
        ROS_INFO("---realVec--- \n%s", ss.str().c_str());

        //hlines, vlines
        ss.str("");
        for(auto& line : hlines)
          ss << "(" << line.slope << ',' << line.intercept << ")";
        ROS_INFO("---hlines--- \n%s", ss.str().c_str());
      }
      //debuging code end
    }

    Pos pixel_to_real(const Pos& pos){
      int idx_x = -1, idx_y = -1;
      //find upper y (in pixel coordinate)
      for(size_t i = 0 ; i < fileVec.size(); ++i){
        if (fileVec[i][0].y > pos.y) {
          idx_y = i;
          break;
        }
      }

      //find upper x (in pixel coordinate)
      for(size_t i = 0 ; i < fileVec[0].size(); ++i){
        if (fileVec[idx_y][i].x > pos.x) {
          idx_x = i;
          break;
        }
      }
      //exception handling
      if (idx_y <= 0) return Pos(0,0);
      else if (idx_x <= 0) return Pos(0,0);
      //to avoid divide by zero, throw away when hlines.slope == 0
      if ((hlines[idx_y].slope == 0) || (hlines[idx_y-1].slope == 0)) return Pos(0,0);


      // calc top-left coordinate
      Pos real_dr = realVec[idx_y][idx_x];

      // 타겟 지점부터 상하좌우 직선 좌표차
      double up, down, right, left;
      up   = abs( pos.y - ( hlines[idx_y - 1].slope * pos.x + hlines[idx_y-1].intercept ) );
      down = abs( pos.y - ( hlines[idx_y].slope  * pos.x + hlines[idx_y].intercept ) );
      right = abs( pos.x - ( pos.y - vlines[idx_x].slope ) / vlines[idx_x].intercept );
      left = abs( pos.x -  ( pos.y - vlines[idx_x - 1].slope ) / vlines[idx_x - 1].intercept );

      Pos distance( real_dr.y +  0.5 * down * ( up + down ), real_dr.x - 0.5 * right * ( left + right ) );
      return distance;
    }
private:
  Pos center;
  vector<vector<Pos> > fileVec;
  vector<vector<Pos> > realVec;
  vector<Line> hlines; // 수직방향 직선
  vector<Line> vlines; //수평방향 직선
};

class CalDistance{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
public:
    CalDistance():
      transformer("./calibration.txt", "./calibrationLine.txt")
    {
      // 싱글카메라
      sub_ = nh_.subscribe("/cam1/lane",100,&CalDistance::laneCb,this);
      pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/cam1/dist", 100);

      // 멀티카메라
      // sub_ = nh_.subscribe("/"+groupName+"/lane",100,&CalDistance::laneCb,this);
      // pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/"+groupName+"/dist", 100);
    }
    void laneCb(const std_msgs::Int32MultiArray::ConstPtr& laneData);
    void sendDist();
    ros::NodeHandle getNh();
private:
    std::vector<float> laneXData;
    std::vector<float> laneYData;
    std_msgs::Float32MultiArray distData;
    float yDist;
    float xDist;
    int size;
    Transformer transformer;
};


int main(int argc, char** argv){
    ros::init(argc, argv, "cal_distance");
    groupName = argv[1];

    CalDistance calDist;
    while(calDist.getNh().ok()){
        calDist.sendDist();
        ros::spinOnce();
    }
    return 0;
}

void CalDistance::sendDist(){

    distData.data.clear();
    distData.data.resize(0);
    std::vector<float>::iterator itX = laneXData.begin();
    std::vector<float>::iterator itY = laneYData.begin();

    distData.data.push_back((float)size);
    while( (itX!=laneYData.end())&&(itY!=laneYData.end()) ){
        distData.data.push_back((*itX));
        distData.data.push_back((*itY));
        ++itX;
        ++itY;
    }

    pub_.publish(distData);

}

void CalDistance::laneCb(const std_msgs::Int32MultiArray::ConstPtr& laneData){
    if(DEBUG) std::cout<<"start call back"<<std::endl;

    laneXData.clear();
    laneYData.clear();

    std::vector<int>::const_iterator it;

    if(DEBUG) std::cout<<"before push_back"<<std::endl;

    it = laneData->data.begin();

    size = (*it);
    ++it;

    //why try + catch?
    while(it != laneData->data.end()){
        Pos targetPixel;
          Pos targetDist;
            targetPixel.x = (*it);
            ++it;
            targetPixel.y = (*it);
            ++it;

            targetDist = transformer.pixel_to_real(targetPixel);
            if ((targetDist.x == 0) && (targetDist.y == 0)) continue; //when transformer() failed, continue;

            laneXData.push_back(targetDist.x);
            laneYData.push_back(targetDist.y);
    }

    if(DEBUG) {
        std::cout<<"size : "<<size<<std::endl;
        for(int i=0; i<size/2; i++){
            std::cout<<"X : "<<laneXData[i]<<" / Y : "<<laneYData[i]<<std::endl;
        }
    }
}

ros::NodeHandle CalDistance::getNh(){ return nh_; }
