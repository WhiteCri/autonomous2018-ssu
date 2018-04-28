#include <vector>
#include <iostream>
#include <sstream>
#include <string>
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

using namespace std;

struct Pos{
  double x,y;
};

class Transformer{
public:
  Transform(std::string& filename) : fileVec(0){
    std::ifstream ifs(filename);
    int i=0;

    //save Center
    ifs >> center.x >> center.y;

    //flush head
    std::string temp;
    std::getline(ifs, temp);
      
    while(true){
      std::string temp;
      std::getline(ifs, temp);
      if(ifs.eof()) break;

      fileVec.push_back(std::vector<Pos>(0));
      std::stringstream ss(temp);
      std::string word="";
      while(word[0] != '\n'){
        Pos p;
        std::getline(ss, word, ',');
        p.x = std::stod(word);
        std::getline(ss, word, ',');
        p.y = std::stod(word);
        fileVec[i].push_back(p);
      }
      i++;
    }

    this->realVec = fileVec;
    fileVec[0].emplace_back(center.x, center.y); // 맨 윗 실좌표 구성
    for (size_t i = 1 ; i < fileVec[0].size(); ++i)
      realVec[0].emplace_back(
        realVec[i-1].x ,
        realVec[i-1].y - 0.5
      );
    for(size_t i = 1 ; i < fileVec.size(); ++i){
      for( size_t j = 0 ; j < fileVec[0].size(); ++j){
        realVec[i].emplace_back(
          realVec[i-1].x - 0.5,
          realVec[i-1].y
        );
      }
    }
  }

  Pos operator()(const Pos& pos){
    int idx_x = -1, idx_y = -1;
    //find upper y (in pixel coordinate)
    for(size_t i = 0 ; i < fileVec.size(); ++i){
      if (fileVec[i] > pos.y) {
        idx_y = i;
        break;
      }
    }
    //find upper x (in pixel coordinate)
    for(size_t i = 0 ; i < fileVec[0].size(); ++i){
      if (fileVec[idx_y][i] > pos.x) {
        idx_x = i;
        break;
      }
    }
    //exception handling
    if (idx_y <= 0) return Pos(0,0);
    else if (idx_x <= 0) return Pos(0,0);

    // 주위 네 점 구하기
    Pos file_ur, file_ul, file_dr, file_dl;
    Pos real_ur, real_ul, real_dr, real_dl;

    file_dr = fileVec[idx_y][idx_x];
    file_dl = fileVec[idx_y][idx_x - 1];
    file_ur = fileVec[idx_y - 1][idx_x];
    file_ul = fileVec[idx_y - 1][idx_x - 1];

    real_dr = realVec[idx_y][idx_x];
    real_dl = realVec[idx_y][idx_x - 1];
    real_ur = realVec[idx_y - 1][idx_x];
    real_ul = realVec[idx_y - 1][idx_x - 1];

    // 타겟 지점부터 상하좌우 직선 좌표차
    double up, down, right, left;
    up   = abs( pos.y - ( HGradient[idx_y - 1] * pos.x + HIntercept[idx_y-1] ) );
    down = abs( pos.y - ( HGradient[idx_y]     * pos.x + HIntercept[idx_y] ) );
    right = abs( pos.x - ( pos.y - VIntercept[idx_x] ) / VGradient[idx_x] );
    left = abs( pos.x -  pos.y - VIntercept[idx_x - 1] ) / VGradient[idx_x - 1] );

    Pos distance( real_dr.y +  0.5 * down * ( up + down ), real_dr.x - 0.5 * right * ( left + right ) );

    return distance;
  }

  void laneCb(const std_msgs::Int32MultiArray::ConstPtr& laneData);
  void sendDist();

private:
  Pos center;
  std::vector<vector<Pos> > fileVec;
  std::vector<vector<Pos> > realVec;
  std::vector<double> VGradient; // 수직 기울기
  std::vector<double> HGradient; // 수평 기울기
  std::vector<double> VIntercept; // 수직 Y절편
  std::vector<double> HIntercept; // 수평 Y절편

}

class CalDistance{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
public:
    CalDistance()
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

    //file read
    std::ifstream ifs("../calibration.txt");
    std::stringstream ss;

    CalDistance calDist;
    while(calDist.getNh().ok()){
        calDist.sendDist();
        ros::spinOnce();
    }

    return 0;
}

void CalDistance::sendDist(){

    distData.data.clear();

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

    while(it != laneData->data.end()){
        try{
            Pos targetPixel;
            Pos targetDist;
            targetPixel.x = (*it);
            ++it;
            targetPixel.y = (*it);
            ++it;

            targetDist = transformer.(targetPixel);
            laneXData.push_back(targetDist.x);
            laneYData.push_back(targetDist.y);

        }
        catch(std::exception& e){
            break;
        }
    }

    if(DEBUG) {
        std::cout<<"size : "<<size<<std::endl;
        for(int i=0; i<size/2; i++){
            std::cout<<"X : "<<laneXData[i]<<" / Y : "<<laneYData[i]<<std::endl;
        }
    }


}

ros::NodeHandle CalDistance::getNh(){ return nh_; }
