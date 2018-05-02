// Complile : g++ calibration_test.cpp -o calibration_test `pkg-config --cflags --libs opencv` -std=c++11


#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "opencv2/opencv.hpp"
#include "../include/coordinate.h"


#include <stdio.h>
#include <unistd.h>

static int debug;
static std::string groupName;
static std::string calPath;



static const bool CAPTURE = false;
static const bool VIDEO = false;
static const bool IMAGE = true;

using std::vector;
using namespace std;
using namespace cv;

class Transformer{
public:
    Transformer(){}
    Transformer(std::string pixelName, std::string lineName){
      //save Center
      parsePixel(pixelName, fileVec, center);
      parseLine(lineName, hlines, vlines);
      cout<<"size of row : "<<fileVec.size()<<endl;
      cout<<"size of col : "<<fileVec[0].size()<<endl;


      // cout<<"file Vec"<<endl;
      // for(int i=0; i<fileVec.size(); i++){
      //   cout<<"["<<i<<"]  ";
      //   for(int j=0; j<fileVec[0].size(); j++){
      //     cout<<"x :"<<fileVec[i][j].x << "y :"<<fileVec[i][j].y<<"   ";
      //   }
      //   cout<<endl;
      // }

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

      //debuging code
      // {
      //   //make string from fileVec, realVec, linevec
      //
      //   //filevec
      //   std::stringstream ss;
      //   for (auto& posVec: fileVec){
      //     for (auto& pos : posVec){
      //       ss << "(" << pos.x << ',' << pos.y << ")";
      //     }
      //     ss << std::endl;
      //   }
      //
      //   //realvec
      //   ss.str("");
      //   for(auto& posVec : realVec){
      //     for(auto& pos : posVec){
      //       ss << "(" << pos.x << ',' << pos.y << ")";
      //     }
      //     ss << std::endl;
      //   }
      //
      //   //hlines, vlines
      //   ss.str("");
      //   for(auto& line : hlines)
      //     ss << "(" << line.slope << ',' << line.intercept << ")";
      // }
      //debuging code end

      cout<<"file Vec"<<endl;
      for(int i=0; i<fileVec.size(); i++){
        cout<<"["<<i<<"]  ";
        for(int j=0; j<fileVec[0].size(); j++){
          cout<<"x :"<<fileVec[i][j].x << "y :"<<fileVec[i][j].y<<"   ";
        }
        cout<<endl;
      }

      cout<<"Real Vec"<<endl;
      for(int i=0; i<fileVec.size(); i++){
        cout<<"["<<i<<"]  ";
        for(int j=0; j<realVec[0].size(); j++){
          cout<<"x :"<<realVec[i][j].x << "y :"<<realVec[i][j].y<<"   ";
        }
        cout<<endl;
      }

      cout<<"[hlines]"<<endl;
      cout<<"intercept : ";
      for(int i=0; i<hlines.size(); i++){
        cout<<hlines[i].intercept<<"  ";
      }
      cout<<endl;
      cout<<"slope : ";
      for(int i=0; i<hlines.size(); i++){
        cout<<hlines[i].slope<<"  ";
      }
      cout<<endl;

      cout<<"[vlines]"<<endl;
      cout<<"intercept : ";
      for(int i=0; i<vlines.size(); i++){
        cout<<vlines[i].intercept<<"  ";
      }
      cout<<endl;
      cout<<"slope : ";
      for(int i=0; i<vlines.size(); i++){
        cout<<vlines[i].slope<<"  ";
      }
      cout<<endl;


      cout<<"center : "<<center.x <<" / "<<center.y;
    }

    Pos pixel_to_real(const Pos& pos){
      cout<<"11"<<endl;
      int idx_x = -1, idx_y = -1;
      //find upper y (in pixel coordinate)
      cout<<pos.x<<" "<<pos.y<<endl;
      for(size_t i = 0 ; i < fileVec.size(); ++i){
        if (fileVec[i][0].y > pos.y) {
          idx_y = i;
          break;
        }

      }
      cout<<"22 idx_y : "<<idx_y<<endl;
      // exception handling
      if (idx_y <= 0) return Pos(0,0);


      //find upper x (in pixel coordinate)
     for(size_t i = 0 ; i < fileVec[0].size(); ++i){
        if (fileVec[idx_y][i].x > pos.x) {
          cout<<fileVec[idx_y][i].x<<" ";
          idx_x = i;
          break;
        }
      }

      cout<<"33 idx_x : "<<idx_x<<endl;
      if (idx_x <= 0) return Pos(0,0);

      // //to avoid divide by zero, throw away when hlines.slope == 0
      if ((hlines[idx_y].slope == 0) || (hlines[idx_y-1].slope == 0)) return Pos(0,0);
      cout<<"44"<<endl;

      // calc top-left coordinate
      Pos real_dr = realVec[idx_y][idx_x];

      // 타겟 지점부터 상하좌우 직선 좌표차
      double up, down, right, left;
      up   = abs( pos.y - ( hlines[idx_y - 1].slope * pos.x + hlines[idx_y-1].intercept ) );
      down = abs( pos.y - ( hlines[idx_y].slope  * pos.x + hlines[idx_y].intercept ) );

      right = abs( pos.x - ( pos.y - vlines[idx_x].intercept) / vlines[idx_x].slope ); // 1

      // px - ( py - b) / a
      left = abs( pos.x -  ( pos.y - vlines[idx_x - 1].intercept ) / vlines[idx_x - 1].slope); // 0

      cout <<"up : "<< up << "/ down : " << down << " / left : " << left << " / right : "<<right<<endl;
      cout<<"55"<<endl;
      Pos distance( real_dr.x+  0.5 * down / ( up + down ), real_dr.y + 0.5 * right / ( left + right ) );
      cout<<"dist:"<<distance.x <<" / "<<distance.y<<endl;
    }
private:
  Pos center;
  vector<vector<Pos> > fileVec;
  vector<vector<Pos> > realVec;
  vector<Line> hlines; // 수직방향 직선
  vector<Line> vlines; //수평방향 직선
};

void mouseEvent( int event, int x, int y, int flags, void* userdata );

Transformer trans;

int main(int argc, char** argv){

    int camera_number = 1;
    int frequency = 30;
    VideoCapture cap(0);
    int key;
    Mat src;
    cout<<"!!"<<endl;
    trans = Transformer("../data/main_calibration.txt", "../data/main_calibrationLine.txt");



    if(IMAGE){
      Point2f origin(377, 837);

      while(1){
        cout<<"!!"<<endl;
        src = imread("./2.jpg", CV_LOAD_IMAGE_COLOR);

        imshow("origin",src);

        setMouseCallback("origin", mouseEvent, &src);

        key = waitKey();
        if(key == 27) break;
      }
    }
    return 0;
}

void mouseEvent( int event, int x, int y, int flags, void* userdata ){
  if(event == EVENT_LBUTTONDOWN){
    cout<<"x : "<<x <<endl;
    cout<<"y : "<<y<<endl;
    trans.pixel_to_real(Pos(x,y));
  }
}
