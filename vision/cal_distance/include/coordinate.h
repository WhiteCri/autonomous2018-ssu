#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <exception>
#include <algorithm>
#include <ros/ros.h>
#define MAX_SLOPE 9999

using std::vector;


struct Pos{
  Pos(): x(0), y(0) {}
  Pos(int x, int y) : x(x), y(y) {}
  Pos(double x, double y): x(x), y(y) {}
  double x;
  double y;
};

struct Line{
  void parseLine(std::string filename);
  double slope;
  double intercept;
};

std::istream& operator>>(std::istream& is, Pos& pos){
  char ch;
  is >> pos.x >> ch >> pos.y >> ch;
  return is;
}

std::istream& operator>>(std::istream& is, Line& line){
  char ch;
  is >> line.slope >> ch >> line.intercept >> ch;
  return is;
}

void parsePixel(std::string filename, vector<vector<Pos> >& posVec, Pos& center){

  std::ifstream in(filename);
  std::string err= "file open error! in parsePixel... : " + filename;
  if(in.fail()) throw std::runtime_error(err.c_str());
  //initialize center
  char ch;
  std::string str;
  in >> center.x >> ch >> center.y;
  in >> str;//flush newline

  //initialize pixel
  while(!in.eof()){
    posVec.emplace_back(0); // append

    std::stringstream ss(str);
    while(!ss.eof()){
      Pos p;
      ss >> p;
      posVec.back().push_back(p);
    }
    in >> str;
  }
}

void parseLine(std::string filename, vector<Line> & hlines, vector<Line>& vlines){

  std::ifstream in(filename);
  if(in.fail()) throw std::runtime_error("file open error! in parseLine...");
  std::string str;

  //parse hlines
  in >> str;
  std::stringstream ss(str);
  while(!ss.eof()){
    Line line;
    ss >> line;
    hlines.push_back(line);
  }

  //parse vlines
  in >> str;
  ss.clear(); ss.str(str);
  while(!ss.eof()){
    Line line;
    ss >> line;
    vlines.push_back(line);
  }
}

struct Box{
    struct Pos{
        Pos() : x(0.0), y(0.0) {}
        Pos(double x, double y): x(x), y(y) {}
        double x, y;
    };
    Box(const std::vector<double>&& pos){
        if (pos.size() != 8) { //assert
            ROS_ERROR("Box() failed because of the lack pos information : %ld", pos.size());
            exit(-1);
        }

        for (int i = 0 ; i < 8 ; ++i){ //save
            if (i%2) posAry[(i/2)].y = pos[i];
            else posAry[(i/2)].x = pos[i];
        }

        std::sort(posAry, posAry+4, [](const Pos& f, const Pos& s){
            return f.x < s.x;
        }); //sort by x
        
        //sort by y for [0] [1], [2] [3]. 
        //then the result in posAry would be top-left, bottom-left, bottom-right, bottom-up
        if (posAry[0].y < posAry[1].y) std::swap(posAry[0], posAry[1]);
        if (posAry[2].y > posAry[3].y) std::swap(posAry[2], posAry[3]);

        for (int i = 0 ; i < 4; ++i)
            ROS_INFO("result of square : %lf, %lf", posAry[i].x, posAry[i].y);
    }
    
    bool in(double x, double y){
        //we can divide square to the triangle of two.
        //then, when the pos is in the triangles, we can say that the pos is in the triangle.
        //the condition of the point and triangle is based on the area of triangle
        static auto calc_triangle_area = [](Pos f, Pos s, Pos t)->double { //first, second, third
            double area = fabs((f.x*(s.y-t.y) + s.x*(t.y-f.y) + t.x*(f.y-s.y))/2);
            return area;
        };
        static double EPSILON = 0.00001;

        Pos p(x,y);
        //0,1,2 2,3,0
        double area_with_point1 = 
            calc_triangle_area(p, posAry[0], posAry[1]) +
            calc_triangle_area(p, posAry[1], posAry[2]) +
            calc_triangle_area(p, posAry[2], posAry[0]);
        double area_without_point1 = 
            calc_triangle_area(posAry[0], posAry[1], posAry[2]);
        if (fabs(area_with_point1 - area_without_point1) <= EPSILON) return true; //the point is in first triangle

        double area_with_point2 = 
            calc_triangle_area(p, posAry[2], posAry[3]) +
            calc_triangle_area(p, posAry[3], posAry[0]) +
            calc_triangle_area(p, posAry[0], posAry[2]);
        double area_without_point2 = 
            calc_triangle_area(posAry[2], posAry[3], posAry[0]);
        if (fabs(area_with_point2 - area_without_point2) <= EPSILON) return true; //the point is in first triangle
        return false;
    }
public:
    Pos posAry[4];
};