#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#define X_CENTER 526.0
#define Y_CENTER 509.0

// xGap = xCenter - xTarget, yGap = yTarget - yCenter

static const bool DEBUG = true;

class CalDistance{
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
public:
    CalDistance()
        : xCenter(X_CENTER), yCenter(Y_CENTER), yGap(0.0), yGap50(0.0), yDist(0.0), xGap(0.0), xDist(0.0)
    {   
        sub_ = nh_.subscribe("/cam0/lane",100,&CalDistance::laneCb,this);
        pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/cam0/dist", 100);
    }
    void calXdist();
    void calYgap50();
    void calYdist();    
    void laneCb(const std_msgs::Int32MultiArray::ConstPtr& laneData);
    void sendDist();


private:
    std::vector<float> laneXData;
    std::vector<float> laneYData;
    std_msgs::Float32MultiArray distData;
    float xCenter;
    float yCenter;
    float yGap;
    float yGap50;
    float yDist;
    float xGap;
    float xDist;
    int size;
};


int main(int argc, char** argv){
    ros::init(argc, argv, "cal_distance");

    CalDistance calDist;
    ros::spin();

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


void CalDistance::calXdist(){
    xDist = 0.4819*std::exp(0.0066*xGap);
}

void CalDistance::calYgap50(){ 
    yGap50 = 0.0146*std::pow(xDist, 6) - 0.3673*std::pow(xDist, 5) + 3.8374*std::pow(xDist, 4) 
        - 21.814*std::pow(xDist, 3) + 74.873*std::pow(xDist, 2) - 167.26*xDist + 271.67;
}

void CalDistance::calYdist(){
    yDist = 0.5*yGap / yGap50;
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
            xGap = xCenter - (*it);
            calXdist();
            laneXData.push_back(xDist);
            ++it;

            yGap = (*it)-yCenter;
            calYdist();
            calYgap50();
            laneYData.push_back(yDist);
            ++it;       

        }
        catch(std::exception& e){
            break;
        }
    }

    if(DEBUG) {    
        std::cout<<"size : "<<size<<std::endl;
        for(int i=0; i<size; i++){
            std::cout<<"X : "<<laneXData[i]<<" / Y : "<<laneYData[i]<<std::endl;
        }
    }
    
    sendDist();
}

