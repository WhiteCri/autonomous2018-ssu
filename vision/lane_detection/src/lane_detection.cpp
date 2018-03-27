//warpAffine 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <preprocessing/preLaneDetect.hpp>
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int32MultiArray.h"
//#include <lanedetection/coordinateMsg.h>
//#include <lanedetection/vecMsg.h>
//#include <lanedetection/dataSize.h>
// 매크로 상수
#define H_MIN 8
#define H_MAX 20
#define S_MIN 77
#define S_MAX 154
#define V_MIN 10
#define V_MAX 164


// 매크로 상수
#define H_MIN 8
#define H_MAX 20
#define S_MIN 77
#define S_MAX 154
#define V_MIN 10
#define V_MAX 164


static const std::string OPENCV_WINDOW_VF = "Image by videofile";
static const std::string OPENCV_WINDOW_WC = "Image by webcam";
static const bool DEBUG_SW = false;
static const bool IMSHOW_SW = false;
static const bool TRACK_BAR = false;
static const bool TIME_CHECK = false;

lane_detect_algo::vec_mat_t lane_m_vec;

int check = 0;
using namespace lane_detect_algo;
class InitImgObjectforROS{
    public:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub_img;
        std_msgs::Int32MultiArray coordi_array;
        cv::Mat pub_img;
        ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("/cam1/lane",100);//파라미터로 카메라 번호 받도록하기
        
        InitImgObjectforROS();
        ~InitImgObjectforROS();
        void imgCb(const sensor_msgs::ImageConstPtr& img_msg);
};



InitImgObjectforROS::InitImgObjectforROS():it(nh){
            if(DEBUG_SW){//'DEBUG_SW == TURE' means subscribing videofile image
                sub_img = it.subscribe("/videofile/image_raw",1,&InitImgObjectforROS::imgCb,this);
            }
            else{//'DEBUG_SW == FALE' means subscribing webcam image
                sub_img = it.subscribe("/cam1/raw_image",1,&InitImgObjectforROS::imgCb,this);
            }
}


InitImgObjectforROS::~InitImgObjectforROS(){
                if(DEBUG_SW){//'DEBUG_SW == TURE' means subscribing videofile image
                cv::destroyWindow(OPENCV_WINDOW_VF);
            }
            else{//'DEBUG_SW == FALE' means subscribing webcam image
                cv::destroyWindow(OPENCV_WINDOW_WC);
            }
}


void InitImgObjectforROS::imgCb(const sensor_msgs::ImageConstPtr& img_msg){
            cv_bridge::CvImagePtr cv_ptr;
            cv::Mat frame, canny_img, gray, yellow_hsv, 
                    yellow_thresh, white_thresh,yellow_labeling,white_labeling, laneColor, origin;
            uint frame_height, frame_width;
            try{
                cv_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
                frame = cv_ptr->image;
                origin = cv_ptr->image;
                if(!frame.empty()){
                cv::resize(origin,frame,cv::Size(320,240),0,0,CV_INTER_AREA);
                /*another solution->*/ //cv::resize(frame, frame, cv::Size(), 0.2, 0.2 320 240); 
                frame_height = (uint)frame.rows;
                frame_width = (uint)frame.cols;
                unsigned int* H_yResultYellow = new unsigned int[frame_width];
                std::memset(H_yResultYellow, 0, sizeof(uint) * frame_width);
                unsigned int* H_yResultWhite = new unsigned int[frame_width];
                std::memset(H_yResultWhite, 0, sizeof(uint) * frame_width);
                unsigned int* H_xResultYellow = new unsigned int[frame_height];
                std::memset(H_xResultYellow, 0, sizeof(uint) * frame_height);
                unsigned int* H_xResultWhite = new unsigned int[frame_height];
                std::memset(H_xResultWhite, 0, sizeof(uint) * frame_height);
                 if(TRACK_BAR) {
                    cv::namedWindow("TRACKBAR", cv::WINDOW_AUTOSIZE);
                    int hmin, hmax, smin, smax, vmin, vmax;

                    cv::createTrackbar("h min", "TRACKBAR", &hmin, 50, NULL);
                    cv::setTrackbarPos("h min", "TRACKBAR", 7);


                    cv::createTrackbar("h max", "TRACKBAR", &hmax, 50, NULL);
                    cv::setTrackbarPos("h max", "TRACKBAR", 21);

                    cv::createTrackbar("s min", "TRACKBAR", &smin, 255, NULL);
                    cv::setTrackbarPos("s min", "TRACKBAR", 52);

                    cv::createTrackbar("s max", "TRACKBAR", &smax, 255, NULL);
                    cv::setTrackbarPos("s max", "TRACKBAR", 151);

                    cv::createTrackbar("v min", "TRACKBAR", &vmin, 255, NULL);
                    cv::setTrackbarPos("v min", "TRACKBAR", 0);

                    cv::createTrackbar("v max", "TRACKBAR", &vmax, 255, NULL);
                    cv::setTrackbarPos("v max", "TRACKBAR", 180);
                }
                
                lane_detect_algo::CalLane callane;
                cv::Mat bev = frame.clone();
                callane.birdEyeView(frame,bev);
                cv::cvtColor(bev, gray, CV_BGR2GRAY);
                canny_img = callane.myCanny(bev);

                // 색상검출( 7 21 52 151 0 255 )
                if (TRACK_BAR) {
                    callane.detectHSVcolor(bev, yellow_hsv, H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX);
                }
                else {
                    callane.detectHSVcolor(bev, yellow_hsv, 7, 21, 52, 151, 0, 180);
                }
                cv::threshold(gray, white_thresh, 180, 255, CV_THRESH_BINARY);
                cv::threshold(yellow_hsv, yellow_thresh, 180, 255, CV_THRESH_BINARY);                

                if(IMSHOW_SW){
                    cv::imshow("threshold_yellow",yellow_thresh);
                    cv::imshow("threshold_white",white_thresh);
                }
               
                /*for histogram->*/ //cv::Mat yellowYProj(cv::Size(frame_width, frame_height), CV_8UC1);
                /*for histogram->*/ //cv::Mat whiteYProj(cv::Size(frame_width, frame_height), CV_8UC1);
                /*for histogram->*/ //cv::Mat yellowXProj(cv::Size(frame_width, frame_height), CV_8UC1);
                /*for histogram->*/ //cv::Mat whiteXProj(cv::Size(frame_width, frame_height), CV_8UC1);
                yellow_labeling = yellow_thresh.clone();
                /*for Labeling->*/ //callane.makeContoursLeftLane(yellow_thresh, yellow_labeling);
                /*for Labeling->*/ //callane.makeContoursRightLane(white_thresh, white_labeling);
                if(TIME_CHECK){
                    const int64 start = cv::getTickCount();
                    int64 elapsed = (cv::getTickCount() - start);
                    std::cout << "Elapsed time " << elapsed << std::endl;
                }
                laneColor = yellow_thresh | white_thresh;
                /*for Labling->*/ //laneColor = yellow_labeling | white_labeling; 
                cv::Mat inv_bev = frame.clone();
                callane.inverseBirdEyeView(bev, inv_bev);
                cv::Mat newlane = frame.clone();
                callane.inverseBirdEyeView(laneColor, newlane);

                if(IMSHOW_SW){
                cv::imshow("newlane", newlane);
                }

                cv::Mat output_origin = origin.clone();
                pub_img = newlane.clone();
                int coordi_count = 0;
                coordi_array.data.clear();
                coordi_array.data.push_back(10);

                for(int y = output_origin.rows-1; y>=290; y--){
                    uchar* origin_data = output_origin.ptr<uchar>(y);
                    uchar* pub_img_data = pub_img.ptr<uchar>(y*0.5);//resize 복구(0.5 -> 1))
                        for(int x = 0; x<output_origin.cols; x++){
                            int temp = x*0.5;//resize 복구(0.5 -> 1)
                            if(pub_img_data[temp]!= (uchar)0 && x%2==0){
                                coordi_count++;
                                coordi_array.data.push_back(x);
                                coordi_array.data.push_back(y);
                                origin_data[x*output_origin.channels()] = 255;
                        }
                    }
                } 

                coordi_array.data[0] = coordi_count;

                if(IMSHOW_SW){
                    imshow("myorigin",output_origin);
                }
                cv::Mat inv_lane = laneColor.clone();
                callane.inverseBirdEyeView(laneColor, inv_lane);
                if (TRACK_BAR) cv::imshow("TARCKBAR", yellow_hsv);
                float degree=0., ladian = 0.;
                int no_data;
                }
                
                else{//frame is empty()!
                    while(frame.empty()){//for unplugged camera
                        cv_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
                        frame = cv_ptr->image;
                    }
                }
            }
            catch(cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception : %s", e.what());
                return;
            }
            if(IMSHOW_SW) cv::imshow(OPENCV_WINDOW_VF,frame);
            
            int ckey = cv::waitKey(10);
            if(ckey == 27) exit(1); 
}


int main(int argc, char **argv){
    ros::init(argc, argv, "lane_detection");
    ros::NodeHandle nh_;
    InitImgObjectforROS img_obj;
    ros::Rate loop_rate(15);

    while(img_obj.nh.ok()){
        img_obj.pub.publish(img_obj.coordi_array);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("program killed!\n");
    return 0;
}



