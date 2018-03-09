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
static const bool TRACK_BAR = false;
lane_detect_algo::vec_mat_t lane_m_vec;

int check = 0;
using namespace lane_detect_algo;

//for WebCam
//for VideoFile
class InitImgObjectforROS{
    public:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub_img;
        std_msgs::Int32MultiArray coordi_array;
        cv::Mat pub_img;
        ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("/cam0/lane",100);//파라미터로 카메라 번호 받도록하기
        InitImgObjectforROS():it(nh){
            if(DEBUG_SW){//'DEBUG_SW == TURE' means subscribing videofile image
                sub_img = it.subscribe("/videofile/image_raw",1,&InitImgObjectforROS::imgCb,this);
            //this topic is publish by s'video_stream_opencv' package please run video_file.launch of 
              //  cv::namedWindow(OPENCV_WINDOW_VF);
            }
            else{//'DEBUG_SW == FALE' means subscribing webcam image
                sub_img = it.subscribe("/cam0/raw_image",1,&InitImgObjectforROS::imgCb,this);
               // cv::namedWindow(OPENCV_WINDOW_WC);
            }
        }
        ~InitImgObjectforROS(){
            if(DEBUG_SW){//'DEBUG_SW == TURE' means subscribing videofile image
           //     cv::destroyWindow(OPENCV_WINDOW_VF);
            }
            else{//'DEBUG_SW == FALE' means subscribing webcam image
             //   cv::destroyWindow(OPENCV_WINDOW_WC);
            }
        }
        void imgCb(const sensor_msgs::ImageConstPtr& img_msg){
            cv_bridge::CvImagePtr cv_ptr;
            cv::Mat frame, canny_img, gray, yellow, yellow2, yellow3, white, white2, white3, laneColor;
            
            
            uint frame_height, frame_width;
            try{
                cv_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
                frame = cv_ptr->image;
                if(!frame.empty()){
             //   cv::resize(frame, frame, cv::Size(), 0.2, 0.2); 
              cv::resize(frame,frame,cv::Size(200,200),0,0,CV_INTER_AREA);
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
                    callane.detectHSVcolor(bev, yellow, H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX);
                }
                else {
                    callane.detectHSVcolor(bev, yellow, 7, 21, 52, 151, 0, 180);
                }
                cv::threshold(gray, white, 180, 255, CV_THRESH_BINARY);
                //cv::imshow("before_yellow",yellow);
                cv::Mat before_yellow = yellow.clone();
                //cv::threshold(yellow, before_yellow, 180, 255, CV_THRESH_BINARY);
                //cv::imshow("yellow_threshold",before_yellow);
                //cv::dilate(yellow, yellow2, NULL, cv::Point(-1, -1), 1);
                //cv::imshow("yellow_dilate",yellow2);
                cv::adaptiveThreshold(yellow, // 입력영상
                yellow2, // 이진화 결과 영상
                255, // 최대 화소 값
                cv::ADAPTIVE_THRESH_MEAN_C, // Adaptive 함수
                cv::THRESH_BINARY_INV, // 이진화 타입
                11,  // 이웃크기
                2);
                cv::adaptiveThreshold(white, // 입력영상
                white2, // 이진화 결과 영상
                255, // 최대 화소 값
                cv::ADAPTIVE_THRESH_MEAN_C, // Adaptive 함수
                cv::THRESH_BINARY_INV, // 이진화 타입
                11,  // 이웃크기
                6);//threshold
                //H  :색의 종류를 나타냄. S : 0이면 무채색(gray 색), 255면 가장 선명한(순수한) 색
                //V  : 작을수록 어둡고 클수록 밝은 색
                // 히스토그램
            //    cv::imshow("adaptive_yellow",yellow2);
                cv::Mat yellowYProj(cv::Size(frame_width, frame_height), CV_8UC1);
                cv::Mat whiteYProj(cv::Size(frame_width, frame_height), CV_8UC1);
                cv::Mat yellowXProj(cv::Size(frame_width, frame_height), CV_8UC1);
                cv::Mat whiteXProj(cv::Size(frame_width, frame_height), CV_8UC1);

                yellow3 = yellow2.clone();
                
                const int64 start = cv::getTickCount();/////////////time check///////////////////////
                callane.makeContoursLeftLane(yellow2, yellow3);
                callane.makeContoursRightLane(white2, white3);

                int64 elapsed = (cv::getTickCount() - start);////////time check//////////////////////
               // std::cout << "Elapsed time " << elapsed << std::endl;

                laneColor = yellow3 | white3;

                cv::Mat inv_bev = frame.clone();
             //   imshow("my_bev",bev);
                callane.inverseBirdEyeView(bev, inv_bev);
         //     imshow("my_inv_bev",inv_bev);
                cv::Mat newlane = frame.clone();
                callane.inverseBirdEyeView(laneColor, newlane);
            //    cv::imshow("newlane", newlane);
               
                pub_img = newlane.clone();
                imshow("d",pub_img);
                int coordi_count = 0;
                coordi_array.data.clear();
                coordi_array.data.push_back(10);   
                for (int y = pub_img.rows-1; y >=0; y--) {
                    uchar* inv_bev_data = pub_img.ptr<uchar>(y);
                        for (int x = 0; x < pub_img.cols; x++) {
                            if (inv_bev_data[x] != (uchar)0) {
                                coordi_count++;
                                coordi_array.data.push_back(x);
                                coordi_array.data.push_back(y);
                            }
                        }
                }
                cv::Mat inv_lane = laneColor.clone();
                callane.inverseBirdEyeView(laneColor, inv_lane);
           //     cv::imshow("inv_bev", inv_bev);
           //     cv::imshow("laneColor", laneColor);
                if (TRACK_BAR) cv::imshow("TARCKBAR", yellow);
                float degree=0., ladian = 0.;
                int no_data;
                // if(check<10){
                //     callane.storeMatVec(canny_img, lane_m_vec);
                //     ++check;
                // }
                // else{//check>=10
                //    // callane.cannyToBImg(canny_img);
                //     canny_img = callane.addMatUsingOR(lane_m_vec);    
                //     no_data = callane.cannyToNSample(canny_img);//canny바꾸지 말고 클론만들어서하고, 침식연산 추가
                    
                //     check = 0;
                //     lane_m_vec.clear();
                //     }
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
          //  cv::imshow(OPENCV_WINDOW_VF,frame);
            
            int ckey = cv::waitKey(10);
            if(ckey == 27) exit(1);    
             
        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "lane_detection");
    ros::NodeHandle nh_;
    InitImgObjectforROS img_obj;
  //  ros::Publisher pub = img_obj.nh.advertise<std_msgs::Int32MultiArray>("/cam0/lane",100);
    
     
//    std::cout<<coordi_array.data[0];
    ros::Rate loop_rate(5);
 //   coordi_array.data.push_back(10);
 
   // cv::imshow("mm",img_obj.pub_img);
while(img_obj.nh.ok()){
        img_obj.pub.publish(img_obj.coordi_array);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("program killed!\n");
    return 0;
}




// // //CalLane::sPoint *data = new CalLane::sPoint[no_data];
                    // // // callane.BImgtoRANSACSample(canny_img,data);
                    // // // cost = callane.ransacLineFitting(data, no_data, model, distance_threshold);
                    // // // ran_y1 = ((model.my/model.mx)*(0. - model.sx)) + model.sy;//x == 0일 때 
                    // // // ran_y2 = ((model.my/model.mx)*(240. - model.sx)) + model.sy;//x== 240일때
                    // // // std::cout<<"cost : "<<cost<<std::endl;
                    // // // if(cost>10.){
                    // // //     ladian = atan2f(ran_y2-ran_y1, 240.);
                    // // //     degree = ladian*180/CV_PI;
                    // // //     cv::circle(frame, cv::Point((*data).x, (*data).y), 2, cv::Scalar(0, 0, 255), -1);
                    // // //    // if(degree > 20){
                    // // //       //  cv::line(frame, cv::Point(0,ran_y1), cv::Point(240,ran_y2),
                    // // //     //                    cv::Scalar(255,0, 0), 3, 0);
                                          
                    // // //    // }
                    // // // }
                    
                    // // // delete[] data; 
