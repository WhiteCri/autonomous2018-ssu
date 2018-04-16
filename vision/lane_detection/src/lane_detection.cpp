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
#include <time.h>
//15 21 52 151 000 180
//0 180 0 24 172 255


//#include <lanedetection/coordinat0eMsg.h>
//#include <lanedetection/vecMsg.h>
//#include <lanedetection/dataSize.h>
#define COORDI_COUNT 4000
#define CLOCK_PER_SEC 1000
static const std::string OPENCV_WINDOW_VF = "Image by videofile";
static const std::string OPENCV_WINDOW_WC = "Image by webcam";
static const bool DEBUG_SW = true;
static const bool WEB_CAM = true;
static const bool IMSHOW_SW = false;
static const bool TRACK_BAR = true;
static const bool TIME_CHECK = false;
static const bool USE_LABEL_COUNT_FOR_CROSSWALK = false;

lane_detect_algo::vec_mat_t lane_m_vec;

 bool is_stop_checked = false;
using namespace lane_detect_algo;

class InitImgObjectforROS{
    public:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub_img;
        std_msgs::Int32MultiArray coordi_array;
        cv::Mat pub_img;
        ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("/cam1/lane",100);//파라미터로 카메라 번호 받도록하기
      
        int y_hmin, y_hmax, y_smin, y_smax, y_vmin, y_vmax;
        int w_hmin, w_hmax, w_smin, w_smax, w_vmin, w_vmax;
        int check_stop_count;
       

        InitImgObjectforROS();
        ~InitImgObjectforROS();
        void imgCb(const sensor_msgs::ImageConstPtr& img_msg);
};



InitImgObjectforROS::InitImgObjectforROS():it(nh){
            if(!WEB_CAM){//'DEBUG_SW == TURE' means subscribing videofile image
                sub_img = it.subscribe("/videofile/image_raw",1,&InitImgObjectforROS::imgCb,this);
            }
            else{//'DEBUG_SW == FALSE' means subscribing webcam image
                sub_img = it.subscribe("/cam1/raw_image",1,&InitImgObjectforROS::imgCb,this);
            }
            
            if(TRACK_BAR) {
                    
                    cv::namedWindow("TRACKBAR_YELLOW", cv::WINDOW_AUTOSIZE);
                    cv::namedWindow("TRACKBAR_WHITE" ,cv::WINDOW_AUTOSIZE);
           
                    cv::createTrackbar("h min", "TRACKBAR_YELLOW", &y_hmin, 50, NULL);
                    cv::setTrackbarPos("h min", "TRACKBAR_YELLOW", 15);

                    cv::createTrackbar("h max", "TRACKBAR_YELLOW", &y_hmax, 50, NULL);
                    cv::setTrackbarPos("h max", "TRACKBAR_YELLOW", 21);

                    cv::createTrackbar("s min", "TRACKBAR_YELLOW", &y_smin, 255, NULL);
                    cv::setTrackbarPos("s min", "TRACKBAR_YELLOW", 52);

                    cv::createTrackbar("s max", "TRACKBAR_YELLOW", &y_smax, 255, NULL);
                    cv::setTrackbarPos("s max", "TRACKBAR_YELLOW", 151);

                    cv::createTrackbar("v min", "TRACKBAR_YELLOW", &y_vmin, 255, NULL);
                    cv::setTrackbarPos("v min", "TRACKBAR_YELLOW", 0);

                    cv::createTrackbar("v max", "TRACKBAR_YELLOW", &y_vmax, 255, NULL);
                    cv::setTrackbarPos("v max", "TRACKBAR_YELLOW", 180);

                    
                    cv::createTrackbar("h min", "TRACKBAR_WHITE", &w_hmin, 180, NULL);
                    cv::setTrackbarPos("h min", "TRACKBAR_WHITE", 0);

                    cv::createTrackbar("h max", "TRACKBAR_WHITE", &w_hmax, 180, NULL);
                    cv::setTrackbarPos("h max", "TRACKBAR_WHITE", 180);

                    cv::createTrackbar("s min", "TRACKBAR_WHITE", &w_smin, 255, NULL);
                    cv::setTrackbarPos("s min", "TRACKBAR_WHITE", 0);

                    cv::createTrackbar("s max", "TRACKBAR_WHITE", &w_smax, 255, NULL);
                    cv::setTrackbarPos("s max", "TRACKBAR_WHITE", 24);

                    cv::createTrackbar("v min", "TRACKBAR_WHITE", &w_vmin, 255, NULL);
                    cv::setTrackbarPos("v min", "TRACKBAR_WHITE", 172);

                    cv::createTrackbar("v max", "TRACKBAR_WHITE", &w_vmax, 255, NULL);
                    cv::setTrackbarPos("v max", "TRACKBAR_WHITE", 255);
                    }
                check_stop_count = 0;
                //is_stop_checked = false;
                
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
            cv::Mat frame, canny_img, gray, yellow_hsv, white_hsv, 
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
                
                if (TRACK_BAR) {
                    y_hmin = cv::getTrackbarPos("h min", "TRACKBAR_YELLOW");
                    y_hmax = cv::getTrackbarPos("h max", "TRACKBAR_YELLOW");
                    y_smin = cv::getTrackbarPos("s min", "TRACKBAR_YELLOW");
                    y_smax = cv::getTrackbarPos("s max", "TRACKBAR_YELLOW");
                    y_vmin = cv::getTrackbarPos("v min", "TRACKBAR_YELLOW");
                    y_vmax = cv::getTrackbarPos("v max", "TRACKBAR_YELLOW");

                    w_hmin = cv::getTrackbarPos("h min", "TRACKBAR_WHITE");
                    w_hmax = cv::getTrackbarPos("h max", "TRACKBAR_WHITE");
                    w_smin = cv::getTrackbarPos("s min", "TRACKBAR_WHITE");
                    w_smax = cv::getTrackbarPos("s max", "TRACKBAR_WHITE");
                    w_vmin = cv::getTrackbarPos("v min", "TRACKBAR_WHITE");
                    w_vmax = cv::getTrackbarPos("v max", "TRACKBAR_WHITE");
                }
                lane_detect_algo::CalLane callane;
                cv::Mat bev =frame.clone();
                callane.birdEyeView(frame,bev);

                
                if (TRACK_BAR) {
                    callane.detectYHSVcolor(bev, yellow_hsv, y_hmin, y_hmax, y_smin, y_smax, y_vmin, y_vmax);
                    callane.detectWhiteLane(bev,white_hsv, w_hmin, w_hmax, w_smin, w_smax, w_vmin, w_vmax,0,0);
                    cv::imshow("TRACKBAR_WHITE",white_hsv);
                    cv::imshow("TRACKBAR_YELLOW",yellow_hsv);
                }
                else {// 색상검출 디폴트
                    callane.detectYHSVcolor(bev, yellow_hsv, 7, 21, 52, 151, 0, 180);
                    callane.detectWhiteLane(bev, white_hsv, 0, 180, 0, 29, 179, 255,0,0);
                }
                
 
                cv::Mat yellowYProj(cv::Size(frame_width, frame_height), CV_8UC1);
                cv::Mat whiteYProj(cv::Size(frame_width, frame_height), CV_8UC1);
                cv::Mat yellowXProj(cv::Size(frame_width, frame_height), CV_8UC1);
                cv::Mat whiteXProj(cv::Size(frame_width, frame_height), CV_8UC1);
               
                if(USE_LABEL_COUNT_FOR_CROSSWALK){
                int crosswalk_check = 0;
                int *crosswalk = &crosswalk_check;
                callane.makeContoursRightLane(white_hsv, white_labeling, crosswalk);
                nh.setParam("hl_controller/crosswalk",false);
                if(crosswalk_check){
                        ROS_INFO("param set crosswalk true!\n");
                        nh.setParam("hl_controller/crosswalk",true);                       
                        crosswalk_check = 0;
                    }
                }
                callane.makeContoursLeftLane(yellow_hsv, yellow_labeling);//for labeling(source channel is 1)
                if(!USE_LABEL_COUNT_FOR_CROSSWALK){
                    callane.makeContoursRightLane(white_hsv, white_labeling);//for labeling(source channel is 1)
                }

               
                if(TIME_CHECK){
                    const int64 start = cv::getTickCount();
                    int64 elapsed = (cv::getTickCount() - start);
                    std::cout << "Elapsed time " << elapsed << std::endl;
                }
                laneColor = yellow_labeling | white_labeling;
               //  callane.makeXProjection(yellow_hsv,yellowXProj,H_xResultYellow);

                
                cv::Mat inv_bev = frame.clone();//color inverse bev //do not delete
                callane.inverseBirdEyeView(bev, inv_bev);
                cv::Mat newlane = frame.clone();
                callane.inverseBirdEyeView(laneColor, newlane);

                if(IMSHOW_SW){
                    cv::imshow("inverseBirdEye", newlane);
                }

                cv::Mat output_origin = origin.clone();
                pub_img = newlane.clone();
                int coordi_count = 0;
                coordi_array.data.clear();
                coordi_array.data.push_back(10);

                for(int y = output_origin.rows-1; y>=210; y--){
                    uchar* origin_data = output_origin.ptr<uchar>(y);
                    uchar* pub_img_data = pub_img.ptr<uchar>(y*0.5);//resize 복구(0.5 -> 1))
                        for(int x = 0; x<output_origin.cols; x++){
                            int temp = x*0.5;//resize 복구(0.5 -> 1)
                            if(pub_img_data[temp]!= (uchar)0){
                                coordi_count++;
                                coordi_array.data.push_back(x);
                                coordi_array.data.push_back(y);
                                origin_data[x*output_origin.channels()] = 255;
                        }
                    }
                } 
                std::vector<cv::Vec4i> lines;
                std::vector<cv::Vec4i>::iterator it;

                cv::Mat my_test_hough = inv_bev.clone();//bev되돌리기 전 이미지 가지고 디텍팅하면 복구할떄 복잡하니까 되돌린후 허프적용

                cv::HoughLinesP(newlane, lines, 1, CV_PI / 180.0, 80, 80, 5);
                if(!is_stop_checked){
                   // nh.setParam("hl_controller/crosswalk",false);
                }
                if(!lines.empty()){
                    float ladian;
                    int degree;
                    it = lines.end() - 1;
                    ladian = atan2f((*it)[3] - (*it)[1], (*it)[2] - (*it)[0]);
                    degree = ladian * 180 / CV_PI; 
                    if(abs(degree)>=0 && abs(degree)<=5){
                    
                    cv::line(my_test_hough, cv::Point((*it)[0], (*it)[1]),
                                            cv::Point((*it)[2] , (*it)[3] ),
                                            cv::Scalar(255,0, 0 ), 3, 0);
                    
                    if(DEBUG_SW){
                        ROS_INFO("-----------STOP!-----------\n");
                        ROS_INFO("check_stop_count : %d",check_stop_count);
                    }
                    
                    check_stop_count++;
                    
                   // if(check_stop_count>0){
                        //is_stop_checked++;
                        nh.setParam("hl_controller/crosswalk",true);
                        check_stop_count = 1;
                        is_stop_checked = true;
                        ROS_INFO("param set crosswalk true!\n");
                        imshow("stop_lane_before_crosswalk",my_test_hough);
                   // }                       
                    if(is_stop_checked){
		     //is_stop_checked = false;
		    } 
                    if(DEBUG_SW){
                        std::cout<<"coordi_count : "<<coordi_count<<std::endl;    
                        }
                    }
                    
                }
                coordi_array.data[0] = coordi_count;

                if(!IMSHOW_SW){
                    imshow("colorfulLane",output_origin);
                }
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

            int ckey = cv::waitKey(10);
            if(ckey == 27) exit(1); 
}


int main(int argc, char **argv){
    ros::init(argc, argv, "lane_detection");
    ros::NodeHandle nh_;
    InitImgObjectforROS img_obj;
    ros::Rate loop_rate(30);
    
    while(img_obj.nh.ok()){
        img_obj.pub.publish(img_obj.coordi_array);
        ros::spinOnce();
        loop_rate.sleep();

    }
    ROS_INFO("program killed!\n");
    return 0;
}



