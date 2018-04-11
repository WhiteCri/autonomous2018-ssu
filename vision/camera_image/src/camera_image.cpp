#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

// argv[0] : camera_number, argv[1] : frequency


static const bool DEBUG = true; // 디버깅 스위치
static const bool CALIBRATION = false;
static const bool SIZEUP = false;
static const std::string OPENCV_WINDOW = "Raw Image Window";

template < typename T > std::string to_string( const T& n );

class CameraImage{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher camera_image_pub_;

public:
    CameraImage(int par_camera_num,int par_frequency, cv::Mat camMat, cv::Mat distMat)
        : it_(nh_), camera_number(par_camera_num), frequency(par_frequency), cameraMatrix(camMat), distCoeffs(distMat)
    { 
        createTopicName();  
        if(DEBUG) std::cout<<"topic_name : "<<topic_name<<std::endl;
        camera_image_pub_ = it_.advertise(topic_name,1);
        cap.open(camera_number);
        
        if(CALIBRATION){
            cameraMatrix=(cv::Mat1d(3, 3) << 625.546173, 0, 329.374745, 0, 625.809222, 221.233758, 0, 0, 1);
            distCoeffs=(cv::Mat1d(1, 5) << 0.103856, -0.124799, -0.001845, 0.012219, 0);
        }
        
    }

    ~CameraImage()
    {
        if(DEBUG) cv::destroyWindow(OPENCV_WINDOW);
    }

    void sendImage(); // image 퍼블리시
    std::string createTopicName();// topic이름 생성

private:
    int camera_number;
    int frequency;    
    cv::VideoCapture cap;
    std::string topic_name;
    sensor_msgs::ImagePtr msg; 
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;  
    
};

int main(int argc, char** argv){
    
    if(DEBUG)
    {   
        ROS_INFO("DEBUG MODE ACTIVATED!");
        ROS_INFO("argc : %d",argc);
        for(int i=0; i<argc; i++){
            ROS_INFO("argv[%d] : %s",i,argv[i]);
        }
    }

    ros::init(argc, argv, "camera_image");

    int camera_number = atoi(argv[1]);
    int frequency = atoi(argv[2]);

    ROS_INFO("camera_number : %d / frequency : %d",camera_number, frequency);
    if(!frequency){
        ROS_INFO("Frequency is number more than 0");
        return 1;
    }

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);

    CameraImage cimage(camera_number, frequency, cameraMatrix, distCoeffs);

    if(DEBUG) ROS_INFO("Start publishing");
    
    cimage.sendImage();
   
    if(DEBUG) ROS_INFO("Publishing done");

    return 0;
}





// 구현부

void CameraImage::sendImage(){
    ros::Rate loop_rate(frequency);
        cv::Mat frame;
        cv::Mat temp;
    while(nh_.ok()){
        cap >> frame;
        if(!frame.empty()){

            if(SIZEUP) cv::resize(frame, frame, cv::Size(frame.cols * 2, frame.rows * 2), 0, 0, CV_INTER_NN);
            if(CALIBRATION) {
                cv::undistort(frame, temp, cameraMatrix, distCoeffs);
                frame = temp;
            }
            if(DEBUG) cv::imshow(OPENCV_WINDOW,frame);
            
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            camera_image_pub_.publish(msg);             
        }                        
        int ckey = cv::waitKey(1);
        if(ckey == 27)break;
        loop_rate.sleep();
    }
}

std::string CameraImage::createTopicName(){
    topic_name =  "/cam"+ to_string(1) +"/raw_image";
}

template < typename T > 
std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}
