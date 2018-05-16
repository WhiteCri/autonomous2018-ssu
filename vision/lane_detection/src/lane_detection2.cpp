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
#include <string>

#define HSV 0
#define RGB 1

using namespace std;
using namespace cv;

static int debug;
static int track_bar;
static int lane;
static int video;
static int w_hmin, w_hmax, w_smin, w_smax, w_vmin, w_vmax;
static int y_hmin, y_hmax, y_smin, y_smax, y_vmin, y_vmax;

static string groupName;

class Lane_detection {
public:
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub_img;
	std_msgs::Int32MultiArray coordi_array;
	std::vector<int> lane_width_array;
	cv::Mat pub_img;
	ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("/"+groupName+"/lane",100);

	Lane_detection() : it(nh){

		initParam();

		// if(video == 1){sub_img = it.subscribe("//videofile/image_raw",1,&Lane_detection::laneCb,this); }
		// else if (video == 0){ sub_img = it.subscribe("/"+groupName+"/raw_image",1,&Lane_detection::laneCb,this); }
		sub_img = it.subscribe("/"+groupName+"/raw_image",1,&Lane_detection::laneCb,this);

		if (track_bar) {
			yTrackName = groupName + "/YELLOW";
			wTrackName = groupName + "/WHITE";
			initTrackbar();
		}
	};


	Mat wTrackBar();
	Mat yTrackBar();
	void initParam();
	void applyTrackbar(const Mat& src, Mat& dst, int flag, double minH, double maxH, double minS, double maxS, double minV, double maxV);
	void initTrackbar();

	void laneCb(const sensor_msgs::ImageConstPtr& img_msg);
	void createData();

	Mat src;
	Mat yellow;
	Mat white;
	Mat yErode, wErode;
	Mat yFilter;
	Mat wFilter;
	Mat yEdge, wEdge;
	Mat yInterval;
	Mat wInterval;
	Mat wLine;
	Mat output;

	string wTrackName;
	string yTrackName;
	VideoCapture cap;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "lane_detection");
	groupName = argv[1];
	Lane_detection lane_detect;
	ros::Rate loop_rate(30);

	while(lane_detect.nh.ok()){
		lane_detect.pub.publish(lane_detect.coordi_array);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


void Lane_detection::laneCb(const sensor_msgs::ImageConstPtr& img_msg) {

	cv_bridge::CvImagePtr cv_ptr;

	cv_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
	src = cv_ptr->image;
	
	//src = imread("/home/whiteherb/catkin_ws/src/autonomous2018-ssu/vision/cal_distance/data/left_calibration.png", CV_LOAD_IMAGE_COLOR);
	
	Rect rc(0,0,640,150);
	Scalar color(0,0,0);
	rectangle(src, rc, color, CV_FILLED);	

	if (track_bar == 1) {
		applyTrackbar(src, yellow, HSV, y_hmin, y_hmax, y_smin, y_smax, y_vmin, y_vmax);
		applyTrackbar(src, white, HSV, w_hmin, w_hmax, w_smin, w_smax, w_vmin, w_vmax);
	}

	GaussianBlur(yellow, yellow, Size(5, 5), 0.5);
	GaussianBlur(white, white, Size(5, 5), 0.5);

	medianBlur(yellow, yFilter, 5);
	medianBlur(white, wFilter, 5);

	Canny(yFilter, yEdge, 125, 350);
	Canny(wFilter, wEdge, 125, 350);

	output = yEdge | wEdge;

	if(debug){
		imshow(groupName + "/src", src);
		imshow(yTrackName, yellow);
		imshow(wTrackName, white);
		imshow(groupName + "/Yellow_Filter", yFilter);
		imshow(groupName + "/White_Filter", wFilter);
		// imshow(groupName + "/Yellow_Edge", yEdge);
		// imshow(groupName + "/White_Edge", wEdge);
	}
	if(lane){
			imshow(groupName + "/output", output);
	}


	createData();

	int key = waitKey(30);
	if (key == 27) exit(1);
}

void Lane_detection::applyTrackbar(const Mat& src, Mat& dst, int flag, double minH, double maxH, double minS, double maxS, double minV, double maxV) {
	Mat hsv;

	if(flag == HSV) cvtColor(src, hsv, COLOR_BGR2HSV);
	else hsv = src;

	vector<Mat> channels;
	split(hsv, channels);

	Mat mask1;
	Mat mask2;

	threshold(channels[0], mask1, maxH, 255, THRESH_BINARY_INV);
	threshold(channels[0], mask2, minH, 255, THRESH_BINARY);
	Mat hueMask;
	if (minH < maxH) hueMask = mask1 & mask2;
	else hueMask = mask1 | mask2;

	threshold(channels[1], mask1, maxS, 255, THRESH_BINARY_INV);
	threshold(channels[1], mask2, minS, 255, THRESH_BINARY);

	Mat satMask;
	satMask = mask1 & mask2;

	threshold(channels[2], mask1, maxV, 255, THRESH_BINARY_INV);
	threshold(channels[2], mask2, minV, 255, THRESH_BINARY);

	Mat valMask;
	valMask = mask1 & mask2;

	dst = hueMask & satMask & valMask;
}

void Lane_detection::initTrackbar() {
	namedWindow(yTrackName, WINDOW_AUTOSIZE);
	namedWindow(wTrackName, WINDOW_AUTOSIZE);


	cv::createTrackbar("h min", wTrackName, &w_hmin, 255, NULL);
	cv::setTrackbarPos("h min", wTrackName, w_hmin);

	cv::createTrackbar("h max", wTrackName, &w_hmax, 255, NULL);
	cv::setTrackbarPos("h max", wTrackName, w_hmax);

	cv::createTrackbar("s min", wTrackName, &w_smin, 255, NULL);
	cv::setTrackbarPos("s min", wTrackName, w_smin);

	cv::createTrackbar("s max", wTrackName, &w_smax, 255, NULL);
	cv::setTrackbarPos("s max", wTrackName, w_smax);

	cv::createTrackbar("v min", wTrackName, &w_vmin, 255, NULL);
	cv::setTrackbarPos("v min", wTrackName, w_vmin);

	cv::createTrackbar("v max", wTrackName, &w_vmax, 255, NULL);
	cv::setTrackbarPos("v max", wTrackName, w_vmax);

	cv::createTrackbar("h min", yTrackName, &y_hmin, 255, NULL);
	cv::setTrackbarPos("h min", yTrackName, y_hmin);

	cv::createTrackbar("h max", yTrackName, &y_hmax, 255, NULL);
	cv::setTrackbarPos("h max", yTrackName, y_hmax);

	cv::createTrackbar("s min", yTrackName, &y_smin, 255, NULL);
	cv::setTrackbarPos("s min", yTrackName, y_smin);

	cv::createTrackbar("s max", yTrackName, &y_smax, 255, NULL);
	cv::setTrackbarPos("s max", yTrackName, y_smax);

	cv::createTrackbar("v min", yTrackName, &y_vmin, 255, NULL);
	cv::setTrackbarPos("v min", yTrackName, y_vmin);

	cv::createTrackbar("v max", yTrackName, &y_vmax, 255, NULL);
	cv::setTrackbarPos("v max", yTrackName, y_vmax);
}

void Lane_detection::createData(){
	coordi_array.data.clear();
	coordi_array.data.push_back(10);
	int count = 0;


	for(int y = output.rows-1; y>=0; y--){
		for(int x = 0; x < output.cols; x++){
			count++;
			if(output.at<uchar>(x,y) > 0){
				count++;
				coordi_array.data.push_back(y);
				coordi_array.data.push_back(x);
			}
		}
	}

	coordi_array.data[0] = count;

}


void Lane_detection::initParam() {
	nh.param<int>("/"+groupName+"/lane_detection2/debug", debug, 1);
	nh.param<int>("/"+groupName+"/lane_detection2/track_bar", track_bar, 1);
	nh.param<int>("/"+groupName+"/lane_detection2/track_bar", lane, 1);
	nh.param<int>("/"+groupName+"/lane_detection2/debug", video, 0);
	nh.param<int>("/"+groupName+"/lane_detection2/y_hmin",y_hmin,12);
	nh.param<int>("/"+groupName+"/lane_detection2/y_hmax",y_hmax,24);
	nh.param<int>("/"+groupName+"/lane_detection2/y_smin",y_smin,60);
	nh.param<int>("/"+groupName+"/lane_detection2/y_smax",y_smax,146);
	nh.param<int>("/"+groupName+"/lane_detection2/y_vmin",y_vmin,6);
	nh.param<int>("/"+groupName+"/lane_detection2/y_vmax",y_vmax,255);
	nh.param<int>("/"+groupName+"/lane_detection2/w_hmin",w_hmin,33);
	nh.param<int>("/"+groupName+"/lane_detection2/w_hmax",w_hmax,125);
	nh.param<int>("/"+groupName+"/lane_detection2/w_smin",w_smin,0);
	nh.param<int>("/"+groupName+"/lane_detection2/w_smax",w_smax,15);
	nh.param<int>("/"+groupName+"/lane_detection2/w_vmin",w_vmin,117);
	nh.param<int>("/"+groupName+"/lane_detection2/w_vmax",w_vmax,255);

	ROS_INFO("%d %d", w_hmin, w_vmin);
}
