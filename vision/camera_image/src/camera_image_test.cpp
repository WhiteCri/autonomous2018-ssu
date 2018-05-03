#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"


// Complile : g++ camera_image_test.cpp -o camera_image_test `pkg-config --cflags --libs opencv` -std=c++11

using namespace std;
using namespace cv;

int main(int argc, char** argv){

    int camera_number = 1;
    int frequency = 30;

    VideoCapture cap(0);

    int key;
    int num = 1;
    Mat src;
    Mat src2;
    Mat calib1;
    Mat calib2;

    Mat cameraMatrix= Mat::eye(3, 3, CV_64FC1);
    Mat distCoeffs = Mat::zeros(1, 5, CV_64FC1);

    cameraMatrix=(Mat1d(3, 3) << 708.611809, 0, 320.331083, 0, 703.012319, 260.343059, 0, 0, 1);

    distCoeffs=(Mat1d(1, 5) << 0.09776299999999999, -0.235306, 0.00463, -0.001884, 0);



    while(1){
        cap >> src;
        imshow("origin",src);
        // cout<<"src x : "<<src.cols << " / src y : "<<src.rows<<endl;

        resize(src, src2, Size(src.cols * 2, src.rows * 2), 0, 0, CV_INTER_NN);
        //imshow("Size up",src2);
        // cout<<"src2 x : "<<src2.cols << " / src_up y : "<<src2.rows<<endl;

        //undistort(src, calib1, cameraMatrix, distCoeffs);

        //undistort(src2, calib2, cameraMatrix, distCoeffs);
        //resize(calib1, calib2, Size(calib1.cols * 2, calib1.rows * 2), 0, 0, CV_INTER_NN);

        //imshow("Calibration 1",calib1);

        //imshow("Calibration 2",calib2);

        key = waitKey(30);
        if(key == 27) {
            break;
        }
        else if (key == 32) {
        
             cout<<"Image write "<<to_string(num)<<endl;
             imwrite("../sample_image/src_"+to_string(num)+".jpg", src);
         //    imwrite("../sample_image/src2_"+to_string(num)+".jpg", src2);
        //     imwrite("../sample_image/calib1_"+to_string(num)+".jpg", calib1);
        //     imwrite("../sample_image/calib2_"+to_string(num)+".jpg", calib2);
        
        
             num++;
        }
    }

    return 0;
}
