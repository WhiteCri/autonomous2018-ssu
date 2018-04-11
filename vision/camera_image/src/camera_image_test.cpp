#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"


// Complile : g++ camera_image_test.cpp -o camera_image_test `pkg-config --cflags --libs opencv` -std=c++11

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    
    int camera_number = 1;
    int frequency = 30;

    VideoCapture cap(1);
    
    int key;
    int num = 1;
    Mat src;
    Mat src2;
    Mat calib1;
    Mat calib2;

    Mat cameraMatrix= Mat::eye(3, 3, CV_64FC1);
    Mat distCoeffs = Mat::zeros(1, 5, CV_64FC1);

    cameraMatrix=(Mat1d(3, 3) <<  8.6284313724313131e+02, 0., 640., 0., 8.6284313724313131e+02, 360., 0., 0., 1.);
    distCoeffs=(Mat1d(1, 5) << -4.2037029317618180e-01, 2.6770730624131461e-01, 0., 0.,-1.2516364113555581e-01);



    while(1){
        cap >> src;
        imshow("origin",src);
        // cout<<"src x : "<<src.cols << " / src y : "<<src.rows<<endl;

        resize(src, src2, Size(src.cols * 2, src.rows * 2), 0, 0, CV_INTER_NN);
        imshow("Size up",src2);
        // cout<<"src2 x : "<<src2.cols << " / src_up y : "<<src2.rows<<endl;

        undistort(src, calib1, cameraMatrix, distCoeffs);
        undistort(src2, calib2, cameraMatrix, distCoeffs);

        imshow("Calibration 1",calib1);

        imshow("Calibration 2",calib2);

        key = waitKey(30);
        if(key == 27) {
            break;
        }
        else if (key == 32) {
            
            cout<<"Image write "<<to_string(num)<<endl;
            imwrite("../sample_image/src_"+to_string(num)+".jpg", src);
            imwrite("../sample_image/src2_"+to_string(num)+".jpg", src2);
            imwrite("../sample_image/calib1_"+to_string(num)+".jpg", calib1);
            imwrite("../sample_image/calib2_"+to_string(num)+".jpg", calib2);
            
            num++;

        }
    }    

    return 0;
}

