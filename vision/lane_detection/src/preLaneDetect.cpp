#include <preprocessing/preLaneDetect.hpp>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace std;//다하고 지워
using namespace cv;//다하고 지워
static const bool DEBUG_SW = true;

//http://blog.daum.net/pg365/62 --> RANSAC algorithm 참고.
namespace lane_detect_algo{
    
            
            void CalLane::printVec4i(vec4i_t print_vec_4i){
                for(auto &vec_4i : print_vec_4i){
                    if(DEBUG_SW) {//print vec_4i info when DEBUG_SW is true.
                        std::cout<<vec_4i<<std::endl;
                    }
                }   
            }
            void CalLane::printVec2i(vec2i_t print_vec_2i){
                for(auto &vec_2i : print_vec_2i){
                    if(DEBUG_SW) {//print vec_2i info when DEBUG_SW is true.
                        std::cout<<vec_2i<<std::endl;
                    }
                }
            }
            void CalLane::printMat(cv::Mat &print_mat){
                if(DEBUG_SW){//print mat info when DEBUG_SW is true.
                    std::cout<<print_mat<<std::endl;
                }
            }

            void CalLane::sortLaneVec(vec_p_t &lane_p_vec){
                auto temp_y = lane_p_vec.front().y;
                for(uint i =0; i< lane_p_vec.size(); ++i){//데이터 정렬되어있을 가능성이 높으므로 삽입정렬
                    for(uint j=i+1; j<lane_p_vec.size(); ++j){
                        if(lane_p_vec[j].y > lane_p_vec[i].y){
                            temp_y = lane_p_vec[i].y;
                            lane_p_vec[i].y = lane_p_vec[j].y;
                            lane_p_vec[j].y = temp_y;
                        }
                    }
                }
                
            }

            void CalLane::storePointVec(cv::Point &lane_point, vec_p_t &lane_p_vec){
                
     //**********//   vec2i_t a;
     //**********//   std::vector<vec2i_t> tt;
     //**********//   tt.push_back(a);//point말고 벡터를 넣은 벡터..이렇게하는거가 더 좋을지 생각좀...
                try{
                    lane_p_vec.push_back(lane_point);
                }
                catch(cv::Exception e){
                    ROS_INFO("%s\n storeLaneVec is fail!\n",e.what());
                }
               
            }

            void CalLane::storeMatVec(cv::Mat lane_mat, vec_mat_t &lane_m_vec){
                try{
                    lane_m_vec.push_back(lane_mat);
                }
                catch(cv::Exception e){
                    ROS_INFO("%s\n storeMatVec is fail!\n",e.what());
                }
            }

            void CalLane::scoreBoard(cv::Mat &b_score_board, cv::Mat b_img){
                if(b_score_board.size() == b_img.size()){
                    
                }
                else{//b_sroce_bore.size() != b_img
                    std::cout<<"the score_board and the b_img do not match the size!"<<std::endl;
                }
                
            }

            cv::Mat CalLane::addMatUsingIt(vec_mat_t add_mat){
                cv::Mat store_mat = add_mat[0];
                cv::Mat_<uchar>::iterator it_m = store_mat.begin<uchar>();
                cv::Mat_<uchar>::iterator it_s = store_mat.begin<uchar>();//중요..까먹지좀말자
                for(uint i = 0; i<add_mat.size(); ++i){
                    it_m = add_mat[i].begin<uchar>();
                    int size = store_mat.rows*store_mat.cols;
                    for(int j = 0; j<size; ++j,++it_m,++it_s){
                        if(*it_m != *it_s && *it_s==0){
                            *it_s = 255;
                        }
                    }
                    it_s = store_mat.begin<uchar>();
                }

                
                return store_mat;
            }
            cv::Mat CalLane::addMatUsingOR(vec_mat_t add_mat){
                cv::Mat store_mat = add_mat[0];
               // int a= 0|255;
               // std::cout<<"Sss: "<<a<<std::endl;
                for(uint i = 0; i<add_mat.size(); ++i){
                    store_mat = store_mat | add_mat[i];
                }
                //std::cout<<store_mat<<std::endl;
                return store_mat;
            }

            cv::Mat CalLane::addMatVec(vec_mat_t add_mat_vec){
                cv::Mat add_mat;
                return add_mat;
            }
            void CalLane::cannyToBImg(cv::Mat& canny_img){
                //ineger형 1은 unsigned char 1로 들어가..
                cv::Mat_<uchar>::iterator it = canny_img.begin<uchar>();
               //**영상처리에서 실수처리는 float, 정수 처리는 unsigned char로 한다!!**//
                if(DEBUG_SW){
                    //std::cout<<"channels() : "<<canny_img.channels()<<std::endl;
                }
                if(canny_img.channels()==1){
                   int i = 1;
                   for(;it!=canny_img.end<uchar>(); ++it){
                        if(DEBUG_SW){
                     //       std::cout<<" "<<(int)*it<<"   ";
                        }
                        if(i%10 == 0) std::cout<<std::endl;
                        if(((int)*it) > 0){
                            *it = 1;
                      //      cout<<"dfds"<<*it<<std::endl;
                        }
                        i++;
                    }
                }else{//canny_img.channels() != 1
                    std::cout<<"please input grayImg for cannyToBImg"<<std::endl;
                }
            }
            int CalLane::cannyToNSample(cv::Mat canny_img){
                
                cv::Mat_<uchar>::iterator it = canny_img.begin<uchar>();
               //**영상처리에서 실수처리는 float, 정수 처리는 unsigned char로 한다!!**//
                int no_sample = 0;
                if(canny_img.channels()==1){
                   for(;it!=canny_img.end<uchar>(); ++it){
                        if((int)*it > 0){
                            ++no_sample;
//                            *it = 1;
                            //std::cout<<"no_sample : "<<(int)*it;
                        }
                    }
                }else{//canny_img.channels() != 1
                    std::cout<<"please input grayImg for cannyToBImg"<<std::endl;
                }
                return no_sample;
            }
                void CalLane::BImgtoRANSACSample(cv::Mat canny_img, sPoint samples[]){
               // CalLane::sPoint *samples = new CalLane::sPoint[no_sample];
                auto it = canny_img.begin<uchar>();
                //auto itend = canny_img.end<uchar>();
                int i = 0,j=0;
                int sample_index = 0;
              //  std::cout<<canny_img;
                int it_to_int;
                //  for(int i = 0; i < canny_img.cols*canny_img.rows; ++i)
                //  {
                //  unsigned char* ptr = (unsigned char*)canny_img.data;
                //  std::cout<<" "<<(int)ptr[i]<<" ";
                //  }
              //  std::cout<<" \n"<<canny_img<<std::endl<<"__";
                for(; i<canny_img.rows; ++i){
                    for(; j<canny_img.cols; ++j){
                   //     std::cout<<"it : "<<(uchar)*it;
                        it_to_int =(int)*it;
                      //  std::cout<<(int)*it<<"_";
                        //std::cout<<it_to_int<<std::endl;
                        //std::cout<<" "<<it_to_int;
                        if(it_to_int == 255){//for canny img
                            samples[sample_index].x = i;
                            samples[sample_index].y = j;
                           // std::cout<<"x : "<<samples[sample_index].x<<", y : "<<samples[sample_index].y<<std::endl;
                            ++sample_index;
                            }++it;
                        
                        }
                    }
                    //int a=1;
                 //   return *samples;//이렇게 리턴하는 거시.. 맞는가?..
            }
            cv::Mat CalLane::outputRANSAC(cv::Mat score_board){
                return score_board;
            }
            void CalLane::outputPCA(cv::Mat lane_point_mat, CalLane::sLine &model){
                int i=0, j=0;
                double sum_Xi=0, sum_Yi=0, sum_Xi_expo=0, sum_Yi_expo=0, sum_XixYi=0, N=0;
                double cov_xx, cov_yy, cov_xy;
                auto it = lane_point_mat.begin<uchar>();
             //   auto itend = lane_point_mat.end<uchar>();
                for(; i<lane_point_mat.rows; ++i){
                    for(; j<lane_point_mat.cols; ++j,++it){
                        if(*it == 2){
                            sum_Xi += i;
                            sum_Yi += j;
                            sum_Xi_expo += i*i;
                            sum_Yi_expo += j*j;
                            sum_XixYi = i*j;
                            ++N;
                        }
                       // if(*it == itend){//itend+1이랑 같아지면 끝내야하나?.. 이조건 없어도 되나?
                         //   break;
                        }
                    }
                cov_xx = (sum_Xi_expo - sum_Xi*sum_Xi/N)/N;
                cov_xy = (sum_XixYi - sum_Xi*sum_Yi/N)/N;
                cov_yy = (sum_Yi_expo - sum_Yi*sum_Yi/N)/N;
                
                double theta = std::atan2(2*cov_xy, cov_xx-cov_yy)/2;
                model.mx = std::cos(theta);
                model.my = std::sin(theta);
                model.sx = sum_Xi/N;
                model.sy = sum_Yi/N;
                /***///cov_xx == cos(theta)^2, cov_yy = sin(theta)^2
                /***///cov_xx - cov_yy == cos(theta)^2 - sin(theta)^2
                /***/// == cos(2*theta)
                /***///2*cov_xy == 2*cos(theta)*sin(theta)
                /***/// == sin(2*theta)
                /***///atan2(y,x)--> atan2(sin(2theta),cos(2*theta)) 로 적용시키기 위해 위와 같이 쓴것임.
                /***///2로 나누는 이유는 2*cov_xy를 넣어줬으므로..
                /***///******************************//
                /***///** Xm == sigma(Xi)/N ** (vice versa for Y)
                /***///** Xm is means of Xi **
                /***///N*s_dev_x(standard devision for X)^2 == sigma((Xi - Xm)^2) 
                /***///== sigma(Xi^2) - 2*Xm(sigma(Xi)) + sigma(Xm^2)
                /***///== sigma(Xi^2) - 2*Xm*Xm*N + N*Xm*Xm
                /***///== sigma(Xi^2) - N*(Xm^2)
                /***///******************************//
                /***///N*cov(this likes s_dev_xy)^2 == sigma((Xi-Xm)(Yi-Ym))
                /***///== sigma(Xi*Yi) - sigma(Xm*Yi) - sigma(Xi*Ym) + sigma(Xm*Ym)
                /***///== sigma(Xi*Yi) - Xm*sigma(Yi) - Ym*sigma(Xi) + N*Xm*Ym
                /***///== sigma(Xi*Yi) - Xm*N*Ym - Ym*N*Xm + N*Xm*Ym
                /***///== sigma(Xi*Yi) - N*Xm*Ym
                /***///== sigma(Xi*Yi) - N*(sgima(Xi)/N)*(sigma(Yi)/N)
                /***///== sigma(Xi*Yi) - sigma(Xi)*sigma(Yi)/N
                
            }
            //////********http://blog.daum.net/pg365/62*****//////
            /////*********RANSAC algorithm 출처 *************//////
            void CalLane::computeModelParameter(CalLane::sPoint samples[], int no_samples, CalLane::sLine &model){
                // PCA 방식으로 직선 모델의 파라메터를 예측한다.
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
                double sx  = 0, sy  = 0;
                double sxx = 0, syy = 0;
                double sxy = 0, sw  = 0;

                for(int i = 0; i<no_samples;++i)
                {
                    double &x = samples[i].x;
                    double &y = samples[i].y;

                    sx  += x;	
                    sy  += y;
                    sxx += x*x; 
                    sxy += x*y;
                    syy += y*y;
                    sw  += 1;
                }

                //variance;
                double vxx = (sxx - sx*sx/sw)/sw;
                double vxy = (sxy - sx*sy/sw)/sw;
                double vyy = (syy - sy*sy/sw)/sw;
                
                //principal axis
                double theta = atan2(2*vxy, vxx - vyy)/2;
                
                model.mx = cos(theta);
                model.my = sin(theta);
                
                //center of mass(xc, yc)
                model.sx = sx/sw;
                model.sy = sy/sw;
                
                //직선의 방정식: sin(theta)*(x - sx) = cos(theta)*(y - sy);
                //////********http://blog.daum.net/pg365/62*****//////  
                /////*********RANSAC algorithm 출처 *************//////
            }
            bool CalLane::findDupSamples(CalLane::sPoint *samples, int no_samples, CalLane::sPoint *data){
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
                for (int i=0; i<no_samples; ++i) {
                if (samples[i].x == data->x && samples[i].y == data->y) {
                    return true;
                    }
                }
                return false;
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
            }
            void CalLane::getSamples(CalLane::sPoint *samples, int no_samples, CalLane::sPoint *data, int no_data){
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
                // 데이터에서 중복되지 않게 N개의 무작위 셈플을 채취한다.
                for (int i=0; i<no_samples; ) {
                    int j = rand()%no_data;
                    
                    if (!findDupSamples(samples, i, &data[j])) {
                        samples[i] = data[j];
                        ++i;
                    }
                };
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
            }
            double CalLane::computeDistance(CalLane::sLine &line, CalLane::sPoint &x){
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
                // 한 점(x)로부터 직선(line)에 내린 수선의 길이(distance)를 계산한다.
            	return fabs((x.x - line.sx)*line.my - (x.y - line.sy)*line.mx)/sqrt(line.mx*line.mx + line.my*line.my);
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
            }
            double CalLane::modelVerification(CalLane::sPoint *inliers, int *no_inliers, CalLane::sLine &estimated_model, CalLane::sPoint *data, int no_data, double distance_threshold){
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////            
                *no_inliers = 0;
                double cost = 0.;

                for(int i=0; i<no_data; i++){
                    // 직선에 내린 수선의 길이를 계산한다.
                    double distance = computeDistance(estimated_model, data[i]);
                
                    // 예측된 모델에서 유효한 데이터인 경우, 유효한 데이터 집합에 더한다.
                    if (distance < distance_threshold) {
                        cost += 1.;

                        inliers[*no_inliers] = data[i];
                        ++(*no_inliers);
                    }
                }

                return cost;
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
            }
            double CalLane::ransacLineFitting(CalLane::sPoint *data, int no_data, CalLane::sLine &model, double distance_threshold){
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
                const int no_samples = 2;
                
                if (no_data < no_samples) {
                    return 0.;
                }

                CalLane::sPoint *samples = new CalLane::sPoint[no_samples];//edge좌표...
                    

                int no_inliers = 0;
                CalLane::sPoint *inliers = new CalLane::sPoint[no_data];

                CalLane::sLine estimated_model;
                double max_cost = 0.;

                int max_iteration = (int)(1 + log(1. - 0.99)/log(1. - pow(0.5, no_samples)));

                for (int i = 0; i<max_iteration; i++) {
                    // 1. hypothesis

                    // 원본 데이터에서 임의로 N개의 셈플 데이터를 고른다.
                    getSamples(samples, no_samples, data, no_data);

                    // 이 데이터를 정상적인 데이터로 보고 모델 파라메터를 예측한다.
                    computeModelParameter(samples, no_samples, estimated_model);

                    // 2. Verification

                    // 원본 데이터가 예측된 모델에 잘 맞는지 검사한다.
                    double cost = modelVerification(inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);

                    // 만일 예측된 모델이 잘 맞는다면, 이 모델에 대한 유효한 데이터로 새로운 모델을 구한다.
                    if (max_cost < cost) {
                        max_cost = cost;
                        computeModelParameter(inliers, no_inliers, model);
                    }
                }
                
                delete [] samples;
                delete [] inliers;

                return max_cost;
                //////********http://blog.daum.net/pg365/62*****//////
                /////*********RANSAC algorithm 출처 *************//////
            }
            //////********http://blog.daum.net/pg365/62*****//////
            /////*********RANSAC algorithm 출처 *************//////
            
            vec2i_t CalLane::laneLinearEquation(int x_0, int y_0, int x_1, int y_1){
                vec2i_t lane_coordi;
                return lane_coordi;
            }
            vec4i_t CalLane::houghTransform(cv::Mat& b_img){
                vec4i_t hough_vec;
                return hough_vec;
            }
            cv::Mat CalLane::myCanny(cv::Mat raw_img){ 
                int min_threshold = 80;
                int max_threshold = 15;
                int kernel_size = 3;
                vector<Vec4i> laneInfo;
                cv::Mat gray_img, blur_img, canny_img;
                cv::cvtColor(raw_img,gray_img,COLOR_BGR2GRAY);
                cv::GaussianBlur(gray_img,blur_img,Size(5,5),0,5);
                cv::Canny(blur_img,canny_img,min_threshold,max_threshold,kernel_size);//파라미터 조절 알고리즘짜기
                
                return canny_img;
            }
            void CalLane::storeFrameForVideoFile(cv::Mat& frame){
                
            }
            void CalLane::storeFrameForWebCam(cv::Mat& frame){

            }
            void CalLane::checkFrame(vec_mat_t frame_vec){

            }
            cv::Mat CalLane::makeLanePoint(cv::Mat lane_img){
                return lane_img;
            }
            void CalLane::drawLane(vec2i_t coordi_vec){

            }

            ////여기부터 추가본(color와 lable이용)
            void CalLane::detectHSVcolor(const cv::Mat& src, cv::Mat& dst, double minHue, double maxHue, double minSat, double maxSat, double minVal = 0, double maxVal = 255) {
                //detectHSVcolor(bev, yellow, 7, 21, 52, 151, 0, 180);//tackbar is false
                cv::Mat hsv;
                cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

                std::vector<cv::Mat> channels;
                cv::split(hsv, channels);

                cv::Mat mask1;
                cv::Mat mask2;
                
                
                cv::threshold(channels[0], mask1, maxHue, 255, cv::THRESH_BINARY_INV);
                cv::threshold(channels[0], mask2, minHue, 255, cv::THRESH_BINARY);
                cv::Mat hueMask;
                if (minHue < maxHue) {
                hueMask = mask1 & mask2;//all values are zero
                }
                else {//minHue > maxHue
                hueMask = mask1 | mask2;//all values are 255
                }
                cv::threshold(channels[1], mask1, maxSat, 255, cv::THRESH_BINARY_INV);

                cv::threshold(channels[1], mask2, minSat, 255, cv::THRESH_BINARY);

                cv::Mat satMask;
                satMask = mask1 & mask2;

                cv::threshold(channels[0], mask1, maxVal, 255, cv::THRESH_BINARY_INV);
                cv::threshold(channels[0], mask2, minVal, 255, cv::THRESH_BINARY);

                cv::Mat valMask;
                valMask = mask1 & mask2;

                dst = hueMask & satMask & valMask;
                }
            void CalLane::birdEyeView(cv::Mat src, cv::Mat& dst) {
                // Input Quadilateral or Image plane coordinates
                cv::Point2f inputQuad[4];
                // Output Quadilateral or World plane coordinates
                cv::Point2f outputQuad[4];

                // Lambda Matrix
                cv::Mat lambda(2, 4, CV_32FC1);

                // Set the lambda matrix the same type and size as input
                lambda = cv::Mat::zeros(dst.rows, dst.cols, dst.type());

                // The 4 points that select quadilateral on the input , from top-left in clockwise order
                // These four pts are the sides of the rect box used as input
                inputQuad[0] = cv::Point2f(0, dst.rows*0.5);
                inputQuad[1] = cv::Point2f(dst.cols, dst.rows*0.5);
                inputQuad[2] = cv::Point2f(dst.cols, dst.rows*0.85);
                inputQuad[3] = cv::Point2f(0, dst.rows*0.85);
                // The 4 points where the mapping is to be done , from top-left in clockwise order
                outputQuad[0] = cv::Point2f(0, 0);
                outputQuad[1] = cv::Point2f(dst.cols - 1, 0);
                outputQuad[2] = cv::Point2f(dst.cols*0.6, dst.rows - 1);
                outputQuad[3] = cv::Point2f(dst.cols*0.4, dst.rows - 1);

                // Get the Perspective Transform Matrix i.e. lambda
                lambda = getPerspectiveTransform(inputQuad, outputQuad);
                // Apply the Perspective Transform just found to the src image
                warpPerspective(src, dst, lambda, dst.size());
                }
            void CalLane::inverseBirdEyeView(cv::Mat src, cv::Mat& dst) {
                // Input Quadilateral or Image plane coordinates
                cv::Point2f inputQuad[4];
                // Output Quadilateral or World plane coordinates
                cv::Point2f outputQuad[4];

                // Lambda Matrix
                cv::Mat lambda(2, 4, CV_32FC1);

                // Set the lambda matrix the same type and size as input
                lambda = cv::Mat::zeros(src.rows, src.cols, src.type());

                // The 4 points that select quadilateral on the input , from top-left in clockwise order
                // These four pts are the sides of the rect box used as input
                inputQuad[0] = cv::Point2f(0, src.rows*0.5);
                inputQuad[1] = cv::Point2f(src.cols, src.rows*0.5);
                inputQuad[2] = cv::Point2f(src.cols, src.rows*0.85);
                inputQuad[3] = cv::Point2f(0, src.rows*0.85);
                // The 4 points where the mapping is to be done , from top-left in clockwise order
                outputQuad[0] = cv::Point2f(0, 0);
                outputQuad[1] = cv::Point2f(src.cols - 1, 0);
                outputQuad[2] = cv::Point2f(src.cols*0.6, src.rows - 1);
                outputQuad[3] = cv::Point2f(src.cols*0.4, src.rows - 1);

                // Get the Perspective Transform Matrix i.e. lambda
                lambda = getPerspectiveTransform(outputQuad, inputQuad );
                // Apply the Perspective Transform just found to the src image
                warpPerspective(src, dst, lambda, dst.size());
                }
            void CalLane::makeYProjection(cv::Mat src, cv::Mat dst, unsigned int* H_result) {
                uchar pixel;
                unsigned int *H = new unsigned int[src.cols];
                std::memset(H, 0x00, 4 * src.cols);
                for (int y = 0; y < src.rows; y++) {
                uchar* hist_data = src.ptr<uchar>(y+0);
                    for (int x = 0; x < src.cols; x++) {//1채널이라 (left+width)에 채널값 안곱함    
                        if (hist_data[x + 0] != (uchar)0) {
                            H[x]++;
                            H_result[x]++;
                            }
                        }
                }
                std::sort(H_result, H_result + src.cols);
                for (int x = 0; x < src.cols; x++) {
                    for (int y = 0; y < H[x]; y++) {
                    dst.at<uchar>(y, x) = 255;
                    }
                }
                delete[] H;
                }     
            void CalLane::makeXProjection(cv::Mat src, cv::Mat dst, unsigned int* H_result){
                uchar pixel;
                unsigned int *H = new unsigned int[src.rows];
                std::memset(H, 0x00, 4 * src.rows);
                for (int x = 0; x < src.cols; x++) {
                    for (int y = 0; y < src.rows; y++) {
                    pixel = src.at<uchar>(y + 0, x + 0);
                        if (pixel != 0) {
                            H[y]++;
                            H_result[y]++;
                        }
                    }
                }

                for (int y = 0; y < src.rows; y++) {
                    for (int x = 0; x < H[y]; x++) {
                    dst.at<uchar>(y, x) = 255;
                    }
                }
                delete[] H;
                }
            void CalLane::medianForXHistogram(unsigned int* H_result, int H_result_size){
                int midean;
                unsigned int *sorted_H_result = new unsigned int[H_result_size];
                std::memset(sorted_H_result, 0, 4 * H_result_size);
                std::copy(H_result, H_result + H_result_size, sorted_H_result);
                std::sort(sorted_H_result, sorted_H_result + H_result_size);
                midean = H_result_size / 2;
                for (int y = 0; y < H_result_size; y++) {
                    for (int x = 0; x < sorted_H_result[y]; x++) {
                    
                    }std::cout << "H : " << sorted_H_result[y] << std::endl;
                }std::cout << "___" << std::endl;
                delete[] sorted_H_result;
                }     
            void CalLane::makeWindow(cv::Mat src, cv::Mat dst, int window_width, int window_offset, int per_lane_checked){
                cv::Mat_<uchar>::iterator it = src.begin<uchar>();
                unsigned int *lane_check_array = new unsigned int[window_width+window_offset];
                std::memset(lane_check_array, 0, 4 * (window_width + window_offset));
                int check_window = 0;
                
                uint *vote_lane_array = new uint[src.cols*3];//data order is 'value, x, y'
                std::memset(vote_lane_array, 0, 4 * (src.cols * 3));
                int max_lane = 0, temp = 0, vote_index=0;
                
                it = it + (src.rows - 1)*src.cols;
                for (uint i = 1; i < src.rows; ++i) {
                    for (uint j = 0; j < src.cols*per_lane_checked; ++j) {
                        check_window = j;
                        while (*(it+check_window) != (uchar)0) {
                            check_window++;
                            temp++;
                            try {
                                if (*(it + check_window) == (uchar)0) {
                                vote_lane_array[vote_index] = temp;
                                vote_lane_array[vote_index + 1] = j;
                                vote_lane_array[vote_index + 2] = i;
                                vote_index += 3;
                                check_window = 0;
                                temp = 0;
                                break;
                                }
                            }
                            catch (cv::Exception e) {
                            std::cout << "makeWindow fuction error!\n Wrong iterator access!\n"<< e.what() << std::endl;
                            }
                        }
                        temp = 0;
                        if (check_window != 0 && *it != (uchar)0) {//check_window의 끝까지 1이 채워져있을 경우 try의 if문에 걸리지 못하고 빠져나오게 되므로 이를 처리
                            vote_lane_array[vote_index] = temp;
                            vote_lane_array[vote_index + 1] = j;
                            vote_lane_array[vote_index + 2] = i;
                            //vote value
                            vote_index = 0;
                            check_window = 0;    
                        }
                    
                    }
                    it = it + (src.rows - i - 1)*src.cols;
                }
                delete[] lane_check_array;
                delete[] vote_lane_array;
                }        
            void CalLane::slideWindow(cv::Mat src, int window_width, int window_height, int per_lane_in_window){  
                std::cout << "window_width : " << src.rows << std::endl << "window_height : " << src.cols;
                /***/// cv::Rect roi(10, 20, 100, 50);
                /***/// cv::Mat window = src(roi);
                cv::Mat window = src(cv::Rect(0, src.rows - window_height, window_width, window_height)).clone();
                cv::imshow("window", window);
                cv::MatIterator_<uchar> it;// = window.end<uchar>();
                //it = it - window.cols - 1;
                /*
                for (int i = 1; i <= src.rows; ++i) {
                it = window.end<uchar>() - window.cols*i - 1;
                for (int j = 1; j < src.cols/2; ++j) {
                if (addMat(window, per_lane_in_window)) {//bool함수의 리턴값에 따라 슬라이딩 윈도우 고정시킴
                    int count = j;
                    if(*it != (uchar)0) {
                    while (*(it + count) != (uchar)0) {//it+count의 vlaue가 0이 되면 종료
                    ++count;
                    }
                    while (*(it + count) == (uchar)0) {//it+count의 vlaue가 1이 되면 종료
                    *(it + count) = (uchar)255;
                    }
                    } 
                }
                ++it;
                window(cv::Rect(j, 0, window.cols, window.rows));
                cv::Rect(0,src.rows-window.rows, window.cols, window.rows);
                }
                }*/
                }
            bool CalLane::addMat_imsi(cv::Mat src, int per_lane_in_window){
                cv::MatIterator_<uchar> it = src.begin<uchar>();
                int count = 0;
                for (int i = 0; i < src.rows; ++i) {
                    for (int j = 0; j < src.cols; ++j) {
                        if (*it != (uchar)0) {
                            count++;
                        }
                    ++it;
                    }
                }
                if (count >= src.rows*src.cols*per_lane_in_window){
                return true;
                    }
                else { // count < src.rows*src.cols*per_lane_in_window. this means not enough lane detected in window.
                return false;
                    }
                } 
            void CalLane::makeContoursLeftLane(cv::Mat src, cv::Mat& dst) {
                std::vector<std::vector<cv::Point>> countours;
                std::vector<cv::Vec4i> hierachy;

                cv::findContours(src, countours, hierachy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
                dst = cv::Mat::zeros(src.size(), CV_8UC3);

                for (int i = 0; i < countours.size(); ++i) {
                    cv::drawContours(dst, countours, i, CV_RGB(255, 255, 255), -1, 8, hierachy, 0, cv::Point());
                }
                cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);  // Convert the image to Gray
                cv::threshold(dst, dst, 127, 255, cv::THRESH_BINARY);
               // cv::Mat draw_lable;//레이블확인용
               // cv::threshold(dst, draw_lable, 127, 255, cv::THRESH_BINARY_INV);//레이블 확인용
                cv::Mat img_labels, stats, centroids;
                int numOfLables = cv::connectedComponentsWithStats(dst, img_labels, stats, centroids, 8, CV_32S);

                
                int temp_area = 0, max_area = 0;
                for (int row = 1; row < numOfLables; row++) {
                    int* area_data = stats.ptr<int>(row);
                    temp_area = area_data[cv::CC_STAT_AREA];
                    if (temp_area > max_area) {
                    max_area = temp_area;
                    }
                }
                
                for (int row = 1; row < numOfLables; row++) {
                
                int* data = stats.ptr<int>(row);   
                int area = data[cv::CC_STAT_AREA];
                int left = data[cv::CC_STAT_LEFT];
                int top = data[cv::CC_STAT_TOP];
                int width = data[cv::CC_STAT_WIDTH];
                int height = data[cv::CC_STAT_HEIGHT]; 
                
               // cv::rectangle(draw_lable,cv::Point(left,top),cv::Point(left+width,top+height),cv::Scalar(0,0,255),1);
               // imshow("lable", draw_lable);
                if (area == max_area && width<height && left<src.cols / 2 && width<src.cols / 2) {//이 조건들에 추가조건 더해서 레이블 유효성 검사
                    for (int delete_row = dst.rows - 1, coordi_index = 0; delete_row >= 0; --delete_row, ++coordi_index) {
                        uchar* delete_data = dst.ptr<uchar>(delete_row);
                        
                        for (int delete_col = dst.cols-1 ; delete_col>=0; --delete_col) {
                        if ((delete_col > left + width || delete_col < left) || (delete_row<top || delete_row>top + height)) {
                        delete_data[delete_col] = (uchar)0;
                        }
                        //else {//range of max lable box///////////////////////for visible center line////////////////////////////////////////////
                        // if (delete_data[delete_col] != 0 && !lane_checked && coordi_index<dst.cols/2*3) {
                        //  lane_checked = true;
                        //  left_lane_first_coordi[coordi_index + coordi_offset] = (uint)delete_col;
                        //  left_lane_first_coordi[coordi_index + coordi_offset + 1] = (uint)delete_row;
                        //  int lane_width_check = 0;
                        //  while (delete_data[delete_col - lane_width_check] != 0) {
                        //   lane_width_check++;
                        //  }
                        //  left_lane_first_coordi[coordi_index + coordi_offset + 2] = lane_width_check;
                        // }

                        //}/////////////////////////////////////////////////////////////////////////////////
                        }
                    }
                //box check
                    //turn left
                //for(int )/////////////////////////////////////////////////////////////
                    //turn right/////////////////////짜야해!!!///////////////////////////
                }/////////////////////////////////////////////////////////////////////////
                else {//delete wrong lable
                    for (int row = top; row < top + height; row++) {
                        uchar* data = dst.ptr<uchar>(row);
                        for (int col = left; col < left + width; col++) {//1채널이라 (left+width)에 채널값 안곱함    
                        data[col] = (uchar)0;
                        
                        }
                    }
                }
               // imshow("lane_left", dst);
                }
               // imshow("drawing", draw_max_lable);//for visible lane max lable box
                }
           void CalLane::makeContoursRightLane(cv::Mat src, cv::Mat& dst){
                std::vector<std::vector<cv::Point>> countours;
                std::vector<cv::Vec4i> hierachy;

                cv::findContours(src, countours, hierachy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
                dst = cv::Mat::zeros(src.size(), CV_8UC3);

                for (int i = 0; i < countours.size(); ++i) {
                    cv::drawContours(dst, countours, i, CV_RGB(255, 255, 255), -1, 8, hierachy, 0, cv::Point());
                }
                cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);  // Convert the image to Gray
                cv::threshold(dst, dst, 127, 255, cv::THRESH_BINARY);
                cv::Mat draw_max_lable;

                cv::threshold(dst, draw_max_lable, 127, 255, cv::THRESH_BINARY_INV);
                cv::Mat img_labels, stats, centroids;
                int numOfLables = cv::connectedComponentsWithStats(dst, img_labels, stats, centroids, 8, CV_32S);

                int temp_area = 0, max_area = 0;
                for (int row = 1; row < numOfLables; row++) {
                    int* area_data = stats.ptr<int>(row);
                    temp_area = area_data[cv::CC_STAT_AREA];
                    if (temp_area > max_area) {
                        max_area = temp_area;
                    }
                }
                int center_x;
                int center_y;

                for (int row = 1; row < numOfLables; row++) {

                    int* data = stats.ptr<int>(row);
                    int area = data[cv::CC_STAT_AREA];
                    int left = data[cv::CC_STAT_LEFT];
                    int top = data[cv::CC_STAT_TOP];
                    int width = data[cv::CC_STAT_WIDTH];
                    int height = data[cv::CC_STAT_HEIGHT];

                    if (area == max_area && width<height && left>src.cols / 3 && width<src.cols / 2) {//이 조건들에 추가조건 더해서 레이블 유효성 검사
                    
                    for (int delete_row = dst.rows-1; delete_row >= 0; --delete_row) {
                        uchar* delete_data = dst.ptr<uchar>(delete_row);
                        for (int delete_col = 0; delete_col < dst.cols; ++delete_col) {
                            if ((delete_col > left + width || delete_col < left) || (delete_row<top || delete_row>top + height)) {
                            delete_data[delete_col] = (uchar)0;
                            }
                            //else {//range of max lable box///////////////////////for visible center line////////////////////////////////////////////
                            // if (delete_data[delete_col] != 0 && !lane_checked&& coordi_index<dst.cols / 2 * 3) {
                            //  lane_checked = true;
                            //  right_lane_first_coordi[coordi_index + coordi_offset] = (uint)delete_col;
                            //  right_lane_first_coordi[coordi_index + coordi_offset + 1] = (uint)delete_row;
                            //  int lane_width_check = 0;
                            //  while (delete_data[delete_col + lane_width_check] != 0) {
                            //   lane_width_check++;
                            //  }
                            //  right_lane_first_coordi[coordi_index + coordi_offset + 2] = lane_width_check;
                            // }

                            //}///////////////////////////////////////////////////////////////////////////////////
                            }
                    
                        }

                    }
                    else {//delete wrong lable
                        for (int row = top; row < top + height; row++) {
                            uchar* data = dst.ptr<uchar>(row);
                                for (int col = left; col < left + width; col++) {//1채널이라 (left+width)에 채널값 안곱함    
                                data[col] = (uchar)0;
                                }
                        }
                    }
                //    imshow("lane_right", dst);
                    }
                //imshow("drawing", draw_max_lable);//for visible lane max lable box
                }     
    
}

/*
//포인터 o
std::vector<int*> a;
int* pA = NULL;
a.reserve(3);//예약 걸어둘수있는건 다 걸어두자
pA= new int;
a.push_back(pA);
pA=new int;
a.push_back(pA);
이런 데이터일 경우

std::vector<int*>::iterator iter;
for(iter = a.begin(); iter!=a.end(); ++iter)
{
    delete (*iter);
}a.clear();

//포인터 x
std::vector vclear;
a.swap(vclear);

vclear.clear();
a.clear();
*/