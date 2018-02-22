#include "odometry/ackermann.h"


#define PI 3.141592
#define IGNORE_STEER_LOWER 0.0001 * (PI/180.0)
#define IGNORE_STEER_UPPER 20.0 * (PI/180.0)
#define TIME_NEER_ZERO 0.0001
#define CURVATURE_NEER_ZERO 0.0001
#define STEER_PLATFORM_TO_RAD 20.0/2000.0 * (PI/180.0)
#define TIME_GAIN 1.0
#define TRANS_COV 3.0
#define ROT_COV 99999.0

namespace odometry_ackermann{

Odometry::Odometry()
    : wheelbase_(1.0)
    , timestamp_(0.0)
    , velocity_(0.0)
    , steering_(0.0)
    , heading_(0.0)
    , curvature_(0.0)
    , ignore_low_(0.0)
    , ignore_up_(0.0)
    , ds_(0.0)
    , dth_(0.0)
    , dx_(0.0)
    , dy_(0.0)
    , x_(0.0)
    , y_(0.0)
{
    nh_.getParam("odom_init_heading",heading_);
}



void Odometry::init(const ros::Time &time)
{
    timestamp_ = time;
    pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);         // publish 할 인스턴스 정의 
<<<<<<< HEAD
    //init odom
    odom_.header.stamp = time;
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_link";
    //init tf
    odom_trans_.header.frame_id = "odom";
    odom_trans_.child_frame_id = "base_link";
    
    ROS_INFO("OdomINIT FINISHED!");
=======
    /*    init odom    */
    odom_.header.stamp = time;
    odom_.header.frame_id = "odom_combined";
    odom_.child_frame_id = "base_link";
    /*     init tf     */
    odom_trans_.header.stamp = time;
    odom_trans_.header.frame_id = "odom_combined";
    odom_trans_.child_frame_id = "base_link";
>>>>>>> upstream/master
}



void Odometry::callback(const platform_rx_msg::platform_rx_msg::ConstPtr& PlatformRX_data)
{         
    ros::Time time = ros::Time::now(); // 현재 주기의 시간 측정 (Odometry Loop의 주기 측정 -> dt -> Odometry 계산)

    Calc_Odom(PlatformRX_data, time);  // odometry 계산
   
    Odom_Transform(time);              // odometry transform 계산

    Odom_Set(time);                    // odometry 정보 입력
}

bool Odometry::Calc_Odom(const platform_rx_msg::platform_rx_msg::ConstPtr& PlatformRX_data,
                         const ros::Time& time)
{
    const double dt = (time - timestamp_).toSec();                  // Calc_Odom 함수가 호출되는 주기 계산 dt
    timestamp_ = time;                                              // timestamp 갱신
    
    velocity_ = PlatformRX_data->speed;
    steering_ = PlatformRX_data->steer * STEER_PLATFORM_TO_RAD;

    nh_.getParam("wheelbase", wheelbase_);
    curvature_ = wheelbase_ / cos( PI/2 - steering_ );
    
    /*     curvature 범위 설정 (일종의 필터)    */
    //ignore_low_    = wheelbase_ / cos( PI/2 - IGNORE_STEER_LOWER );
    //ignore_up_     = wheelbase_ / cos( PI/2 - IGNORE_STEER_UPPER );   
    
    //ROS_INFO("Max: %lf", ignore_low_);
    ROS_INFO("Curvature: %lf [m]", curvature_);
    //ROS_INFO("Min: %lf", ignore_up_);

    //if(fabs(curvature_) > ignore_up_ && fabs(curvature_) < ignore_low_){
        ds_ = velocity_ * dt * TIME_GAIN;
        dth_ = ds_ / curvature_;
        const double x_curvature = curvature_ * sin(dth_);
        const double y_curvature = curvature_ * (cos(dth_) - 1.0);
        const double wheel_heading = heading_ + steering_;
        dx_  = x_curvature * cos(wheel_heading) - y_curvature * sin(wheel_heading);
        dy_  = x_curvature * sin(wheel_heading) + y_curvature * cos(wheel_heading);
        dth_ = ds_ / curvature_;
        x_ += dx_;
        y_ += dy_;
        heading_ += dth_;
    //}
    //else{
    //    ds_ = velocity_ * dt;
    //    dth_ = 0.0;
    //    dx_ = ds_;
    //    dy_ = 0.0;
    //    x_ += dx_;
    //    y_ += dy_;
    //    heading_ += dth_;
    //}
    
    if (dt < TIME_NEER_ZERO)
        return false;
    else
        return true;
}


void Odometry::Odom_Set(const ros::Time& time)
{
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_);
    
    odom_.header.stamp = time;
<<<<<<< HEAD
    odom_.header.frame_id = "odom";
    odom_.pose.pose.position.x = x_;
    odom_.pose.pose.position.y = y_;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation = odom_quat;
    odom_.child_frame_id = "base_link";
    odom_.twist.twist.linear.x = dx_;
    odom_.twist.twist.linear.y = dy_;
    odom_.twist.twist.angular.z = dth_;
    ROS_INFO("OdomSet FINISHED!");
=======
>>>>>>> upstream/master

    boost::array<double, 36> covariance = {{
    TRANS_COV, 0, 0, 0, 0, 0,
    0, TRANS_COV, 0, 0, 0, 0,
    0, 0, TRANS_COV, 0, 0, 0,
    0, 0, 0, ROT_COV, 0, 0,
    0, 0, 0, 0, ROT_COV, 0,
    0, 0, 0, 0, 0, ROT_COV
    }};

<<<<<<< HEAD
    odom_.pose.covariance = covariance;

=======
    odom_.pose.pose.position.x = x_;
    odom_.pose.pose.position.y = y_;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation = odom_quat;
    odom_.pose.covariance = covariance;
    
    odom_.twist.twist.linear.x = dx_;
    odom_.twist.twist.linear.y = dy_;
    odom_.twist.twist.angular.z = dth_;
    //odom_.twist.covariance = covariance;
>>>>>>> upstream/master
}


void Odometry::Odom_Transform(const ros::Time& time)
{   
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_);
         
    odom_trans_.header.stamp = time;
<<<<<<< HEAD
    odom_trans_.header.frame_id = "odom";
    odom_trans_.child_frame_id = "base_link";    
=======
>>>>>>> upstream/master
    odom_trans_.transform.translation.x = x_;
    odom_trans_.transform.translation.y = y_;
    odom_trans_.transform.translation.z = 0.0;
    odom_trans_.transform.rotation = odom_quat;
<<<<<<< HEAD
    ROS_INFO("OdomTransform FINISHED!");
=======
>>>>>>> upstream/master
}


void Odometry::sendTransform(){
    odom_broadcaster_.sendTransform(odom_trans_);
}

<<<<<<< HEAD
=======

>>>>>>> upstream/master
void Odometry::publish(){
    pub_.publish(odom_);
}

}