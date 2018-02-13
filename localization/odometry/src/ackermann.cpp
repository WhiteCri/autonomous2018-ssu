#include "odometry/ackermann.h"


#define PI 3.141592
#define IGNORE_STEER 0.001 // IGNORE_STEER 이하의 각도[deg]에서는 직진으로 odometry 계산(singularity point 때문에)
#define TIME_NEER_ZERO 0.0001
#define CURVATURE_NEER_ZERO 0.0001
#define STEER_PLATFORM_TO_RAD 1/100.0*(PI/180.0)
#define TIME_GAIN 1
#define TRANS_COV 5
#define ROT_COV 5

namespace odometry_ackermann{

Odometry::Odometry()
    : wheelbase_(1.0)
    , timestamp_(0.0)
    , velocity_(0.0)
    , steering_(0.0)
    , heading_(0.0)
    , curvature_(0.0)
    , ignore_(0.0)
    , ds_(0.0)
    , dth_(0.0)
    , dx_(0.0)
    , dy_(0.0)
    , x_(0.0)
    , y_(0.0){}



void Odometry::init(const ros::Time &time)
{
    ROS_INFO("OdomINIT START!");
    timestamp_ = time;
    pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);         // publish 할 인스턴스 정의 
    //init odom
    odom.header.stamp = time;
    odom.header.frame_id = "odom"; // "odom"
    odom.child_frame_id = "base_link";
    //init tf
    odom_trans.header.frame_id = "odom"; // "odom"
    odom_trans.child_frame_id = "base_link";
    
    ROS_INFO("OdomINIT FINISHED!");
}



void Odometry::callback(const platform_rx_msg::platform_rx_msg::ConstPtr& PlatformRX_data)
{  
    ROS_INFO("OdomCallback START!");

    //tf::TransformBroadcaster odom_broadcaster;                     // tf broadcast
    //nav_msgs::Odometry odom;                                       // publish할 odometry
    //geometry_msgs::TransformStamped odom_trans;                    // tf으로 날릴 odometry transform
       
    /*---  Odometry Loop의 주기 측정 -> dt -> Odometry 계산  ---*/   
    ros::Time time = ros::Time::now();                             // 현재 주기의 시간 측정
    

    Calc_Odom(PlatformRX_data, time);                              // odometry 계산
    Odom_Transform(time);                              // odometry transform 계산
    //odom_broadcaster.sendTransform(odom_trans);                    // tf에 odometry transform을 broadcast

    Odom_Set(time);                                          // odometry 정보 입력
    
    //pub_.publish(odom);                                            // odometry 메세지 publish
    ROS_INFO("OdomCallback FINISHED!");
}

bool Odometry::Calc_Odom(const platform_rx_msg::platform_rx_msg::ConstPtr& PlatformRX_data,
                         const ros::Time& time)
{
    ROS_INFO("OdomCalc STARTS!");
    const double dt = (time - timestamp_).toSec();                  // Calc_Odom 함수가 호출되는 주기 계산 dt
    timestamp_ = time;                                              // timestamp 갱신
    
    velocity_ = PlatformRX_data->speed;
    steering_ = PlatformRX_data->steer/STEER_PLATFORM_TO_RAD;
    
    //TEST
    //velocity_ = 1;
    //steering_ = PI/6;

    nh_.getParam("wheelbase", wheelbase_);
    curvature_ = wheelbase_ / cos( PI/2 - steering_ );
    ignore_    = wheelbase_ / cos( PI/2 - IGNORE_STEER*(PI/180) );   
       
    if(fabs(curvature_) > CURVATURE_NEER_ZERO && fabs(curvature_) < ignore_){
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
    }
    else{
        ds_ = velocity_ * dt;
        dth_ = 0.0;
        dx_ = ds_;
        dy_ = 0.0;
        x_ += dx_;
        y_ += dy_;
        heading_ += dth_;
    }

    
    if (dt < TIME_NEER_ZERO)
        return false;
    else
        return true;
    ROS_INFO("OdomCalc FINISHED!");
}

void Odometry::Odom_Set(const ros::Time& time)
{
    ROS_INFO("OdomSet STARTS!");
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_);
    
    odom.header.stamp = time;
    odom.header.frame_id = "odom"; // "odom"
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = dx_;
    odom.twist.twist.linear.y = dy_;
    odom.twist.twist.angular.z = dth_;
    ROS_INFO("OdomSet FINISHED!");

    boost::array<double, 36> covariance = {{
    TRANS_COV, 0, 0, 0, 0, 0,
    0, TRANS_COV, 0, 0, 0, 0,
    0, 0, TRANS_COV, 0, 0, 0,
    0, 0, 0, ROT_COV, 0, 0,
    0, 0, 0, 0, ROT_COV, 0,
    0, 0, 0, 0, 0, ROT_COV
    }};

    odom.pose.covariance = covariance;

}

void Odometry::Odom_Transform(const ros::Time& time)
{   
    ROS_INFO("OdomTransform STARTS!");
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading_);
         
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom"; // "odom"
    odom_trans.child_frame_id = "base_link";    
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    ROS_INFO("OdomTransform FINISHED!");
}

void Odometry::sendTransform(){
    odom_broadcaster.sendTransform(odom_trans);
}
void Odometry::publish(){
    pub_.publish(odom);
}
}