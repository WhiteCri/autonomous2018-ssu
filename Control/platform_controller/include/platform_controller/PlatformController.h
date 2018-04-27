#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "platform_controller/cmd_platform.h"
#include "platform_rx_msg/platform_rx_msg.h"
#include <geometry_msgs/Twist.h>

/* Platform Protocol 관련 */
#define NO_ACCEL 0
#define MAX_ACCEL 200
#define NO_BRAKE 1
#define NO_SLIP_BRAKE 70
#define MAX_BRAKE 200
#define MAX_STEER 2000
#define GEAR_FORWARD 0
#define GEAR_BACKWARD 2
#define SWITHCING_GEAR_SPEED_TOLERANCE 0.2

/* Platform Dynamics 관련 */
#define PI 3.141592
#define RAD2SERIAL (180.0/PI)*100.0 // rad -> serial
#define M_S2SERIAL 1.0/0.0598 // m/s -> serial (@ steady-state)
#define C0 0.1080 // acceleration [m/s^2] = 0.108*exp(0.0184*platform) -> C0 = 0.108
#define C1 0.0184 // acceleration [m/s^2] = 0.108*exp(0.0184*platform) -> C1 = 0.0184
#define C2 0.0598 // steady-state speed [m/s] = 0.0598*platform        -> C2 = 0.0598

#define FILTER_SIZE 2 // Reference Steering Angle 진동 잡기 위한 이동평균필터 크기
#define EPSILON 0.05   // log(negative) 방지

/* Debug */
//#define MY_DEBUG
//#define MY_TEST

class PlatformController {
public:
PlatformController()
    : ref_speed_(0.0), current_speed_(0.0), err_speed_(0.0)
    , ref_steer_(0.0), current_steer_(0.0), err_steer_(0.0)
    , cmd_accel_(0.0), cmd_steer_(0.0), cmd_brake_(0.0)
    , kp_steer_(1.0), ki_steer_(0.0), kd_steer_(0.0)
    , kp_brake_(30.0), ki_brake_(0.0), kd_brake_(0.0)
    , settling_time_(0.8), brake_max_(NO_SLIP_BRAKE)
    , dt_(0.0), index_(0)
    , ss_speed_(0), ss_speed_weight_(1.35), ss_speed_shift_(20.0)
    , target_accel_(0.0), current_gear_(GEAR_FORWARD), target_gear_(GEAR_FORWARD) 
{
     
}


void Init(int argc, char **argv){ // Controller 돌리기 전에 initialize (dt 계산을 위한 time 초기값)
    ros::init(argc, argv, "Platform_Controller");
    ros::NodeHandle priv_nh_("~");
    ros::NodeHandle nh_;

    priv_nh_.param<double>("/control/accel/settling_time", settling_time_, 0.8);
    priv_nh_.param<double>("/control/speed/weight", ss_speed_weight_, 1.1);
    priv_nh_.param<double>("/control/speed/shift", ss_speed_shift_, 0.0);

    priv_nh_.param<double>("/control/steer/kp", kp_steer_, 1.0);
    priv_nh_.param<double>("/control/steer/ki", ki_steer_, 0.0);
    priv_nh_.param<double>("/control/steer/kd", kd_steer_, 0.0);

    priv_nh_.param<int>("/control/brake/max", brake_max_, NO_SLIP_BRAKE);
    priv_nh_.param<double>("/control/brake/kp", kp_brake_, 50.0);
    priv_nh_.param<double>("/control/brake/ki", ki_brake_, 0.0);
    priv_nh_.param<double>("/control/brake/kd", kd_brake_, 0.0);

    pub_ = nh_.advertise<platform_controller::cmd_platform>("control/cmd_platform", 100);
    
    timestamp_ = ros::Time::now();
}

void Calc_PID(void){ // read_state, read_reference로 읽은 후에 Platform_TX(SI Unit -> Platform Unit) 하기 전에 호출
    ros::Time now = ros::Time::now();
    dt_ = (now - timestamp_).toSec();
    timestamp_ = now;

    UpdateParameters(); // PID Gain Parameter Update


    Calc_longitudinal(); // SPEED CONTROL

    Calc_lateral(); // STEER CONTROL
}

void RX_Callback(const platform_rx_msg::platform_rx_msg::ConstPtr& rx_data){
    current_speed_ = rx_data->speed;
    current_steer_ = rx_data->steer;
    Calc_PID();
    publish();
}

void Cmd_Callback(const geometry_msgs::TwistConstPtr& twist){
    ref_speed_ = twist->linear.x;
    ref_steer_ = mv_avg_filter( BoundaryCheck_Steer(RAD2SERIAL*(twist->angular.z)) );

    //Calc_PID();
    //publish();
}

void publish(){
    pub_.publish(cmd_);

    #ifdef MY_TEST // rqt_plot으로 비교하기 위한 용도로 테스트
        //cmd_.reference = BoundaryCheck_Steer(ref_steer_);
        cmd_.reference = BoundaryCheck_Accel(cmd_accel_);
    #endif
    
    #ifdef MY_DEBUG
        ROS_INFO("Gear: %d (0: Forward / 2: Backward)", cmd_.gear);
        ROS_INFO("Reference Speed: %lf   Current Speed: %lf   Command Accel: %d   Command Brake: %d", ref_speed_, current_speed_ ,cmd_.accel, cmd_.brake);
        ROS_INFO("Reference Steer: %lf   Current Steer: %lf   Command Steer: %d", ref_steer_, current_steer_ ,cmd_.steer);
        ROS_INFO("Platform Command Topic Published !!!");
    #endif
}




private:
ros::Publisher pub_;
ros::Time timestamp_;

platform_controller::cmd_platform cmd_;

double filter_[FILTER_SIZE];
int index_;

double dt_;
double ref_speed_, current_speed_, err_speed_;
double ref_steer_, current_steer_, err_steer_;
double target_accel_;
int current_gear_, target_gear_, ss_speed_, brake_max_;
int cmd_accel_, cmd_steer_, cmd_brake_;

double settling_time_;
double kp_steer_, ki_steer_, kd_steer_;
double kp_brake_, ki_brake_, kd_brake_;
double ss_speed_weight_, ss_speed_shift_;



double mv_avg_filter(double data){
    double sum = 0;
 
    filter_[index_] = data;
    for(int i = 0; i < FILTER_SIZE; i++)   sum += filter_[i];
    index_ = (index_+1) % FILTER_SIZE;
    return (sum/FILTER_SIZE);
}

inline void UpdateParameters(void){
    ros::NodeHandle priv_nh_("~");
    
    priv_nh_.getParam("/control/accel/settling_time", settling_time_);
    priv_nh_.getParam("/control/speed/weight", ss_speed_weight_);
    priv_nh_.getParam("/control/speed/shift", ss_speed_shift_);

    priv_nh_.getParam("/control/steer/kp", kp_steer_);
    priv_nh_.getParam("/control/steer/ki", ki_steer_);
    priv_nh_.getParam("/control/steer/kd", kd_steer_);
    
    priv_nh_.getParam("/control/brake/max", brake_max_);
    priv_nh_.getParam("/control/brake/kp", kp_brake_);
    priv_nh_.getParam("/control/brake/ki", ki_brake_);
    priv_nh_.getParam("/control/brake/kd", kd_brake_);
}

inline int BoundaryCheck_Accel(const int accel){
    if(accel <= NO_ACCEL){
        return NO_ACCEL;
    }
    else{
        return (accel <= MAX_ACCEL) ? accel : MAX_ACCEL;
    }    
}

inline int BoundaryCheck_Brake(const int brake){
    if(brake <= NO_BRAKE){
        return NO_BRAKE;
    }
    else{
        return (brake <= brake_max_) ? brake : brake_max_;
    }
}

inline int BoundaryCheck_No_Slip_Brake(const int brake){

    if(brake <= NO_BRAKE){
        return NO_BRAKE;
    }
    else{
        return (brake <= NO_SLIP_BRAKE) ? brake : NO_SLIP_BRAKE;
    }
}

inline int BoundaryCheck_Steer(const int steer){
    return (fabs(steer) <= MAX_STEER) ? steer : MAX_STEER*(steer/fabs(steer));
}

void Calc_lateral(void){
    cmd_.steer = BoundaryCheck_Steer(ref_steer_);
}

bool Calc_longitudinal(void){
    target_gear_ = (ref_speed_ >= 0.0) ? GEAR_FORWARD : GEAR_BACKWARD;
    if(current_gear_ != target_gear_){
        if(fabs(current_speed_) > SWITHCING_GEAR_SPEED_TOLERANCE){
            cmd_.accel = NO_ACCEL;
            cmd_.brake = NO_SLIP_BRAKE; 
            return false;
        }
        else{
            cmd_.gear = target_gear_;
            current_gear_ = target_gear_;
        }
    }
    const double dir = (current_gear_ == GEAR_FORWARD) ? 1.0 : -1.0;
    err_speed_ = dir * (ref_speed_ - current_speed_);
    target_accel_ = err_speed_ / settling_time_;
    ss_speed_ = (int)(ss_speed_shift_ + ss_speed_weight_ * fabs(ref_speed_) * M_S2SERIAL);
    if(target_accel_ > C0 + EPSILON){ // 가속(Acceleration)
        double min_accel =  C0 * exp(C1 * ref_speed_*M_S2SERIAL);
        cmd_accel_ = (target_accel_ > min_accel) ? (int)(log(target_accel_ / C0) / C1) : ss_speed_;
        cmd_brake_ = NO_BRAKE;
    }
    else{ // 가속 필요 X or 감속(Deceleration)
        cmd_accel_ = ss_speed_;
        cmd_brake_ = fabs(err_speed_) * (kp_brake_ + ki_brake_*dt_ + kd_brake_/dt_);
        cmd_brake_ = BoundaryCheck_No_Slip_Brake(cmd_brake_);
    }
    cmd_.accel = cmd_accel_;
    cmd_.brake = cmd_brake_;
    return true;
}

/*
    Desired direction: Forward   err = (+) - (?) 
        1. Reference > Current : (+) acceleration (to forward)
        2. Reference < Current : (-) deceleration (->with brake)
    
    Desired direction: Backward (-) - (?)
        1. Reference > Current : (+) deceleration (->with brake)
        2. Reference < Current : (-) acceleration (to backward)
*/




};
