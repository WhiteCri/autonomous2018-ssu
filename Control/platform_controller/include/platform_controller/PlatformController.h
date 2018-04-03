#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "platform_controller/cmd_platform.h"
#include "platform_rx_msg/platform_rx_msg.h"
#include <geometry_msgs/Twist.h>

/* Platform Protocol 관련 */
#define NO_ACCEL  0
#define MAX_ACCEL 200
#define NO_BRAKE  1
#define MAX_BRAKE 200
#define MAX_STEER 2000
#define GEAR_FORWARD 0
#define GEAR_BACKWARD 2

/* Platform Dynamics 관련 */
#define PI 3.141592
#define RAD2SERIAL (180.0/PI)*100.0 // rad -> serial
#define M_S2SERIAL 1.0/0.0598 // m/s -> serial (@ steady-state)
#define C0 0.1080 // acceleration [m/s^2] = 0.108*exp(0.0184*platform) -> C0 = 0.108
#define C1 0.0184 // acceleration [m/s^2] = 0.108*exp(0.0184*platform) -> C1 = 0.0184
#define C2 0.0598 // steady-state speed [m/s] = 0.0598*platform        -> C2 = 0.0598

/* Debug */
#define MY_DEBUG
#define MY_TEST


class PlatformController {
public:
PlatformController()
    : ref_speed_(0.0), current_speed_(0.0), err_speed_(0.0)
    , ref_steer_(0.0), current_steer_(0.0), err_steer_(0.0)
    , cmd_accel_(0.0), cmd_steer_(0.0), cmd_brake_(0.0)
    , kp_steer_(1.0), ki_steer_(0.0), kd_steer_(0.0)
    , kp_brake_(1.0), ki_brake_(0.0), kd_brake_(0.0)
    , settling_time_(0.5)
    , dt_(0.0)
{
     
}


void Init(int argc, char **argv) // Controller 돌리기 전에 initialize (dt 계산을 위한 time 초기값)
{
    ros::init(argc, argv, "Platform_Controller");
    ros::NodeHandle priv_nh_("~");
    ros::NodeHandle nh_;

    priv_nh_.param<double>("/control/accel/settling_time", settling_time_, 0.5);

    priv_nh_.param<double>("/control/steer/kp", kp_steer_, 1.0);
    priv_nh_.param<double>("/control/steer/ki", ki_steer_, 0.0);
    priv_nh_.param<double>("/control/steer/kd", kd_steer_, 0.0);
    
    priv_nh_.param<double>("/control/brake/kp", kp_brake_, 1.0);
    priv_nh_.param<double>("/control/brake/ki", ki_brake_, 0.0);
    priv_nh_.param<double>("/control/brake/kd", kd_brake_, 0.0);

    pub_ = nh_.advertise<platform_controller::cmd_platform>("control/cmd_platform", 100);
    
    timestamp_ = ros::Time::now();
}



void Calc_PID(void) // read_state, read_reference로 읽은 후에 Platform_TX(SI Unit -> Platform Unit) 하기 전에 호출
{    
    ros::Time now = ros::Time::now();
    dt_ = (now - timestamp_).toSec();
    timestamp_ = now;

    UpdateParameters(); // PID Gain Parameter Update

    Calc_accleration(); // SPEED CONTROL

    Calc_steer(); // STEER CONTROL
}


void RX_Callback(const platform_rx_msg::platform_rx_msg::ConstPtr& rx_data)
{
    current_speed_ = rx_data->speed;
    current_steer_ = rx_data->steer;
}


void Cmd_Callback(const geometry_msgs::TwistConstPtr& twist)
{
    ref_speed_ = twist->linear.x;
    ref_steer_ = RAD2SERIAL*(twist->angular.z);
}


//void Ack_Callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& ack_data)
//{
//    ref_speed_ = ack_data->drive.speed;
//    ref_steer_ = RAD2SERIAL*(ack_data->drive.steering_angle);
//}


void publish()
{
    pub_.publish(cmd_);

    #ifdef MY_TEST // rqt_plot으로 비교하기 위한 용도로 테스트
        cmd_.reference = BoundaryCheck_Steer(ref_steer_);
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

double dt_;

double ref_speed_, current_speed_, err_speed_;
double ref_steer_, current_steer_, err_steer_;
int cmd_accel_, cmd_steer_, cmd_brake_;

double settling_time_;
double kp_steer_, ki_steer_, kd_steer_;
double kp_brake_, ki_brake_, kd_brake_;


inline void UpdateParameters(void)
{
    ros::NodeHandle priv_nh_("~");
    
    priv_nh_.getParam("/control/accel/settling_time", settling_time_);

    priv_nh_.getParam("/control/steer/kp", kp_steer_);
    priv_nh_.getParam("/control/steer/ki", ki_steer_);
    priv_nh_.getParam("/control/steer/kd", kd_steer_);

    priv_nh_.getParam("/control/brake/kp", kp_brake_);
    priv_nh_.getParam("/control/brake/ki", ki_brake_);
    priv_nh_.getParam("/control/brake/kd", kd_brake_);
}


inline int BoundaryCheck_Accel(const int accel)
{
    if(accel <= NO_ACCEL){
        return NO_ACCEL;
    }
    else{
        return (accel <= MAX_ACCEL) ? accel : MAX_ACCEL;
    }    
}

inline int BoundaryCheck_Brake(const int brake)
{
    if(brake <= NO_BRAKE){
        return NO_BRAKE;
    }
    else{
        return (brake <= MAX_BRAKE) ? brake : MAX_BRAKE;
    }
}

inline int BoundaryCheck_Steer(const int steer)
{
    return (fabs(steer) <= MAX_STEER) ? steer : MAX_STEER*(steer/fabs(steer));
}



inline int Calc_gear(double ref_speed)
{
    if(ref_speed >= 0.0){
        cmd_.gear = GEAR_FORWARD;
        return 1;
    }
    else{
        cmd_.gear = GEAR_BACKWARD;
        return -1;
    }
}

void Calc_accleration(void) // 나중에 Brake에 대한 가속도 실험 후 수정 필요(감속부분)
{
    /*
    Desired direction: Forward   err = (+) - (?) 
        1. Reference > Current : (+) acceleration (to forward)
        2. Reference < Current : (-) deceleration (->with brake)

    Desired direction: Backward (-) - (?)
        1. Reference > Current : (+) deceleration (->with brake)
        2. Reference < Current : (-) acceleration (to backward)
    */

//    cmd_.gear = (ref_speed_ >= 0.0) ? GEAR_FORWARD : GEAR_BACKWARD;
//    const int dir = (int)( ref_speed_ / fabs(ref_speed_) );
    
    const int dir = Calc_gear(ref_speed_);
    err_speed_ = ref_speed_ - current_speed_; // error speed 단위: [m/s]
    
    if(err_speed_ * dir > 0.0){ // 가속(Acceleration)
        cmd_accel_ = (int)log( fabs(err_speed_) / (C0 * settling_time_) ) / C1; // command acceleration, 단위: [Platform Unit 0 ~ 200]
        cmd_brake_ = NO_BRAKE; // Brake = 1 : No brake !
    }
    else{ // 감속(Deceleration)
        cmd_accel_ = fabs(ref_speed_) * M_S2SERIAL; // ref_speed가 수렴속도일 때 platform에 넣어야 할 값 계산 (steady-state speed)
        cmd_brake_ = fabs(err_speed_) * (kp_brake_ + ki_brake_*dt_ + kd_brake_/dt_);
    }
    
    cmd_.accel = BoundaryCheck_Accel(cmd_accel_);
    cmd_.brake = BoundaryCheck_Brake(cmd_brake_);
}

void Calc_steer(void)
{
    err_steer_ = BoundaryCheck_Steer(ref_steer_) - current_steer_;
    
    cmd_steer_ = (int)( err_steer_ * (kp_steer_ + ki_steer_*dt_ + kd_steer_/dt_) );

    cmd_.steer = BoundaryCheck_Steer(current_steer_ + cmd_steer_);
}

};