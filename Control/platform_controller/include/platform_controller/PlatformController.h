#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "platform_controller/cmd_platform.h"
#include "platform_rx_msg/platform_rx_msg.h"

#define PI 3.141592
#define RAD2SERIAL (180.0 / PI) * 100.0      // rad ->[ 86%] Built target robot_localization deg -> serial
#define M_S2SERIAL (3600.0 / 1000.0) * 10.0  // m/s -> km/h -> serial

#define C0 0.1080 // acceleration [m/s^2] = 0.108*exp(0.0184*platform) -> C0 = 0.108
#define C1 0.0184 // acceleration [m/s^2] = 0.108*exp(0.0184*platform) -> C1 = 0.0184
#define C2 0.0598 // steady-state speed [m/s] = 0.0598*platform        -> C2 = 0.0598


class PlatformController {
public:
PlatformController()
    : ref_speed_(0.0), current_speed_(0.0), err_speed_(0.0)
    , ref_steer_(0.0), current_steer_(0.0), err_steer_(0.0)
    , cmd_accel_(0.0), cmd_steer_(0.0), cmd_brake_(0.0)
    , kp_steer_(0.0), ki_steer_(0.0), kd_steer_(0.0)
    , kp_brake_(0.0), ki_brake_(0.0), kd_brake_(0.0)
    , settling_time_(1.0)
{
    priv_nh_.param<double>("/control/accel/settling_time", settling_time_, 0.0);

    priv_nh_.param<double>("/control/steer/kp", kp_steer_, 0.0);
    priv_nh_.param<double>("/control/steer/ki", ki_steer_, 0.0);
    priv_nh_.param<double>("/control/steer/kd", kd_steer_, 0.0);
    
    priv_nh_.param<double>("/control/brake/kp", kp_brake_, 0.0);
    priv_nh_.param<double>("/control/brake/ki", ki_brake_, 0.0);
    priv_nh_.param<double>("/control/brake/kd", kd_brake_, 0.0);     
}

void Init(int argc, char **argv) // Controller 돌리기 전에 initialize (dt 계산을 위한 time 초기값)
{
    ros::init(argc, argv, "Platform_Controller");
    
    pub_ = nh_.advertise<platform_controller::cmd_platform>("control/cmd_platform", 100);
    
    timestamp_ = ros::Time::now();
}

void Read_State(double speed, double steer) // "Platform_RX 받는 부분에 사용 (Platform Unit-> SI Unit으로 변환해서 넣기)
{
    current_speed_ = speed;
    current_steer_ = steer;
}

void Read_Reference(double speed, double steer) // "ackermann_cmd 받는 부분에 사용 (Platform Unit-> SI Unit으로 변환해서 넣기)
{
    ref_speed_ = speed;
    ref_steer_ = steer;
}

void UpdateParameters(void){
     priv_nh_.getParam("/control/accel/settling_time", settling_time_);

     priv_nh_.getParam("/control/steer/kp", kp_steer_);
     priv_nh_.getParam("/control/steer/ki", ki_steer_);
     priv_nh_.getParam("/control/steer/kd", kd_steer_);
     
     priv_nh_.getParam("/control/brake/kp", kp_brake_);
     priv_nh_.getParam("/control/brake/ki", ki_brake_);
     priv_nh_.getParam("/control/brake/kd", kd_brake_);
}

void Calc_PID(void) // read_state, read_reference로 읽은 후에 Platform_TX(SI Unit -> Platform Unit) 하기 전에 호출
{    
    ros::Time now = ros::Time::now();
    const double dt = (now - timestamp_).toSec();
    timestamp_ = now;

    UpdateParameters(); // PID Gain Parameter Update
    
    // SPEED CONTROL
    err_speed_ = ref_speed_ - current_speed_;
    cmd_accel_ = (int)log( fabs(err_speed_) / (C0 * settling_time_) ) / C1; // command acceleration [Platform Unit 0 ~ 200]
    if(err_speed_ >= 0.0){
        cmd_accel_ = (cmd_accel_ <= 200) ? cmd_accel_ : 200; // Boundary Check
        cmd_brake_ = 1; // Brake = 1 : No brake !
    }
    else{
        cmd_accel_ = ref_speed_ / C2; // ref_speed가 수렴속도일 때 platform에 넣어야 할 값 계산
        cmd_brake_ = fabs(err_speed_) * (kp_brake_ + ki_brake_*dt + kd_brake_/dt);
    }

    // STEER CONTROL
    err_steer_ = (RAD2SERIAL)*ref_steer_ - current_steer_;
    cmd_steer_ = (int)( err_steer_ * (kp_steer_ + ki_steer_*dt + kd_steer_/dt) );

    cmd_.accel = cmd_accel_;
    cmd_.brake = cmd_brake_;
    cmd_.steer = cmd_steer_;
}

void RX_Callback(const platform_rx_msg::platform_rx_msg::ConstPtr& rx_data)
{
    current_speed_ = rx_data->speed;
    current_steer_ = rx_data->steer;
}

void Ack_Callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& ack_data)
{
    ref_speed_ = ack_data->drive.speed;
    ref_steer_ = ack_data->drive.steering_angle;
}

void publish(){
    pub_.publish(cmd_);
}

private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Publisher pub_;
    ros::Time timestamp_;

    platform_controller::cmd_platform cmd_;

    double ref_speed_, current_speed_, err_speed_;
    double ref_steer_, current_steer_, err_steer_;
    int cmd_accel_, cmd_steer_, cmd_brake_;

    double settling_time_;
    double kp_steer_, ki_steer_, kd_steer_;
    double kp_brake_, ki_brake_, kd_brake_;
};