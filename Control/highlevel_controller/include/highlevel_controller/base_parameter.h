#pragma once
#include <ros/ros.h>
#include <highlevel_controller/params.h>
#include "HybridAutomata.h"

enum {
    INIT = 0,
    TOWARD_GOAL,
    PROCESS_CROSSWALK,
    PROCESS_MOVINGOBJ,
    PROCESS_PARKING,
    PROCESS_UTURN,
    PROCESS_RECOVERY,
    DONE
};

const std::string MOVING_STATUES[] = {
    "normal",
    "crosswalk",
    "movingobj",
    "parking",
    "b"
};


// when you add new param, you should config base_parameter.h, base_parameter.cpp, hl_controller.yaml
class Parameters{
public:
    
    /* global parameter */
    int frequency;
    bool publish_param;

    /* goal list */
    std::vector<double> x_goal, y_goal, ori_z_goal, ori_w_goal;
    std::vector<std::string> goal_type;

    /* tx control parameter */
    bool tx_control_static;
    int tx_speed;
    int tx_steer;
    int tx_brake;

    /* crosswalk parameter */
    bool crosswalk;
    bool use_process_crosswalk;
    bool use_crosswalk_onetime_flag;

    double crosswalk_check_duration;
    double crosswalk_driving_duration;
    double crosswalk_stop_duration;
    bool crosswalk_onetime_flag;

    /* movingobj paramter */
    bool movingobj;
    bool use_process_movingobj;
    bool use_movingobj_onetime_flag;

    double movingobj_check_duration;
    double movingobj_driving_duration;
    double movingobj_stop_duration;
    bool movingobj_onetime_flag;

    /* parking parameter */
    bool parking_near;
    bool parking_far;
    bool use_process_parking;
    bool use_parking_onetime_flag;

    double parking_check_duration;
    double parking_stop_duration;
    bool parking_onetime_flag;

    //goalpoint members    
    double parking_near_arrive_point_x;
    double parking_near_arrive_point_y;
    double parking_near_arrive_point_ori_z;
    double parking_near_arrive_point_ori_w;
    double parking_near_back_point_x;
    double parking_near_back_point_y;
    double parking_near_back_point_ori_z;
    double parking_near_back_point_ori_w;
    
    double parking_far_arrive_point_x;
    double parking_far_arrive_point_y;
    double parking_far_arrive_point_ori_z;
    double parking_far_arrive_point_ori_w;
    double parking_far_back_point_x;
    double parking_far_back_point_y;
    double parking_far_back_point_ori_z;
    double parking_far_back_point_ori_w;

    /* uturn members */
    bool uturn;
    bool use_process_uturn;
    bool use_uturn_onetime_flag;

    double uturn_check_duration;
    bool uturn_onetime_flag;

    //tx control members
    int uturn_tx_speed;
    int uturn_tx_steer;
    int uturn_tx_brake;
    double uturn_duration;

    /* recovery members */
    bool recovery;
    bool use_process_recovery;

    double recovery_check_duration;

    /* Done members */
    bool reached_goal;

    /* HybridAutomata */
    HybridAutomata *HA;

    //nodehandle
    ros::NodeHandle nh;

    //explicit inline declaration
    inline void load_param(){
        nh.getParam("hl_controller/crosswalk",              crosswalk);
        nh.getParam("hl_controller/crosswalk_onetime_flag", crosswalk_onetime_flag);
        nh.getParam("hl_controller/movingobj",              movingobj);
        nh.getParam("hl_controller/movingobj_onetime_flag", movingobj_onetime_flag);
        nh.getParam("hl_controller/parking_near",           parking_near);
        nh.getParam("hl_controller/parking_far",            parking_far);
        nh.getParam("hl_controller/parking_onetime_flag",   parking_onetime_flag);
        nh.getParam("hl_controller/uturn",                  uturn);
        nh.getParam("hl_controller/uturn_onetime_flag",     uturn_onetime_flag);
        nh.getParam("hl_controller/recovery",               recovery);
        nh.getParam("hl_controller/reached_goal",           reached_goal);
        
        if (publish_param){
            static size_t seq = 0;
            static ros::Publisher param_pub =
                nh.advertise<highlevel_controller::params>("hl_controller_debug", 100);
            highlevel_controller::params msg;

            msg.stamp = ros::Time::now();
            msg.seq = seq++;
            msg.crosswalk = crosswalk;
            msg.movingobj = movingobj;
            msg.parking_near = parking_near;
            msg.parking_far = parking_far;
            msg.recovery = recovery;
            msg.uturn = uturn;

            param_pub.publish(msg);
        }
    }

    void setHA(HybridAutomata* HA_ptr);

    //singletone
    static Parameters* getInstancePtr();
private:
    Parameters();
    static Parameters* obj_ptr;
};