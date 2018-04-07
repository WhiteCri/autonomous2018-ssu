#pragma once
#include <ros/ros.h>
#include <highlevel_controller/params.h>

class Parameters{
public:
    
    /* global parameter */
    int frequency;
    bool publish_param;

    /* tx control parameter */
    bool tx_stop;

    /* crosswalk parameter */
    bool crosswalk;
    bool use_process_crosswalk;
    bool use_crosswalk_onetime_flag;

    double crosswalk_driving_duration;
    double crosswalk_stop_duration;
    bool crosswalk_onetime_flag;

    /* movingobj paramter */
    bool movingobj;
    bool use_process_movingobj;
    bool use_movingobj_onetime_flag;

    double movingobj_driving_duration;
    double movingobj_stop_duration;
    bool movingobj_onetime_flag;

    /* parking parameter */
    bool parking;
    bool use_process_parking;
    bool use_parking_onetime_flag;

    double parknig_stop_duration;
    bool parking_onetime_flag;

    //goalpoint members    
    double parking_near_arrive_point_x;
    double parking_near_arrive_point_y;
    double parking_far_arrive_point_x;
    double parking_far_arrive_point_y;
    double parking_near_back_point_x;
    double parking_near_back_point_y;
    double parking_far_back_point_x;
    double parking_far_back_point_y;

    /* recovery members */
    bool need_recovery;

    //explicit inline declaration
    inline void load_param(ros::NodeHandle& nh){
        nh.getParam("hl_controller/crosswalk",              crosswalk);
        nh.getParam("hl_controller/crosswalk_onetime_flag", crosswalk_onetime_flag);
        nh.getParam("hl_controller/movingobj",              movingobj);
        nh.getParam("hl_controller/movingobj_onetime_flag", movingobj_onetime_flag);
        nh.getParam("hl_controller/parking",                parking);
        nh.getParam("hl_controller/parking_onetime_flag",   parking_onetime_flag);
        nh.getParam("hl_controller/need_recovery", need_recovery);
        
        if (publish_param){
            static size_t seq = 0;
            static ros::Publisher param_pub =
                nh.advertise<highlevel_controller::params>("hl_controller_debug", 100);
            highlevel_controller::params msg;

            msg.stamp = ros::Time::now();
            msg.seq = seq++;
            msg.crosswalk = crosswalk;
            msg.movingobj = movingobj;
            msg.parking = parking;
            msg.need_recovery = need_recovery;

            param_pub.publish(msg);
        }
    }

    //singletone
    static Parameters* getInstance(ros::NodeHandle& nh);
private:
    Parameters(ros::NodeHandle& nh);
    static Parameters* obj_ptr;
};