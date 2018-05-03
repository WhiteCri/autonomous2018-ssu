#include "highlevel_controller/base_parameter.h"
#include <exception>
Parameters* Parameters::getInstancePtr(){
    if(!obj_ptr) obj_ptr = new Parameters();
    return obj_ptr;
}

Parameters::Parameters(){
        
    /* global parameter */
    nh.param("hl_contoller/frequency", frequency, 5);
    nh.param("hl_controller/publish_param", publish_param, true);

    /* goal list */
    if (!nh.getParam("hl_controller/x_goal", x_goal)) throw std::runtime_error("no x goal!");
    if (!nh.getParam("hl_controller/y_goal", y_goal)) throw std::runtime_error("no y goal!");
    if (!nh.getParam("hl_controller/ori_z_goal", ori_z_goal)) throw std::runtime_error("no ori_z goal!");
    if (!nh.getParam("hl_controller/ori_w_goal", ori_w_goal)) throw std::runtime_error("no ori_w goal!");
    if (!nh.getParam("hl_controller/goal_type", goal_type)) throw std::runtime_error("no goal_type !");
    //reverse
    x_goal = std::vector<double>(x_goal.rbegin(), x_goal.rend());
    y_goal = std::vector<double>(y_goal.rbegin(), y_goal.rend());
    ori_z_goal = std::vector<double>(ori_z_goal.rbegin(), ori_z_goal.rend());
    ori_w_goal = std::vector<double>(ori_w_goal.rbegin(), ori_w_goal.rend());
    goal_type = std::vector<std::string>(goal_type.rbegin(), goal_type.rend());

    /* tx control parameter */
    nh.param("hl_controller/tx_speed", tx_speed, 0);
    nh.param("hl_controller/tx_steer", tx_steer, 0);
    nh.param("hl_controller/tx_brake", tx_brake, 0);
    nh.param("hl_controller/tx_control_static", tx_control_static, false);

    /* crosswalk parameter */
    nh.param("hl_controller/crosswalk", crosswalk, false);
    nh.param("hl_controller/crosswalk_check_duration", crosswalk_check_duration, 1.0);
    nh.param("hl_controller/use_process_crosswalk", use_process_crosswalk, true);
    nh.param("hl_controller/use_crosswalk_onetime_flag", use_crosswalk_onetime_flag, true);

    nh.param("hl_controller/crosswalk_driving_duration", crosswalk_driving_duration, 1.0);
    nh.param("hl_controller/crosswalk_stop_duration", crosswalk_stop_duration, 3.0);
    nh.param("hl_controller/crosswalk_onetime_flag", crosswalk_onetime_flag, true);

    /* movingobj paramter */
    nh.param("hl_controller/movingobj", movingobj, false);
    nh.param("hl_controller/movingobj_check_duration", movingobj_check_duration, 1.0);
    nh.param("hl_controller/use_process_movingobj", use_process_movingobj, true);
    nh.param("hl_controller/use_movingobj_onetime_flag", use_movingobj_onetime_flag, true);

    nh.param("hl_controller/movingobj_driving_duration", movingobj_driving_duration, 1.0);
    nh.param("hl_controller/movingobj_stop_duration", movingobj_stop_duration, 5.0);
    nh.param("hl_controller/movingobj_onetime_flag", movingobj_onetime_flag, true);

    /* parking parameter */
    nh.param("hl_controller/parking", parking, false);
    nh.param("hl_controller/parking_check_duration", parking_check_duration, 1.0);
    nh.param("hl_controller/use_process_parking", use_process_parking, true);
    nh.param("hl_controller/use_parking_onetime_flag", use_parking_onetime_flag, true);

    nh.param("hl_controller/parking_stop_duration", parking_stop_duration, 10.0);
    nh.param("hl_controller/parking_onetime_flag", parking_onetime_flag, true);

    //goalpoint members    
    nh.param("hl_controller/parking_near_arrive_point_x",       parking_near_arrive_point_x, -200.0);  
    nh.param("hl_controller/parking_near_arrive_point_y",       parking_near_arrive_point_y, -200.0);
    nh.param("hl_controller/parking_near_arrive_point_ori_z",   parking_near_arrive_point_ori_z, 0.0);  
    nh.param("hl_controller/parking_near_arrive_point_ori_w",   parking_near_arrive_point_ori_w, 0.0);
    nh.param("hl_controller/parking_near_back_point_x",         parking_near_back_point_x, -200.0);
    nh.param("hl_controller/parking_near_back_point_y",         parking_near_back_point_y, -200.0);
    nh.param("hl_controller/parking_near_back_point_ori_z",     parking_near_back_point_ori_z, 0.0);
    nh.param("hl_controller/parking_near_back_point_ori_w",     parking_near_back_point_ori_w, 0.0);

    nh.param("hl_controller/parking_far_arrive_point_x",       parking_far_arrive_point_x, -200.0);  
    nh.param("hl_controller/parking_far_arrive_point_y",       parking_far_arrive_point_y, -200.0);
    nh.param("hl_controller/parking_far_arrive_point_ori_z",   parking_far_arrive_point_ori_z, 0.0);  
    nh.param("hl_controller/parking_far_arrive_point_ori_w",   parking_far_arrive_point_ori_w, 0.0);
    nh.param("hl_controller/parking_far_back_point_x",         parking_far_back_point_x, -200.0);
    nh.param("hl_controller/parking_far_back_point_y",         parking_far_back_point_y, -200.0);
    nh.param("hl_controller/parking_far_back_point_ori_z",     parking_far_back_point_ori_z, 0.0);
    nh.param("hl_controller/parking_far_back_point_ori_w",     parking_far_back_point_ori_w, 0.0);

    /* uturn members */
    bool use_process_uturn;
    bool use_uturn_onetime_flag;

    int uturn_tx_speed;
    int uturn_tx_steer;
    int uturn_tx_brake;
    double uturn_duration;
    nh.param("hl_controller/use_process_uturn", use_process_uturn, true);
    nh.param("hl_controller/uturn_check_duration", uturn_check_duration, 1.0);
    nh.param("hl_controller/use_uturn_onetime_flag", use_uturn_onetime_flag, true);

    nh.param("hl_controller/uturn_tx_speed", uturn_tx_speed, 50);
    nh.param("hl_controller/uturn_tx_steer", uturn_tx_steer, 1999);
    nh.param("hl_controller/uturn_tx_brake", uturn_tx_brake, 0);
    nh.param("hl_controller/uturn_duration", uturn_duration, 1.0);

    /* recovery members */
    nh.param("hl_controller/recovery", recovery, false);
    nh.param("hl_controller/use_process_recovery", use_process_recovery, true);
    nh.param("hl_controller/recovery_check_duration", recovery_check_duration, 0.5);
    

    /* Done members */
    nh.param("hl_controller/reached_goal", reached_goal, false);
}

Parameters* Parameters::obj_ptr = nullptr;