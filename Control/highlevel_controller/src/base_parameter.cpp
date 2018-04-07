#include "highlevel_controller/base_parameter.h"

Parameters* Parameters::getInstance(ros::NodeHandle& nh){
    if(!obj_ptr) obj_ptr = new Parameters(nh);
    return obj_ptr;
}

Parameters::Parameters(ros::NodeHandle& nh){
        
    /* global parameter */
    nh.param("hl_contoller/frequency", frequency, 5);
    nh.param("hl_controller/publish_param", publish_param, true);

    /* tx control parameter */
    nh.param("hl_controller/tx_stop", tx_stop, false);

    /* crosswalk parameter */
    nh.param("hl_controller/crosswalk", crosswalk, false);
    nh.param("hl_controller/use_process_crosswalk", use_process_crosswalk, true);
    nh.param("hl_controller/use_crosswalk_onetime_flag", use_crosswalk_onetime_flag, true);

    nh.param("hl_controller/crosswalk_driving_duration", crosswalk_driving_duration, 1.0);
    nh.param("hl_controller/crosswalk_stop_duration", crosswalk_stop_duration, 3.0);
    nh.param("hl_controller/crosswalk_onetime_flag", crosswalk_onetime_flag, true);

    /* movingobj paramter */
    nh.param("hl_controller/movingobj", movingobj, false);
    nh.param("hl_controller/use_process_movingobj", use_process_movingobj, true);
    nh.param("hl_controller/use_movingobj_onetime_flag", use_movingobj_onetime_flag, true);

    nh.param("hl_controller/movingobj_driving_duration", movingobj_driving_duration, 1.0);
    nh.param("hl_controller/movingobj_stop_duration", movingobj_stop_duration, 5.0);
    nh.param("hl_controller/movingobj_onetime_flag", movingobj_onetime_flag, true);

    /* parking parameter */
    nh.param("hl_controller/parking", parking, false);
    nh.param("hl_controller/use_process_parking", use_process_parking, true);
    nh.param("hl_controller/use_parking_onetime_flag", use_parking_onetime_flag, true);

    nh.param("hl_controller/parking_stop_duration", parknig_stop_duration, 10.0);
    nh.param("hl_controller/parking_onetime_flag", parking_onetime_flag, true);

    //goalpoint members    
    nh.param("hl_controller/parking_near_arrive_point_x",parking_near_arrive_point_x, -200.0);  
    nh.param("hl_controller/parking_near_arrive_point_y",parking_near_arrive_point_y, -200.0);
    nh.param("hl_controller/parking_far_arrive_point_x",parking_far_arrive_point_x, -200.0);  
    nh.param("hl_controller/parking_far_arrive_point_y",parking_far_arrive_point_y, -200.0);  
    nh.param("hl_controller/parking_near_back_point_x",parking_near_back_point_x, -200.0);
    nh.param("hl_controller/parking_near_back_point_y",parking_near_back_point_y, -200.0);
    nh.param("hl_controller/parking_far_back_point_x",parking_far_back_point_x, -200.0);
    nh.param("hl_controller/parking_far_back_point_y",parking_far_back_point_y, -200.0);

    /* recovery members */
    nh.param("hl_controller/need_recovery", need_recovery, false);
}

//create instance
Parameters* Parameters::obj_ptr = nullptr;