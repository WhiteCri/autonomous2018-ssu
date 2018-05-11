#include "highlevel_controller/StateHandler.h"
#include "highlevel_controller/base_parameter.h"
#include "highlevel_controller/goalSender.h"
#include <ros/ros.h>

#define ABORT_FLAG TRUE
#define TX_STOP_BRAKE 75

extern Parameters* param_ptr;
extern GoalSender* goalSender_ptr;

inline void showGoal(double x, double y, double ori_z, double ori_w, const std::string& str){
    ROS_INFO("set new goal : %lf, %lf, %lf, %lf, %s",
        x, y, ori_z, ori_w, str.c_str());
}

bool handler_setGoal(bool newGoal=false){
    //if new goal, pop
    if (newGoal){
        param_ptr->x_goal.pop_back();
        param_ptr->y_goal.pop_back();
        param_ptr->ori_z_goal.pop_back();
        param_ptr->ori_w_goal.pop_back();
        param_ptr->goal_type.pop_back();
    }
    //if any element lack, I assumes all goal has been done
    if ((param_ptr->x_goal.size()==0)||(param_ptr->y_goal.size()==0)||(param_ptr->ori_z_goal.size()==0)
            ||(param_ptr->ori_w_goal.size()==0)||(param_ptr->goal_type.size()==0)) return false;
    
    //if type is skip, pass the goal
    while (true){
        std::string goal_type = param_ptr->goal_type.back();
        if (goal_type != "skip") break;
        else {
            param_ptr->x_goal.pop_back();
            param_ptr->y_goal.pop_back();
            param_ptr->ori_z_goal.pop_back();
            param_ptr->ori_w_goal.pop_back();
            param_ptr->goal_type.pop_back();
        }
    }
    double x = param_ptr->x_goal.back();
    double y = param_ptr->y_goal.back();
    double ori_z = param_ptr->ori_z_goal.back();
    double ori_w = param_ptr->ori_w_goal.back();
    std::string goal_type = param_ptr->goal_type.back();

    goalSender_ptr->setGoal(x, y, ori_z, ori_w, goal_type);
    goalSender_ptr->sendGoal();
    showGoal(x, y, ori_z, ori_w, goal_type);

    return true;
}

void init(){
    ROS_INFO("State Init...");
    param_ptr->nh.setParam("hl_controller/curState","INIT");

    handler_setGoal();
}

void toward_goal(){
    typedef GoalSender::GoalStates GoalStates;
    param_ptr->nh.setParam("hl_controller/tx_control_static", false);
    param_ptr->nh.setParam("hl_controller/curState","TOWARD_GOAL");
    
    auto state = goalSender_ptr->getState();
    if (state == GoalStates::STATE_SUCCEEDED){
        ROS_INFO("SUCCEEDED...");
        if (handler_setGoal(true) == false){
            ROS_INFO("reached all goals!");
            param_ptr->nh.setParam("hl_controller/reached_goal", true);
            return;
        }
        ROS_INFO("success end!");
    }
    else if (state == GoalStates::STATE_LOST){
        ROS_INFO("LOST...");
        handler_setGoal();
    }
    else if (state == GoalStates::STATE_ACTIVE){}
    else if (state == GoalStates::STATE_PENDING) {
        ROS_INFO("PENDING...");
    }
    else if (state == GoalStates::STATE_PREEMPTED){
        ROS_INFO("PREEMPTED...Are you using rviz to set goal?");
        handler_setGoal();
    }
    else if (state == GoalStates::STATE_ABORTED){
#ifdef ABORT_FLAG
        ROS_ERROR("ABORTED... IF YOU WANT TO KILL THE PROGRAM, SET THE ABORT FLAG");
#elif 
        ROS_ERROR("ABORTED... exit the program");
        exit(-1);
#endif
    }
    else {
        ROS_INFO("why control reaches here...");
    } 
}

void process_crosswalk(){
    ROS_INFO("crosswalk start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_CROSSWALK");

    //maintaining car's status
    double crosswalk_driving_duration = param_ptr->crosswalk_driving_duration;
    if (crosswalk_driving_duration > 0){
        ROS_INFO("maintaining it's status for %lf seconds...", param_ptr->crosswalk_driving_duration);
        ros::Rate(param_ptr->crosswalk_driving_duration).sleep();
    }

    //take car to stop
    ROS_INFO("stop for %lf seconds...", param_ptr->crosswalk_stop_duration);
    param_ptr->nh.setParam("hl_controller/tx_control_static", true);
    param_ptr->nh.setParam("hl_controller/tx_speed", 0);
    param_ptr->nh.setParam("hl_controller/tx_steer", 0);
    param_ptr->nh.setParam("hl_controller/tx_brake", TX_STOP_BRAKE);

    //take car to wait
    ros::Rate(1 / param_ptr->crosswalk_stop_duration).sleep();

    //unlock tx_control_static
    param_ptr->nh.setParam("hl_controller/tx_control_static",false);
    
    //check that process_crosswalk had been done.
    param_ptr->nh.setParam("hl_controller/crosswalk_onetime_flag",true);

    ROS_INFO("crosswalk done");    
    param_ptr->load_param();
}

void process_movingobj(){
    ROS_INFO("movingobj start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_MOVINGOBJ");

    //maintaining car's status
    double movingobj_driving_duration = param_ptr->movingobj_driving_duration;
    if (movingobj_driving_duration > 0){
        ROS_INFO("maintaining it's status for %lf seconds...", param_ptr->movingobj_driving_duration);
        ros::Rate(param_ptr->movingobj_driving_duration).sleep();
    }
    //take car to stop
    ROS_INFO("stop...");
    param_ptr->nh.setParam("hl_controller/tx_control_static", true);
    param_ptr->nh.setParam("hl_controller/tx_speed", 0);
    param_ptr->nh.setParam("hl_controller/tx_steer", 0);
    param_ptr->nh.setParam("hl_controller/tx_brake", TX_STOP_BRAKE);

    //take car to wait
    ros::Rate(1 / param_ptr->movingobj_stop_duration).sleep();

    //unlock tx_control_static
    param_ptr->nh.setParam("hl_controller/tx_control_static",false);

    //check that process_movingobj had been done.
    param_ptr->nh.setParam("hl_controller/movingobj_onetime_flag",true);

    while(param_ptr->movingobj){
        param_ptr->nh.getParam("hl_controller/movingobj", param_ptr->movingobj);
        ROS_INFO("wait for hl_controller/movingobj to be false...");
        ros::Rate(1).sleep();
    }
    ROS_INFO("movingobj done");
    param_ptr->load_param();
}

void process_parking(){
    ROS_INFO("process parking start");

    ROS_INFO("finding parking point...");
    double parking_point_x;
    double parking_point_y;
    double parking_point_ori_z;
    double parking_point_ori_w;
    double backing_point_x;
    double backing_point_y;
    double backing_point_ori_z;
    double backing_point_ori_w;
    /* 
        I haven't find the solution to select the goal. So temporarily, just select the near goal.
        this code should be modified until the contest
    */
    if (param_ptr->parking_near){
        parking_point_x      = param_ptr->parking_near_arrive_point_x;
        parking_point_y      = param_ptr->parking_near_arrive_point_y;
        parking_point_ori_z  = param_ptr->parking_near_arrive_point_ori_z;
        parking_point_ori_w  = param_ptr->parking_near_arrive_point_ori_w;
        backing_point_x      = param_ptr->parking_near_back_point_x;
        backing_point_y      = param_ptr->parking_near_back_point_y;
        backing_point_ori_z  = param_ptr->parking_near_back_point_ori_z;
        backing_point_ori_w  = param_ptr->parking_near_back_point_ori_w;
        ROS_INFO("set near point as a goal...");
        param_ptr->nh.setParam("hl_controller/curState","PROCESS_PARKING_NEAR");
    } else{
        parking_point_x      = param_ptr->parking_far_arrive_point_x;
        parking_point_y      = param_ptr->parking_far_arrive_point_y;
        parking_point_ori_z  = param_ptr->parking_far_back_point_ori_z;
        parking_point_ori_w  = param_ptr->parking_far_back_point_ori_w;
        backing_point_x      = param_ptr->parking_far_back_point_x;
        backing_point_y      = param_ptr->parking_far_back_point_y;
        backing_point_ori_z  = param_ptr->parking_far_back_point_ori_z;
        backing_point_ori_w  = param_ptr->parking_far_back_point_ori_w;
        ROS_INFO("set far point as a goal...");
        param_ptr->nh.setParam("hl_controller/curState","PROCESS_PARKING_NEAR");
    }
    //set goal to parking point
    ROS_INFO("to the parking point...");
    goalSender_ptr->setGoal(
        parking_point_x,
        parking_point_y,
        parking_point_ori_z,
        parking_point_ori_w,
        "parking"
    );
    goalSender_ptr->sendGoal();

    //wait until arrive.
    while(true){
        auto state = goalSender_ptr->getState();
        if (state == GoalSender::GoalStates::STATE_SUCCEEDED) break;
        ros::Rate(param_ptr->frequency).sleep();
        ROS_INFO("parking point has not been arrived...");
    }
    ROS_INFO("Arrived parking point...");

    //take car to stop
    ROS_INFO("stop...");
    param_ptr->nh.setParam("hl_controller/tx_control_static", true);
    param_ptr->nh.setParam("hl_controller/tx_speed", 0);
    param_ptr->nh.setParam("hl_controller/tx_steer", 0);
    param_ptr->nh.setParam("hl_controller/tx_brake", TX_STOP_BRAKE);

    //take car to wait
    ros::Rate(1 / param_ptr->parking_stop_duration).sleep();

    //set goal to backing point
    ROS_INFO("set tx_stop false");
    param_ptr->nh.setParam("hl_controller/tx_control_static", false);

    ROS_INFO("to the backing point...");
    goalSender_ptr->setGoal(
        backing_point_x,
        backing_point_y,
        backing_point_ori_z,
        backing_point_ori_w,
        "parking"
    );
    goalSender_ptr->sendGoal();

    //wait until arrive.
    while(true){
        auto state = goalSender_ptr->getState();
        if (state == GoalSender::GoalStates::STATE_SUCCEEDED) break;
        ros::Rate(param_ptr->frequency).sleep();
        ROS_INFO("backing point has not been arrived...");
    }
    ROS_INFO("Arrived backing point...");

    param_ptr->nh.setParam("hl_controller/parking_onetime_flag",true);
}

void process_uturn(){
    ROS_INFO("uturn start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_UTURN");

    double uturn_duration = param_ptr->uturn_duration;

    //set tx_control_static
    ROS_INFO("set tx_control value - speed : %d, steer : %d, brake : %d", 
        param_ptr->uturn_tx_speed, param_ptr->uturn_tx_steer, param_ptr->uturn_tx_brake);
    param_ptr->nh.setParam("hl_controller/tx_control_static", true);
    param_ptr->nh.setParam("hl_controller/tx_speed", param_ptr->uturn_tx_speed);
    param_ptr->nh.setParam("hl_controller/tx_steer", param_ptr->uturn_tx_steer);
    param_ptr->nh.setParam("hl_controller/tx_brake", param_ptr->uturn_tx_brake);

    //wait until duration ends
    ROS_INFO("maintaing for %lf seconds...", param_ptr->uturn_duration);
    ros::Rate(1/param_ptr->uturn_duration).sleep();

    //erase tx_control_static flag
    param_ptr->nh.setParam("hl_controller/tx_control_static", false);

    //check that process_movingobj had been done.
    param_ptr->nh.setParam("hl_controller/uturn_onetime_flag",true);

    ROS_INFO("uturn done");
}

void process_sload(){
    ROS_INFO("sload start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_SLOAD");
}

void process_nload(){
    ROS_INFO("nlaod start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_NLOAD");
}

void process_recovery(){
    param_ptr->nh.setParam("hl_controller/recovery",false);
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_RECOVERY");
}

void done(){
    param_ptr->nh.setParam("hl_controller/curState","DONE");
    ROS_INFO("ALL GOAL had been processed");
}