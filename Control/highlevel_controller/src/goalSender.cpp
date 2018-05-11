#include "highlevel_controller/goalSender.h"
#include "highlevel_controller/base_parameter.h"
#include <thread>
#include <algorithm>
#include <cctype>

extern Parameters* param_ptr;

GoalSender::GoalSender() : ac("move_base", true)
{
    while(!ac.waitForServer(ros::Duration(3.0))){
        ROS_ERROR("Waiting for the move_base action server to come up");
    }
    
    bool use_auto_goal_sender;
    param_ptr->nh.getParam("hl_controller/use_auto_goal_sender",use_auto_goal_sender);
    if(use_auto_goal_sender){
        auto thr = std::thread(&GoalSender::auto_goal_sender, this);
        thr.detach();
    }
}

void GoalSender::sendGoal(){
    ac.cancelAllGoals();
    ac.sendGoal(goal);
    if (goal_type=="crosswalk"){
        if (param_ptr->use_process_crosswalk==false) return;
    }else if (goal_type=="movingobj"){
        if (param_ptr->use_process_movingobj==false) return;
    }else if (goal_type=="uturn"){
        if (param_ptr->use_process_uturn==false) return;
    }else if ((goal_type=="parking_near")||(goal_type=="parking_far")){
        if (param_ptr->use_process_parking==false) return;
    }else if ((goal_type=="nload")){
        if (param_ptr->use_process_nload==false) return;
    }else if ((goal_type=="sload")){
        if (param_ptr->use_process_sload==false) return;
    }
    else return; //if not allowed status, return;

    std::thread tr([&](){
        std::string param_name = "hl_controller/"+goal_type;
        //because there are no function to make uppercase string, I decided to use a STL algorithm function
        std::string upper_statename(goal_type); // just fit length.
        std::transform(goal_type.begin(), goal_type.end(), upper_statename.begin(),
            [](unsigned char c) -> unsigned char { return std::toupper(c);});
        std::string curState, targetState = "PROCESS_" + upper_statename;

        while(true){
ROS_INFO("running stateChanger thread...");
            param_ptr->nh.setParam(param_name.c_str(), true);

ROS_INFO("param_name : %s",param_name.c_str());                
ROS_INFO("targetState : %s",targetState.c_str());
            param_ptr->nh.getParam("hl_controller/curState", curState);
            if (curState == targetState) break;
                ros::Rate(param_ptr->frequency).sleep();
            }
    });
    tr.detach(); 
}

void GoalSender::setGoal(double x, double y, double ori_z, double ori_w, std::string goal_type){
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.z = ori_z;
    goal.target_pose.pose.orientation.w = ori_w;
    this->goal_type = goal_type;
}

GoalSender::GoalStates GoalSender::getState(){
    GoalStates ret;

    auto state = ac.getState();
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        ret = GoalStates::STATE_SUCCEEDED;  
    else if (state == actionlib::SimpleClientGoalState::ACTIVE)
        ret = GoalStates::STATE_ACTIVE;     
    else if (state == actionlib::SimpleClientGoalState::PENDING)
        ret = GoalStates::STATE_PENDING;    
    else if (state == actionlib::SimpleClientGoalState::LOST)
        ret = GoalStates::STATE_LOST;
    else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
        ret = GoalStates::STATE_PREEMPTED;
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
        ret = GoalStates::STATE_ABORTED;
    else {
        ROS_INFO("unhandling move_base server state: %s", state.toString().c_str());
        throw std::runtime_error("unhandling state");
    }

    return ret;
}

void GoalSender::auto_goal_sender(){
    int auto_goal_sender_frequency;

    param_ptr->nh.param("hl_controller/auto_goal_sender_frequency", auto_goal_sender_frequency, 3);
    
    ros::Rate loop_rate(auto_goal_sender_frequency);
    while(true){
        sendGoal();
        loop_rate.sleep();
    }
}

GoalSender* GoalSender::getInstancePtr() {
    if (!objptr) objptr = new GoalSender();
    return objptr;
}

GoalSender* GoalSender::objptr = nullptr;