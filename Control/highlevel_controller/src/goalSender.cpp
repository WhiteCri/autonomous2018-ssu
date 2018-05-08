#include "highlevel_controller/goalSender.h"
#include "highlevel_controller/base_parameter.h"
#include <thread>

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
    if (goal_type == "crosswalk"){
    
    }
    else if (goal_type == "movingobj"){

    }
    else if (goal_type == "uturn"){
        
    }
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