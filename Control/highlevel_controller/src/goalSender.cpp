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
    ROS_INFO("send goal(x: %lf, y: %lf)",
        goal.target_pose.pose.position.x,
        goal.target_pose.pose.position.y
    );
}

void GoalSender::setGoal(double x, double y, double yaw){
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    ROS_INFO("set goal(x: %lf, y: %lf)",
        goal.target_pose.pose.position.x,
        goal.target_pose.pose.position.y
    );
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
    else {
        ROS_INFO("unhandling move_base server state: %s", state.toString().c_str());
        ret = GoalStates::STATE_PENDING;
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