#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#define RAD2DEG 1 / (180.0)*3.141592

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "move_base_client");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base_server", true);

    //get param
    ros::NodeHandle nh;
    std::vector<int> x_goal, y_goal;
    std::vector<double> yaw_goal;

    nh.getParam("/move_base_client/send_goal/x_goal", x_goal);
    nh.getParam("/move_base_client/send_goal/y_goal", y_goal);
    nh.getParam("/move_base_client/send_goal/yaw_goal", yaw_goal);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("send_goal");
    ac.sendGoal(goal);
    //we'll send a goal to the robot to move 1 meter forward

    size_t idx = 0;
    size_t goal_count = x_goal.size();
    ros::Rate loop_rate(10);
    while(true){
        idx++;
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished : %s",state.toString().c_str());
        loop_rate.sleep();
        if(idx == 10) {
            ac.cancelAllGoals();
            ac.sendGoal(goal);
        }
    }
    return 0;
}