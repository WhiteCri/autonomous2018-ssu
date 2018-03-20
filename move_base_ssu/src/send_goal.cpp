#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#define RAD2DEG 1 / (180.0)*3.141592

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //get param
    ros::NodeHandle nh;
    std::vector<int> x_goal, y_goal;
    std::vector<double> yaw_goal;

    nh.getParam("/send_goal/send_goal/x_goal", x_goal);
    nh.getParam("/send_goal/send_goal/y_goal", y_goal);
    nh.getParam("/send_goal/send_goal/yaw_goal", yaw_goal);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward

    size_t idx = 0;
    size_t goal_count = x_goal.size();
    while(idx < goal_count){
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = static_cast<int>(x_goal[idx]);
        goal.target_pose.pose.position.y = static_cast<int>(y_goal[idx]);
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(
            static_cast<double>(yaw_goal[idx])
        );

        ROS_INFO("Sending goal : %d %d %lf", x_goal[idx], y_goal[idx], yaw_goal[idx] / RAD2DEG);
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Hooray, the base moved 1 meter forward");
            idx++;
        }
        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
    return 0;
}