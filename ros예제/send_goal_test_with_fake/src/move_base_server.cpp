#include <ros/ros.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseServer;

void execute(const move_base_msgs::MoveBaseGoalConstPtr& goal, MoveBaseServer* as)  // Note: "Action" is not appended to DoDishes here
{
  ROS_INFO("start execute");
  
  static size_t cnt = 0;
  cnt++;
  // Do lots of awesome groundbreaking robot stuff here
  ros::Rate rate(0.5);
  rate.sleep();
  rate.sleep();
  
  move_base_msgs::MoveBaseResult result;
  
  as->setSucceeded(result);
  ROS_INFO("succeed : %lf %lf",goal->target_pose.pose.position.x,
      goal->target_pose.pose.position.y);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base");
  ros::NodeHandle n;
ROS_INFO("starting movebase server...");
  MoveBaseServer server(n, "move_base", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}