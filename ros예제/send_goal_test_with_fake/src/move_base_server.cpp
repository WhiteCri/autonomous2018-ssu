#include <ros/ros.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseServer;

void execute(const move_base_msgs::MoveBaseGoalConstPtr& goal, MoveBaseServer* as)  // Note: "Action" is not appended to DoDishes here
{
  static size_t cnt = 0;
  cnt++;
  // Do lots of awesome groundbreaking robot stuff here
  ros::Rate rate(1);
  rate.sleep();
  if(cnt % 3 == 0){
    as->setSucceeded();
  ROS_INFO("succeed : %lf %lf",goal->target_pose.pose.position.x,
      goal->target_pose.pose.position.y);
  } else as->setPreempted();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_server");
  ros::NodeHandle n;
ROS_INFO("starting movebase server...");
  MoveBaseServer server(n, "move_base", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}