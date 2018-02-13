#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "initialize_pose/init_pose.h"

static geometry_msgs::PoseWithCovarianceStamped init_pose;



void SubCallback(nav_msgs::Odometry& odom){
    init_pose.header.frame_id = "base_link"; // initial pose의 Frame ID를 지정해야 함 (일단 base_link로 해놨는데 나중에 필요한걸로 수정 필요)
    init_pose.header.stamp = ros::Time::now();
    init_pose.pose = odom.pose;
}

bool SrvCallback(initialize_pose::init_pose::Request &req, initialize_pose::init_pose::Response &res){

        ros::NodeHandle Sub_NH;
        ros::Publisher pub = Sub_NH.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",100);
        
        if(req.Req_Flag == 1){
        ROS_INFO("Flag Set!");
        //ros::Subscriber sub = Sub_NH.subscribe("vo", 100, SubCallback);
        //pub.publish(init_pose);
        //ros::spinOnce();
        ROS_INFO("Robot Pose Initialized!");

        res.Res_Flag = 1;
        
    }
    else{
        ROS_INFO("Flag should be set to '1' for initializing");
        res.Res_Flag = 0;
    }


    return true;
  }

int main(int argc, char *argv[]){
  ros::init(argc, argv, "init_pose_service_server");
  ros::NodeHandle nh_srv;

  ros::ServiceServer ros_tutorials_service_server = nh_srv.advertiseService("init_pose_srv", SrvCallback);
  
  ROS_INFO("ready srv server!");

  ros::Rate r(1);
  
  while(1){
    
  
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
