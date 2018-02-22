#include "ros/ros.h"
#include <cstdlib>
#include "initialize_pose/init_pose.h"
//#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/PoseWithCovarianceStamped.h"
//
//
//static geometry_msgs::PoseWithCovarianceStamped init_pose;
//
//void SubCallback(nav_msgs::Odometry& odom){
//    init_pose.header.frame_id = "base_link"; // initial pose의 Frame ID를 지정해야 함 (일단 base_link로 해놨는데 나중에 필요한걸로 수정 필요)
//    init_pose.header.stamp = ros::Time::now();
//    init_pose.pose = odom.pose;
//}
//

int main(int argc, char *argv[]){
  ros::init(argc, argv, "init_pose_service_client");



  ros::NodeHandle nh;
  

  ros::ServiceClient ros_tutorials_service_client =
  nh.serviceClient<initialize_pose::init_pose>("init_pose_srv");
 

  initialize_pose::init_pose srv;
  //ros::Subscriber sub = nh.subscribe("vo", 100, SubCallback);
  srv.request.Req_Flag = atoll(argv[1]);
 
  ros::spinOnce();

  if(ros_tutorials_service_client.call(srv)){
    ROS_INFO("send srv, srv.Request.a and b : %ld",static_cast<long int>(srv.request.Req_Flag));
    ROS_INFO("receive srv, srv.Response.result: %ld", (long int)srv.response.Res_Flag);
  }
  else{
    ROS_ERROR("Faild to call service ros_tutorial_srv");
    return 1;
  }
  return 0;
}
