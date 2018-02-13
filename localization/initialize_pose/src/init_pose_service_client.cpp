#include "ros/ros.h"
#include <cstdlib>
#include "initialize_pose/init_pose.h"

int main(int argc, char *argv[]){
  ros::init(argc, argv, "init_pose_service_client");



  ros::NodeHandle nh;

  ros::ServiceClient ros_tutorials_service_client =
    nh.serviceClient<initialize_pose::init_pose>("init_pose_srv");

  initialize_pose::init_pose srv;

  srv.request.Req_Flag = atoll(argv[1]);


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
