#include "platform_controller/PlatformController.h"
#include <thread>
#include <mutex>


static PlatformController ctrl;
//static int rate = 5;

int main(int argc, char **argv){

  ctrl.Init(argc, argv);
  ros::NodeHandle nh;

  ros::Subscriber sub_1 = nh.subscribe("/raw/platform_rx", 100, &PlatformController::RX_Callback, &ctrl);
  ros::Subscriber sub_2 = nh.subscribe("/cmd_vel", 100, &PlatformController::Cmd_Callback, &ctrl);

  ros::spin();
//  ros::Rate loop_rate(rate);
//
//  while(ros::ok()){
//    ros::spinOnce();
////    ctrl.Calc_PID();
////    ctrl.publish();
//    loop_rate.sleep();
//  }    
//  return 0;
}
