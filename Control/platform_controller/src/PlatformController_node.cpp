#include "platform_controller/PlatformController.h"
#include <thread>
#include <mutex>


std::mutex lock;
static PlatformController ctrl;
static int rate = 10;

void subscriberaw(){
  ros::NodeHandle nh;
  ros::Subscriber sub_1 = nh.subscribe("/raw/platform_rx", 100, &PlatformController::RX_Callback, &ctrl);
  
  ros::Rate loop_rate(rate);
  while(ros::ok()){
    lock.lock();
    ros::spinOnce();
    lock.unlock();
    loop_rate.sleep();
  }
}


int main(int argc, char **argv){

  ctrl.Init(argc, argv);
  ros::NodeHandle nh;

  //ros::Subscriber sub_2 = nh.subscribe("/ackerman_cmd", 100, &PlatformController::Ack_Callback, &ctrl);
  ros::Subscriber sub_2 = nh.subscribe("/cmd_vel", 100, &PlatformController::Cmd_Callback, &ctrl);
  
  std::thread rawnode(subscriberaw);
  rawnode.detach();

  ros::Rate loop_rate(rate);

  while(ros::ok()){
    ros::spinOnce();
    ctrl.Calc_PID();
    ctrl.publish();
    loop_rate.sleep();
  }    
  return 0;
}
