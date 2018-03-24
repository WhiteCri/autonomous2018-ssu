#include "platform_controller/PlatformController.h"
#include <thread>
#include <mutex>

std::mutex lock;
static PlatformController ctrl;

void subscriberaw()
{
  ros::NodeHandle nh;
  ros::Subscriber sub_1 = nh.subscribe("/raw/platform_rx", 100, &PlatformController::RX_Callback, &ctrl);
  while(ros::ok())
  {
    lock.lock();
    ros::spinOnce();
    lock.unlock();
  }
}


int main(int argc, char **argv){
//  PlatformController ctrl;
  ros::NodeHandle nh;
  ctrl.Init(argc, argv);

//  ros::Subscriber sub_1 = nh.subscribe("/raw/platform_rx", 100, &PlatformController::RX_Callback, &ctrl);
  ros::Subscriber sub_2 = nh.subscribe("/ackermann_cmd", 100, &PlatformController::Ack_Callback, &ctrl);

  std::thread rawnode(subscriberaw);
  rawnode.detach();

  ros::Rate loop_rate(loop_rate);

  while(ros::ok()){
    ros::spinOnce();
    ctrl.Calc_PID();
    ctrl.publish();
    loop_rate.sleep();
  }    
  return 0;
}
