#include "ros/ros.h"
#include "ros_tutorials_service/SrvTutorial.h"

#define PLUS            1
#define MINUS           2
#define MULTIPLICATION  3
#define DIVISION        4

long int plus(long int a, long int b){return a + b;}
long int minus(long int a, long int b){return a - b;}
long int mul(long int a, long int b){return a * b;}
long int div(long int a, long int b){ (b == 0) ? 0 : (a/b);}

int g_operator = PLUS;

bool calculation(ros_tutorials_service::SrvTutorial::Request &req,
  ros_tutorials_service::SrvTutorial::Response &res){
    typedef int64_t(*func_t)(int64_t, int64_t);//함수 포인터 재정의
    func_t funcAry[] = {plus,minus,mul,div};//함수포인터 배열 선언
    res.result = funcAry[g_operator - 1](req.a, req.b);

    ROS_INFO("request: x=%ld, y=%ld",static_cast<long int>(req.a), static_cast<long int>(req.b));
    ROS_INFO("sending back response : %ld",static_cast<long int>(res.result));

    return true;
  }

int main(int argc, char *argv[]){
  ros::init(argc, argv, "service_server");
  ros::NodeHandle nh;

  //param
  nh.setParam("calculation_method", PLUS);
  //plus는 초기값
  //

  ros::ServiceServer ros_tutorials_service_server = nh.advertiseService("ros_tutorial_srv",
calculation);

  ROS_INFO("ready srv server!");

  //service 예제임
  //ros::spin();
  //
  ros::Rate r(1);
  //param 예제임
  while(1){
    nh.getParam("calculation_method", g_operator);
    ROS_INFO("calculation_method %d",g_operator);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
