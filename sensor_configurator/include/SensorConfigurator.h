#pragma once
#include <iostream>
#include "ros/ros.h"
#include "SerialCommunicator.h"
#include "IMUParameter.h"//나중에 파라미터용 yaml 파일을 사용할 때, parser를 이용한 자동화 작업이 필요함. 지금은
//yaml파일을 활용할 줄 모르기 때문에 헤더파일로 대체함

/* Interface of each sensor */
class SensorConfigurator{
public:
  //constructor
  SensorConfigurator();

  //Destructor
  virtual ~SensorConfigurator();

  /* protocol set */
  //serial
  void setSerial(const std::string& path, int baudrate = 9600, int timeout = 1000);

  //child method
  virtual void init() = 0;
  virtual void setParameterData(ros::NodeHandle& nh) = 0;
  virtual bool ok() = 0;
protected:

  //return if valid parsing processed
  virtual bool parse() = 0;
  SerialCommunicator * ser_ptr_;
};
