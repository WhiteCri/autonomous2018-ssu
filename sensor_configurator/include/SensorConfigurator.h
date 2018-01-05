#pragma once
#include <iostream>
#include "ros/ros.h"
#include "SerialCommunicator.h"
#include "IMUParameter.h"//나중에 파라미터용 yaml 파일을 사용할 때, parser를 이용한 자동화 작업이 필요함. 지금은
//yaml파일을 활용할 줄 모르기 때문에 헤더파일로 대체함

/* TODOList for sensor
  1. Serial : GPS/PlatformRX/PlatformTX
  2. Ethernet : Lidar
  3. 정체불명 : Camera 

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
  //Ethernet 연결 함수 추가 필요

  /*
    1. void init()
    센서 초기화에 필요한 작업을 수행합니다. init는 자식 클래스의 생성자에서 호출하는것을 원칙으로 합니다.
    algorithm : 통신객체생성 및 validity 검사->파라미터 이름을 vector에 저장->(필요하다면 기타 초기화 작업 수행)
  */
  virtual void init() = 0;

  /*
    2. void setParameterData(ros::NodleHandle& nh)
    algorithm : parse() 수행 및 정상작동 확인->파라미터 등록
    실질적으로 main에서 호출해, 파라미터를 server에 등록해주는 작업을 수행합니다.
    작업을 위해 객체 별 파라미터의 이름을 저장하는 벡터와 데이터를 저장하는 자료구조를 자식 클래스의 멤버로 선언하세요.
  */
  virtual void setParameterData(ros::NodeHandle& nh) = 0;

  /*
    3. bool ok()
    센서의 데이터가 유효한지 검사해 bool 값을 반환합니다.
    유효하지 않은 경우의 예) Serial Data 짤림 현상(센서의 주파수보다 해당 노드의 주파수가 비슷하거나 낮을 때 발생합니다.
                        GPS의 checksum 검사
    센서별 데이터 시트를 확인해보세요.
  */
  virtual bool ok() = 0;
protected:
  /*
    4. parse()
    algorithm : 통신 객체를 이용한 read() 호출->문자열 데이터 파싱
    센서별 파싱을 수행하는 함수입니다. 데이터를 저장할 때 다음의 약속을 지켜주세요.
    정수 : int32_t
    실수 : float64_t
    문자열 : uint8_t
  */
  virtual bool parse() = 0;
  SerialCommunicator * ser_ptr_;
};
