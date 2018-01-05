#include <sstream>
#include "IMUConfigurator.h"

IMUConfigurator::IMUConfigurator(){
  init();
}
void IMUConfigurator::init(){
  setSerial(IMU_PATH);

  //initalize params
  std::string params[] = {
    IMU_PARAM1,
    IMU_PARAM2,
    IMU_PARAM3,
    IMU_PARAM4,
    IMU_PARAM5,
    IMU_PARAM6
  };
  for(auto& param : params)
    paramName.push_back(param);
}
void IMUConfigurator::setParameterData(ros::NodeHandle& nh){
  if(parse() == false) return;

  //set params
  for(size_t i = 0 ; i < 6;++i){
    ROS_INFO("%d",i);
    nh.setParam(paramName[i], imuData[i]);
  }
}
bool IMUConfigurator::parse(){
  //check serial
  //  if(!ser_ptr_->isValid()) ROS_INFO("what the fuck : %s",ser_ptr_->getErrorMsg().c_str());
  std::string parsingData = ser_ptr_->read();
  ROS_INFO("daf :%s",parsingData.c_str());
  std::stringstream ss(parsingData);
  //data format : *-22.29,-31.83,32.33,0.011,0.007,-0.019

  /*
    algorithm :
    1. remove first *-
    2. tokenize using std::getline
    3. save into vector<double>
  */

  //1.
  std::string trash;
  std::getline(ss,trash, '-');

  //2,3
  std::string dataString;

  //if exception occurs, communicator's data is invalid.
  imuData.clear();
  try{
    for(size_t i = 0 ; i < paramName.size();++i){
      getline(ss,dataString,',');
      double data = std::stod(dataString); // stod : string to double
      imuData.push_back(data);
      ROS_INFO("%d : %lf",i ,data);
    }
  }
  catch(std::exception& e){
    ROS_INFO("error : %s", e.what());
    ROS_INFO("parsing data : %s",parsingData.c_str());
    ROS_INFO("why stopping...");
    return false;
  }
  return true;
}

bool IMUConfigurator::ok(){
  return ser_ptr_->isValid();
}
