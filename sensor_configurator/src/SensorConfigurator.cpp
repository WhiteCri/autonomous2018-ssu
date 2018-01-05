#include "SensorConfigurator.h"

void SensorConfigurator::setSerial(const string& path, int baudrate, int timeout){
  if(ser_ptr_) delete ser_ptr_;
    this->ser_ptr_ = new SerialCommunicator(path, baudrate, timeout);
  ROS_INFO("Serial Setting end! : %s",ser_ptr_->getPort());
}

SensorConfigurator::SensorConfigurator(){
  ser_ptr_ = nullptr;
}

SensorConfigurator::~SensorConfigurator(){
  if(ser_ptr_) delete ser_ptr_;
}
