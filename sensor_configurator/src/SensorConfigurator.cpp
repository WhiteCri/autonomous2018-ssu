#include "SensorConfigurator.h"

void SensorConfigurator::setSerial(const string& path, int baudrate, int timeout){
  if(ser_ptr_) delete ser_ptr_;
    this->ser_ptr_ = new SerialCommunicator(path, baudrate, timeout);
}

SensorConfigurator::SensorConfigurator(){
  ser_ptr_ = nullptr;
}

SensorConfigurator::~SensorConfigurator(){}
