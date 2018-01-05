#include "SerialCommunicator.h"
#include <string>
#include <sstream>
string SerialCommunicator::errorMsg = "";
SerialCommunicator::SerialCommunicator(const std::string& port, int baudrate, int timeout)
    //serial setting member
    : port_(port), baudrate_(baudrate), timeout_(timeout), validInstance(false),
    //data member
    raw(""), now(""), ser()
{
  try{
    ser.setPort(port_.c_str());
    ser.setBaudrate(baudrate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
    ser.setTimeout(to);
    ser.open();
    //0 is error num, but i don't know each number point which err. i just put it to
    //complete function call
    if(!ser.isOpen()) throw serial::IOException("ser.isOpen() error!",__LINE__,1);
  }
  catch (serial::IOException& e)
  {
      //hold error Msg
      errorMsg = e.what();
      return;
  }
  catch (std::exception& e)
  {
    errorMsg = string("Not a serial IOException : ") + e.what();
    return;
  }

  validInstance = true;
}

bool SerialCommunicator::isValid(){
  return validInstance;
}

const string& SerialCommunicator::read(){
  if(!isValid()) {
    now = "";
    raw = "";
    return now;
  }

  if(ser.available()){
    //validInstance = true;
      raw = ser.read(ser.available());

      //one line parsing
      std::stringstream ss(raw);
      std::getline(ss,now);
      std::getline(ss,now);
  }
  else {
    errorMsg = "serial is not available";
    raw = now = "";
    //validInstance = false;
  }

  return now;
}

const string& SerialCommunicator::getRaw(){
  return raw;
}

void SerialCommunicator::write(const string& data){
  ser.write(data.c_str());
}

const string& SerialCommunicator::getErrorMsg(){
  return SerialCommunicator::errorMsg;
}
