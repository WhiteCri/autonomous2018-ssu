#pragma once

#include <serial/serial.h>
#include <iostream>

using std::string;

class SerialCommunicator{
public:
  SerialCommunicator(const string& port, int baudrate = 9600, int timeout = 1000);
  bool isValid();
  const std::string& read();
  const std::string& getRaw();
  void write(const string& write);
  const string& getErrorMsg();
private:
  //serial /Accelation_X
/Accelation_Y
/Accelation_Z
/Pitch
/Roll
/Yaw
communication setting
  serial::Serial ser;
  string port_;
  int baudrate_;
  int timeout_;

  //hold raw data
  string raw;
  //hold oneline from raw
  string now;

  //hold errorMsg
  static string errorMsg;

  //check if valid instance
  bool validInstance;
};
