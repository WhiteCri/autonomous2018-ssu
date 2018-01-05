#pragma once

#include <serial/serial.h>
#include <iostream>

using std::string;

class SerialCommunicator{
public:
  //Constructor
  SerialCommunicator(const string& port, int baudrate = 9600, int timeout = 1000);

  //return if instance is valid
  bool isCreationValid() const;

  bool isValid() const;

  //return second line from rawData
  const std::string& read();
  //return raw from sensor
  const std::string& getRaw();

  //write a data to the sensor
  void write(const uint8_t* data, size_t length);

  //if instance is not valid(creation error or availability error), you can
  //get the error type using this function
  const string& getErrorMsg();

  //get member
  const string& getPort();
  int getBaudrate() const;
  int getTimeout() const;

private:
  //serial communication setting
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
  bool creationCheck;
  bool validDataCheck;
};
