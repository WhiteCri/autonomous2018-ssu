#include <ros/ros.h>
#include <serial/serial.h>
#include <PlatformRX.h>

static const double encoderValuePerCycle = 99.2;
static const double distanceValuePerCycle = 1.655;// m

serial::Serial *getSerial(const char* path_, int baudrate_){
    serial::Serial *ser = new serial::Serial();

    ser->setPort(path_);
    ser->setBaudrate(baudrate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser->setTimeout(to);
    ser->open();
    if(!ser->isOpen()) throw serial::IOException("ser.isOpen() error!",__LINE__,"ser.isOpen() error!");
    return ser;
}

double calcSpeed(std::deque<int> encoderValue, int frequencyBetweenData){
    //(두 엔코더 값의 차) / (돌아간 시간) * (1바퀴당 엔코더 값의 변화) * (1바퀴당 회전거리)
    return (encoderValue[0] - encoderValue[1])/(frequencyBetweenData / 60.0)
        /encoderValuePerCycle * distanceValuePerCycle;
}
