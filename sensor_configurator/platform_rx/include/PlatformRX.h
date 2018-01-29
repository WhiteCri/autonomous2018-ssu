#include <serial/serial.h>
#include <deque>

serial::Serial *getSerial(const char* path_, int baudrate_);

//encoder값이 21억이 넘는 순간 error
double calcSpeed(std::deque<int> encoderValue, int frequency);

template <typename T>
T getParsingData(const uint8_t *dataArray, int startIndex){
    T re_ = *(T*)(dataArray + startIndex);
    return re_;
}

template <typename __T>
bool isDataInBound(__T value_cur, __T value_past, __T bound){
    if((value_past - bound) < value_cur && value_cur < (value_past + bound))
        return true;
    else return false;
}
