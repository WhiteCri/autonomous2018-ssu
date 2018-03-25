#include <ros/ros.h>
#include <serial/serial.h>
#include <mutex>
#include <thread>
#include "platform_rx_msg/platform_rx_msg.h"

static constexpr int serial_read_loop_rate = 50;
static constexpr int pub_loop_rate = 25;
serial::Serial ser;
std::mutex lock;

uint8_t packet[18] = {0};

template <typename T>
T getParsingData(const uint8_t *dataArray, int startIndex){
    T re_ = *(T*)(dataArray + startIndex);
    return re_;
}

void readSerial(){
    ros::Rate loop_rate(serial_read_loop_rate);

    std::string raw;
    size_t seq = static_cast<size_t>(-1);
    size_t error_cnt = 0;
    while(true){
        seq++;
        if(ser.available() >= 18) raw = ser.read(ser.available());
        else loop_rate.sleep(); 
        const char *check = raw.c_str();
        if((check[0] == 0x53) & (check[1] == 0x54) & (check[2] == 0x58) & check[16] == 0x0D & check[17] == 0x0a ){
            lock.lock();
            for(int i = 0 ; i < 18; ++i) *(packet + i) = *(check + i);        
            lock.unlock();
        }
        else{
            loop_rate.sleep();
        }
    }
}


int main (int argc, char** argv){
    ros::init(argc, argv, "rx_thread_test");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<platform_rx_msg::platform_rx_msg>("test/platform_rx", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    
    std::thread thr(readSerial);
    thr.detach();

    platform_rx_msg::platform_rx_msg msg;
    ros::Rate loop_rate(pub_loop_rate);
    size_t seq = 0;
    uint8_t packet_main[18] ={0};

int encoder_past = 0;    
    while(ros::ok()){
        lock.lock();
        for(int i = 0 ; i < 18; ++i) *(packet_main + i) = *(packet + i);
        lock.unlock();
        

        encoder_past = msg.encoder;
        msg.encoder = getParsingData<int32_t>(packet_main, 11);
        msg.encoder_gap = msg.encoder - encoder_past;

        pub.publish(msg);     
        loop_rate.sleep();
    }
}

