#include <ros/ros.h>
#include <serial/serial.h>
#include <mutex>
#include <thread>
#include <deque>
#include "platform_rx_msg/platform_rx_msg.h"

static constexpr int serial_read_loop_rate = 20;
static constexpr int pub_loop_rate = 10;
static constexpr int encoder_memory = 4;
static constexpr int encoder_gap_memory = encoder_memory - 1;
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

    ros::Publisher pub = nh.advertise<platform_rx_msg::platform_rx_msg>("raw/platform_rx", 1000);

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

    //speed calcuration member
    std::deque<int32_t> encoder(encoder_memory);
    std::deque<int32_t> encoderGap(encoder_gap_memory);

    //init
    while(ros::ok()){
        //get packet
        lock.lock();
        for(int i = 0 ; i < 18; ++i) *(packet_main + i) = *(packet + i);
        lock.unlock();

        //calc speed
        encoder.push_front(getParsingData<int32_t>(packet_main, 11));
        encoder.pop_back();
        encoderGap.push_front(encoder[0] - encoder[1]);
        encoderGap.pop_back();
ROS_INFO("gap[%ld] : %d, encoder : %d",seq, encoderGap[0],encoder[0]);
        msg.speed = (encoderGap[0] + encoderGap[1] + encoderGap[2]) / 99.2 * 1.655 
            / 0.1 / 3;
         //speed = (encoder[0].first - encoder[1].first) / encoderValuePerCycle * distanceValuePerCycle 
         //   / timeInterval / interval;

        msg.steer = getParsingData<int16_t>(packet_main, 8);
        msg.brake = getParsingData<uint8_t>(packet_main, 10);
        msg.seq = seq++;

        if(seq > 5)
            pub.publish(msg);     
        loop_rate.sleep();
    }
}

