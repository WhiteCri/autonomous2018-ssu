#include <ros/ros.h>
#include <serial/serial.h>
#include <mutex>
#include <thread>
#include <deque>
#include <exception>
#include "platform_rx_msg/platform_rx_msg.h"

serial::Serial ser;
std::mutex lock;

class ParamReader{
public:
    ParamReader(ros::NodeHandle& nh){
        if(!nh.getParam("rx_thread_test/path", path)) throw std::runtime_error("give me path!");
        if(!nh.getParam("rx_thread_test/serial_frequency", serial_frequency)) 
            throw std::runtime_error("give me serial_frequency!");
        if(!nh.getParam("rx_thread_test/publish_frequency", publish_frequency)) 
            throw std::runtime_error("give me publish_frequency!");
        if(!nh.getParam("rx_thread_test/moving_average_element_number", moving_average_element_number))
            throw std::runtime_error("give me moving_average_element_number!");
        if(moving_average_element_number <= 1) std::runtime_error("too small element number...");
    }
    std::string path;
    int serial_frequency;
    int publish_frequency;
    int moving_average_element_number;
};

uint8_t packet[18] = {0};

template <typename T>
T getParsingData(const uint8_t *dataArray, int startIndex){
    T re_ = *(T*)(dataArray + startIndex);
    return re_;
}

void readSerial(int serial_read_loop_rate){
    ros::Rate loop_rate(serial_read_loop_rate);

    std::string raw;
    size_t error_cnt = 0;
    while(true){
        //read from serial
        if(ser.available() >= 18) raw = ser.read(ser.available());
        else loop_rate.sleep(); 

        //save in process variable
        const char *check = raw.c_str();
        if((check[0] == 0x53) & (check[1] == 0x54) & (check[2] == 0x58) & check[16] == 0x0D & check[17] == 0x0a ){
            lock.lock();
            for(int i = 0 ; i < 18; ++i) *(packet + i) = *(check + i);        
            lock.unlock();
        }
        loop_rate.sleep();
    }
}


int main (int argc, char** argv){
    ros::init(argc, argv, "rx_thread_test");
    ros::NodeHandle nh;

    ParamReader reader(nh);

    ros::Publisher pub = nh.advertise<platform_rx_msg::platform_rx_msg>("raw/platform_rx", 1000);

    try
    {
        ser.setPort(reader.path);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR("Unable to open port : %s",e.what());
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    std::thread thr(readSerial, reader.serial_frequency);
    thr.detach();

    platform_rx_msg::platform_rx_msg msg;
    ros::Rate loop_rate(reader.publish_frequency);
    size_t seq = 0;
    uint8_t packet_main[18] ={0};

    //speed calcuration member
    typedef int8_t ALIVE_datatype;
    std::deque<std::pair<int32_t, ALIVE_datatype> > encoder(reader.moving_average_element_number);

 //speed = (encoder[0].first - encoder[1].first) / encoderValuePerCycle * distanceValuePerCycle 
         //   / timeInterval / interval;
    auto calc_speed =[&]()->double{
        double total_encoder_gap = encoder.front().first - encoder.back().first;
        ALIVE_datatype alive_gap = encoder.front().second - encoder.back().second;
        double time_interval = alive_gap * 1.0 / 18; // platform send information for 50hz
        double speed = total_encoder_gap / 99.2 * 1.655 / time_interval;
        return speed;
    };
    //init
    for (int i = 0 ; i < 5; ++i){
        //get packet
        lock.lock();
        for(int i = 0 ; i < 18; ++i) *(packet_main + i) = *(packet + i);
        lock.unlock();

        //get serial sequence
        ALIVE_datatype alive = getParsingData<ALIVE_datatype>(packet_main, 15);
        
        encoder.push_front(std::make_pair(
            getParsingData<int32_t>(packet_main, 11),
            alive
        ));
        encoder.pop_back();
        seq += abs((int)encoder.front().first - (encoder.begin() + 1)->first);

        bool estop = getParsingData<uint8_t>(packet_main, 4);
        nh.setParam("estop", estop); 
        loop_rate.sleep();
    }
    
    while(ros::ok()){
        //get packet
        lock.lock();
        for(int i = 0 ; i < 18; ++i) *(packet_main + i) = *(packet + i);
        lock.unlock();

        //get serial sequence
        ALIVE_datatype alive = getParsingData<ALIVE_datatype>(packet_main, 15);
        
        encoder.push_front(std::make_pair(
            getParsingData<int32_t>(packet_main, 11),
            alive
        ));
        encoder.pop_back();

        if (encoder[0].second == encoder[1].second) { //when encoder is not updated, continue;
            loop_rate.sleep();
            continue;
        }
        msg.speed = calc_speed();
        msg.steer = getParsingData<int16_t>(packet_main, 8);
        msg.brake = getParsingData<uint8_t>(packet_main, 10);
        msg.seq = seq++;
        bool estop = getParsingData<uint8_t>(packet_main, 4);
        nh.setParam("estop", estop);

        pub.publish(msg);     
        loop_rate.sleep();
    }
}

