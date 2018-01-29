#include <ros/ros.h>
#include <deque>
#include "PlatformRX.h"
#include "PlatformRxParameter.h"
#include "sensor_configurator/PlatformRX_msg.h"

static const size_t PlatformRXPacketByte = 18U;
static constexpr int EncoderIndex = 11;
static constexpr int SteerIndex = 8;
static constexpr int BrakeIndex = 10;
//83 84 88 0 1 0 0 0 0 0 -56 -42 6 0 0 -75 13 10
static constexpr int32_t EncoderBound = 100;
static constexpr int16_t SteerBound = 1000;
static constexpr int8_t BrakeBound = 10;


bool checkSerial(serial::Serial **ser, int argc, char **argv);
//serial에 오버로디드 대입 연산자가 삭제됨
//따라서 포인터로 객체를 만들어옴

struct Past{
    Past() : encoder(std::deque<int32_t>(10)), steering(0), brake(0) {}
    std::deque<int32_t> encoder;
    int16_t steering;
    int8_t  brake;
};

#define MY_DEBUG_FLAG 1

int main (int argc, char** argv){
    bool startFlag = false;
    serial::Serial *ser = nullptr;

    if(checkSerial(&ser,argc, argv) == false)
        return -1;
    //open serial
    
    
    ros::init(argc, argv, PLATFORMRX_NAME);
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_configurator::PlatformRX_msg>(PLATFORMRX_PARAM_NAME,100);
    sensor_configurator::PlatformRX_msg msg;
    Past past;
    
    //initialize
    msg.speed = 0;      
    msg.steering = 0;          
    msg.brake = 0;             

    ros::Rate loop_rate(PLATFORMRX_LOOP_RATE);

    int frequencyBetweenData = 0;

    while(ros::ok()){
        std::string raw;

        try{
            raw = ser->read(ser->available());
        }
        catch(...){//invoke when serial port is unpluged
            delete ser;
            checkSerial(&ser, argc, argv);
        }
        frequencyBetweenData++;
        if(raw.size() < PlatformRXPacketByte){
            ROS_WARN("Invalid Packet size : %ld needed but Got %ld"
                , PlatformRXPacketByte, raw.size());
            continue;
        }
        else frequencyBetweenData++;

        uint8_t dataArray[PlatformRXPacketByte];
        for(int i = 0 ; i < PlatformRXPacketByte;++i){
            dataArray[i] = raw.c_str()[i];
        }

        /*--- encoder --- */
        //get Data
        int32_t encoderData = getParsingData<int32_t>(dataArray,EncoderIndex);
        encoderData = (abs(encoderData - past.encoder[0]) < EncoderBound) ?
            encoderData : past.encoder[0];
        past.encoder.push_front(encoderData);
        past.encoder.pop_back();

        msg.speed = calcSpeed(past.encoder, frequencyBetweenData);
        frequencyBetweenData = 0;
    
        //brake
        uint8_t brakeData = getParsingData<uint8_t>(dataArray,BrakeIndex);
        msg.brake = isDataInBound<int8_t>(msg.brake, past.brake, BrakeBound) ? 
            brakeData : past.brake;
        past.brake = msg.brake;
        
        //steering
        uint16_t steeringData = getParsingData<uint16_t>(dataArray,SteerIndex);
        msg.steering = isDataInBound<int16_t>(msg.steering, past.steering, SteerBound) ? 
            steeringData : past.steering;
        past.steering = msg.steering;
        

        #ifdef MY_DEBUG_FLAG
            ROS_INFO("encoder : %d",past.encoder[0]);
            ROS_INFO("speed : %lf", msg.speed);
            ROS_INFO("brake : %d", msg.brake);
            ROS_INFO("steering : %hd",msg.steering);
        #endif
        pub.publish(msg);
        loop_rate.sleep();
    }
}

bool checkSerial(serial::Serial **ser, int argc, char **argv){
    bool startFlag = false;

    while(startFlag == false){
        try{
            if(argc != 2)
                throw std::runtime_error("argument error. Give me [path]");
            
            *ser = getSerial(argv[1],115200);

            return true;
        }
        catch (serial::IOException& e){
            ROS_ERROR("serial error[%s] : wait for 5 second to repair...", e.what());
            sleep(5);
        }
        catch (std::runtime_error& e){
            ROS_ERROR("%s",e.what());
            return false;
        }
        catch (...){
            ROS_ERROR("Unknown Serial error. Exit Node");
            return false;
        }
    }
}
