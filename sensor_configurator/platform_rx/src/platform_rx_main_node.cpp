#include <ros/ros.h>
#include <deque>
#include <serial/serial.h>
#include "platform_rx/PlatformRX_msg.h"

static const size_t PlatformRXPacketByte = 18U;
static constexpr int EncoderIndex = 11;
static constexpr int SteerIndex = 8;
static constexpr int BrakeIndex = 10;
//83 84 88 0 1 0 0 0 0 0 -56 -42 6 0 0 -75 13 10
typedef int32_t EncoderDataType;
typedef int16_t SteeringDataType;
typedef int8_t BrakeDataType;

//initializing macro
static constexpr EncoderDataType    InitialEncoderLength    = 15;
static constexpr EncoderDataType    EncoderInitialDataBound = 200;

//handling error encoder value function
inline double handleEncoderValue(std::deque<EncoderDataType> target){
    //[2][1][0]
    //new = [0] + (([1] - [0]) - (([2] - [1]) - ([1] - [0])))
    double re = 3 * target[0] - 3 * target[1] + target[2];
}

static constexpr EncoderDataType    EncoderInitialBound     = 10000000;//1000만.
//바퀴당 100씩 엔코더가 변화하므로 1000만 / 100 * 1.655m = 165500m, 165km임
static constexpr EncoderDataType    EncoderBound            = 50;
static constexpr SteeringDataType   SteerBound              = 1000;
static constexpr BrakeDataType      BrakeBound              = 10;

int loop = 0;

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

static const double encoderValuePerCycle = 99.2;
static const double distanceValuePerCycle = 1.655;// m

bool checkSerial(serial::Serial **ser, int argc, char **argv);
//serial에 오버로디드 대입 연산자가 삭제됨
//따라서 포인터로 객체를 만들어옴

struct Past{
    Past() : encoder(std::deque<EncoderDataType>(InitialEncoderLength)), steering(0), brake(0) {}
    std::deque<EncoderDataType> encoder;
    SteeringDataType      steering;
    BrakeDataType         brake;
};

#define MY_DEBUG_FLAG 1

int main (int argc, char** argv){
    bool startFlag = false;
    serial::Serial *ser = nullptr;

    if(checkSerial(&ser,argc, argv) == false)
        return -1;
    //open serial
    
    
    ros::init(argc, argv, "platform_rx_main_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<platform_rx::PlatformRX_msg>("raw/platform_rx",100);
    platform_rx::PlatformRX_msg msg;
    Past past;

    //initialize   
    msg.steering = 0;          
    msg.brake = 0;             

    loop = atoi(argv[2]);
    ros::Rate loop_rate(loop);

    size_t cnt = 0;


    //init

    std::vector<EncoderDataType> debugVec(past.encoder.size());
    while(ros::ok()){
        std::string raw;

        try{
            raw = ser->read(ser->available());
        }
        catch(...){//invoke when serial port is unpluged
            delete ser;
            checkSerial(&ser, argc, argv);
        }
        if(raw.size() < PlatformRXPacketByte){
            ROS_WARN("Initial erro Invalid Packet size : %ld needed but Got %ld"
                , PlatformRXPacketByte, raw.size());
            //ROS_WARN("wait for 2 second");
            //sleep(2);
        } 
        else if (static_cast<int>(cnt) < past.encoder.size()){
            uint8_t dataArray[PlatformRXPacketByte];
            for(int i = 0 ; i < PlatformRXPacketByte;++i){
                dataArray[i] = raw.c_str()[i];
            }
            int32_t encoderData = getParsingData<int32_t>(dataArray,EncoderIndex);
            debugVec[cnt] = encoderData;
            //초기값의 인코더 bound를 설정해 너무 큰 잡음을 걸러낸다.
            if(abs(encoderData) > EncoderInitialBound) continue;
            past.encoder[cnt++] = encoderData;
        }
        else break;
        ROS_INFO("receiving initial value : %lu(cnt) need and Got %lu(cnt)",past.encoder.size(),cnt);
        loop_rate.sleep();
    } 

    {
        //initial algorithm start
        std::vector<std::vector<bool> > table(past.encoder.size());
        for(int i = 0 ; i < past.encoder.size();++i)
            table.emplace_back(past.encoder.size()); // fill with FALSE
        std::vector<int> score(past.encoder.size());

        //scoring
        int trueCnt;
        for(size_t i = 0; i < past.encoder.size();++i){
            trueCnt = 0;
            for(size_t j = 0 ; j < past.encoder.size();++j){
                if(abs(past.encoder[i] - past.encoder[j]) < EncoderInitialDataBound){
                    table[i][j] = true;
                    trueCnt++;
                }
            }
            score[i] = trueCnt;
        }

        //find highest score
        int idx = 0;
        for(size_t i = 0 ; i < past.encoder.size();++i){
            if(score[idx] < score[i]) idx = i;
        }

        //reconstruct recent 3 data with most recent valid value
        int validDataIndex = static_cast<int>(past.encoder.size() - 1);
        while(!table[idx][--validDataIndex]);
        past.encoder = std::deque<EncoderDataType>{
            past.encoder[validDataIndex],
            past.encoder[validDataIndex],
            past.encoder[validDataIndex]
        };
    }
    ROS_INFO("Initial end!");
    for(size_t i = 0 ; i < past.encoder.size();++i)
        ROS_INFO("[%lu] : %d, real : %d",i, past.encoder[i], debugVec[i]);

    //init end
 
    cnt = 0;
    while(ros::ok()){
        cnt++;
        std::string raw;

        try{
            raw = ser->read(ser->available());
        }
        catch(...){//invoke when serial port is unpluged
            delete ser;
            checkSerial(&ser, argc, argv);
        }
        if(raw.size() < PlatformRXPacketByte){
            ROS_WARN("Invalid Packet size : %ld needed but Got %ld"
                , PlatformRXPacketByte, raw.size());
            loop_rate.sleep();
            continue;
        }

        uint8_t dataArray[PlatformRXPacketByte];
        for(int i = 0 ; i < PlatformRXPacketByte;++i){
            dataArray[i] = raw.c_str()[i];
        } 

        /*--- distance --- */
        //get Data
        int32_t encoderData = getParsingData<int32_t>(dataArray,EncoderIndex);
        if(abs(encoderData - past.encoder[0]) > EncoderBound){
            ROS_WARN("Got super encoder Value%d!",encoderData);
            encoderData = handleEncoderValue(past.encoder);
        }
        else if ((encoderData - past.encoder[0]) == 0){
            ROS_WARN("Got same encoder Value : %d!",encoderData);
            encoderData = handleEncoderValue(past.encoder);
        }
        past.encoder.push_front(encoderData);
        past.encoder.pop_back();

        //time
    
        //brake
        uint8_t brakeData = getParsingData<uint8_t>(dataArray,BrakeIndex);
        msg.brake = isDataInBound<int8_t>(brakeData, past.brake, BrakeBound) ? 
            brakeData : past.brake;
        past.brake = msg.brake;
        
        //steering
        int16_t steeringData = getParsingData<uint16_t>(dataArray,SteerIndex);
        msg.steering = isDataInBound<int16_t>(steeringData, past.steering, SteerBound) ? 
            steeringData : past.steering;
        past.steering = msg.steering;
        

        #ifdef MY_DEBUG_FLAG
            ROS_INFO("[%ld]encoder : %d",cnt, past.encoder[0]);
            ROS_INFO("brake : %d", msg.brake);
            ROS_INFO("steering : %hd",msg.steering);
            ROS_INFO("distance : %lf",msg.distance);
            ROS_INFO("timeInterval : %lf",1 / static_cast<double>(loop));
        #endif
        pub.publish(msg);
        loop_rate.sleep();
    }
}

bool checkSerial(serial::Serial **ser, int argc, char **argv){
    bool startFlag = false;

    while(startFlag == false){
        try{
            if(argc <3)
                throw std::runtime_error("argument error. Give me [path] [frequency]");
            
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
