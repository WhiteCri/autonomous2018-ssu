#include <ros/ros.h>
#include <deque>
#include <serial/serial.h>
#include <utility>
#include "platform_rx_msg/platform_rx_msg.h"

#define EncoderAccumulateErrorRate 0.4

static const size_t PlatformRXPacketByte = 18U;
static constexpr int EncoderIndex = 11;
static constexpr int SteerIndex = 8;
static constexpr int BrakeIndex = 10;
static constexpr int EstopIndex = 4;
//83 84 88 0 1 0 0 0 0 0 -56 -42 6 0 0 -75 13 10
typedef int32_t EncoderDataType;
typedef int16_t SteeringDataType;
typedef int8_t BrakeDataType;

//initializing macro
static constexpr EncoderDataType    InitialEncoderLength    = 15;
static constexpr EncoderDataType    EncoderInitialDataBound = 200;

static constexpr EncoderDataType    EncoderInitialBound     = 10000000;//1000만.
//바퀴당 100씩 엔코더가 변화하므로 1000만 / 100 * 1.655m = 165500m, 165km임
static constexpr EncoderDataType    EncoderBound            = 100;

int loop = 0;

template <typename T>
T getParsingData(const uint8_t *dataArray, int startIndex){
    T re_ = *(T*)(dataArray + startIndex);
    return re_;
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

inline double calcSpeed(std::deque<std::pair<EncoderDataType,bool> > encoder, double past, int& interval){
    double timeInterval = static_cast<double>(1) / static_cast<double>(loop);
    double speed = 0;
    try{
        speed = (encoder[0].first - encoder[1].first) / encoderValuePerCycle * distanceValuePerCycle 
            / timeInterval / interval;
    }
    catch(...){
        ROS_WARN("divide by zero. use past value");
        speed = past;
    }
    interval = 0;
    return speed;
}

bool checkSerial(serial::Serial **ser, int argc, char **argv);
//serial에 오버로디드 대입 연산자가 삭제됨
//따라서 포인터로 객체를 만들어옴

struct Past{
    Past() : encoder(std::deque<std::pair<EncoderDataType,bool> >(InitialEncoderLength)), steer(0), brake(0) {}
    //encoder deque는 엔코더의 값과 유효성을 동시에 저장함
    std::deque<std::pair<EncoderDataType,bool> > encoder;
    SteeringDataType      steer;
    BrakeDataType         brake;
    double                speed;
};

//#define MY_DEBUG_FLAG 1

int main (int argc, char** argv){
    bool startFlag = false;
    serial::Serial *ser = nullptr;

    if(checkSerial(&ser,argc, argv) == false)
        return -1;
    //open serial


    ros::init(argc, argv, "platform_rx_main_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<platform_rx_msg::platform_rx_msg>("raw/platform_rx",100);
   
    Past past;

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
            past.encoder[cnt++].first = encoderData;
        }
        else break;
        ROS_INFO("receiving initial value : %lu(cnt) need and Got %lu(cnt)",past.encoder.size(),cnt);
        loop_rate.sleep();
    }

    {
        //initial algorithm start
        std::vector<std::vector<bool> > table;
        for(int i = 0 ; i < past.encoder.size();++i)
            table.emplace_back(past.encoder.size()); // fill with FALSE
        std::vector<int> score(past.encoder.size());

        //scoring
        int trueCnt;
        for(size_t i = 0; i < past.encoder.size();++i){
            trueCnt = 0;
            for(size_t j = 0 ; j < past.encoder.size(); ++j){
                if(abs(past.encoder[i].first - past.encoder[j].first) < EncoderInitialDataBound){
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
        past.encoder = std::deque<std::pair<EncoderDataType,bool> >{
            past.encoder[validDataIndex],
            past.encoder[validDataIndex],
            past.encoder[validDataIndex]
        };
    }
    ROS_INFO("Initial end!");
    for(size_t i = 0 ; i < past.encoder.size();++i)
        ROS_INFO("[%lu] : %d, real : %d",i, past.encoder[i].first, debugVec[i]);

    //init end

    cnt = 0;
    int interval = 0;
    while(ros::ok()){
        platform_rx_msg::platform_rx_msg msg;
        cnt++;
        interval++;
        std::string raw;

        try{
            if(ser->waitReadable()){
                int availableCnt = ser->available();
                if(availableCnt >= 18)
                    ser->read(raw,ser->available());
                else {
                    ROS_WARN("[%ld]availableCnt is lower than packetLength : %d",cnt,availableCnt);
                    loop_rate.sleep();
                    continue;
                }
            }
            if(raw.c_str()[0] != 0x53){
                ROS_WARN("[%ld]invalid packet",cnt);
                loop_rate.sleep();
                continue;
            }
    
        }
        catch(...){//invoke when serial port is unpluged
            delete ser;
            checkSerial(&ser, argc, argv);
        }
        if(raw.size() < PlatformRXPacketByte){
            ROS_WARN("[%ld]Invalid Packet size : %ld needed but Got %ld"
                , cnt,PlatformRXPacketByte, raw.size());
            loop_rate.sleep();
            continue;
        }

        uint8_t dataArray[PlatformRXPacketByte];
        for(int i = 0 ; i < PlatformRXPacketByte;++i){
            dataArray[i] = raw.c_str()[i];
        }

        /*--- speed --- */
        //get Data
        int32_t encoderData = getParsingData<int32_t>(dataArray,EncoderIndex);
        bool encoderErrorFlag = false;
        if(abs(encoderData - past.encoder[0].first) > EncoderBound){
            ROS_WARN("[%ld]Got super encoder Value%d!",cnt,encoderData);
            loop_rate.sleep();
            continue;
        }
        // e-stop에 대한 처리가 필요
        else if(getParsingData<bool>(dataArray,EstopIndex)){
            ROS_WARN("e-stop status");
            loop_rate.sleep();
            continue;
        }
        double up = (past.encoder[0].first - past.encoder[1].first) * interval;
        double down = encoderData - past.encoder[0].first + 0.01;
        double differentRate = up / down;
        ROS_INFO("different Rate : %lf",differentRate);
        //if(fabs(1 - differentRate) < EncoderAccumulateErrorRate){
        //    ROS_WARN("[%ld]invalid encoder value : %lf",cnt,differentRate);
        //    loop_rate.sleep();
        //    continue;
        //}
        //else if ((encoderData - past.encoder[0].first) == 0){
        //    if((encoderData-past.encoder[1]) == 0)
        //        if((encoderData-past.encoder[2]) == 0)
        //            ROS_WARN("Got same encoder Value : %d!",encoderData);
        //    //encoderErrorFlag = true;
        //}

        past.encoder.emplace_front(encoderData, !encoderErrorFlag);
        past.encoder.pop_back();
        msg.speed = calcSpeed(past.encoder, past.speed,interval);
        past.speed = msg.speed;
        
        //brake
        uint8_t brakeData = getParsingData<uint8_t>(dataArray,BrakeIndex);
        msg.brake = brakeData;
        past.brake = msg.brake;

        //steer
        int16_t steeringData = getParsingData<uint16_t>(dataArray,SteerIndex);
        msg.steer = steeringData;
        past.steer = msg.steer;

        //seq
        msg.seq = cnt;

        #ifdef MY_DEBUG_FLAG
            ROS_INFO("[%ld]encoder : %d",cnt, past.encoder[0].first);
            ROS_INFO("speed : %lf",msg.speed);
            ROS_INFO("brake : %d", (uint8_t)msg.brake);
            ROS_INFO("steering : %hd",msg.steer);
            for(int i = 0 ; i < 18; ++i)
                msg.raw.push_back(dataArray[i]);
           // for(int i = 0 ; i < 18; ++i)
           //     ROS_INFO("[%d] : %#x",i,msg.raw[i]);
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
