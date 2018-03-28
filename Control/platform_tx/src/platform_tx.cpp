#include <ros/ros.h>
#include <serial/serial.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <deque>
#include <thread>
#include <mutex>
#include "platform_controller/cmd_platform.h"

#define TX_PACKET_LENGTH 14
#define TX_SERIAL_FREQUENCY 50

/*
 실제 차량의 dynamic에 의해 제한되는 최대조향각과 최대가속도, 최대속도값은 teb_local_planner의 parameter 튜닝으로 설정함.
 여기서는 Boundary Check만 진행 (원래 여기에 걸리는 것도 문제가 있음)
 여기서 걸리지 않게 테스트하면서 teb_local_planner parameter 튜닝할 필요가 있지만, 이 작업은 시간이 있으면 진행하는게 좋을듯
*/
#define MAX_SPEED 200
#define MAX_BRAKE 200
#define MAX_STEER 2000

//#define TX_DEBUG
//#define RX_SUBSCRIBE


static std::string cmd_platform_topic_name;
static int frequency;
static int alignmentBias;

std::mutex lock;

uint8_t packet[TX_PACKET_LENGTH] = {};

//serial
serial::Serial *ser;

void initTx(const ros::NodeHandle& nh){
    packet[0] = static_cast<uint8_t>(0x53);
    packet[1] = static_cast<uint8_t>(0x54);
    packet[2] = static_cast<uint8_t>(0x58);
    //manual OR auto. 0x00 - manual/0x01 - auto
    packet[3] = static_cast<uint8_t>(0x01);
    //estop. 0x00 - off/0x01 - on
    packet[4] = static_cast<uint8_t>(0x00);

    packet[12] = static_cast<uint8_t>(0x0D);//0x0D
    packet[13] = static_cast<uint8_t>(0x0A);//0x0A

    //setup
    nh.param("/platform_tx/frequency", frequency, 50);
    nh.param<std::string>("/platform_tx/cmd_platform_topic_name", cmd_platform_topic_name, "cmd_platform");

    //steering member
    nh.param("/platform_tx/alignmentBias", alignmentBias, 0);

}



//rx log variable
#ifdef RX_SUBSCRIBE
#include "platform_rx_msg/platform_rx_msg.h"
std::deque<platform_rx_msg::platform_rx_msg> rxMsgdeq(10);
//for pid. I'm not sure but maybe this will be needed in the future
void rxMsgCallBack(const platform_rx_msg::platform_rx_msg::ConstPtr& msg){
    platform_rx_msg::platform_rx_msg temp;
    rxMsgdeq.push_front(temp);
    rxMsgdeq.pop_back();
    ROS_INFO("rxMsgSubscribing Done!");
}
#endif



inline void checkSpeedBound(int& speed){
    speed = (speed <= MAX_SPEED) ? speed : MAX_SPEED;
    return;
}


inline void checkBrakeBound(int& brake){
    brake = (brake <= MAX_BRAKE) ? brake : MAX_BRAKE;
}


inline void checkSteeringBound(int& steeringAngle){
    if(steeringAngle > MAX_STEER)
        steeringAngle = MAX_STEER;
    else if(steeringAngle < -MAX_STEER)
        steeringAngle = -MAX_STEER;
}

void serialWrite(){
    ros::Rate loop_rate(TX_SERIAL_FREQUENCY);
    while(true){
        lock.lock();
        ser->write(packet,TX_PACKET_LENGTH);
        lock.unlock();
        loop_rate.sleep();
    }
}

void createSerialPacket(const platform_controller::cmd_platform::ConstPtr& msg){
    static uint8_t alive = 0;
/*  
    GEAR
    0x00 : forward
    0x01 : neutral
    0x02 : backward
*/
    packet[5] = (msg->accel >= 0) ?
         static_cast<uint8_t>(0x00) : static_cast<uint8_t>(0x02);
    
// SPEED (원래는 accel이 맞는데 혼란을 피하기 위해 변수 이름들은 그냥 speed로 했음)
    int speed = fabs(msg->accel);    checkSpeedBound(speed);
    uint16_t serialSpeed = speed;
    *(uint16_t*)(packet + 7) = static_cast<uint16_t>(serialSpeed);
    ROS_INFO("serial speed : %u", serialSpeed);

// STEER
    int angle = -msg->steer;    checkSteeringBound(angle);
    int16_t serialSteeringAngle = angle + alignmentBias;
    *(int8_t*)(packet + 8) = *((int8_t*)(&serialSteeringAngle) + 1);
    *(int8_t*)(packet + 9) = *(int8_t*)(&serialSteeringAngle);
    ROS_INFO("serial angle : %d", serialSteeringAngle);

// BRAKE
    int brake = msg->brake;    checkBrakeBound(brake); 
    packet[10] = static_cast<uint8_t>(brake);
    ROS_INFO("serial brake : %u", brake);

// ALIVE
    packet[11] = static_cast<uint8_t>(alive);    alive = (alive + 1) % 256;
}


void Cmd_CallBack_(const platform_controller::cmd_platform::ConstPtr& msg){
    createSerialPacket(msg);
}
/*
void subscribetopic()
{
    ros::NodeHandle Node;
    ros::Subscriber sub1 = Node.subscribe("/ackermann_cmd",100,&Cmd_CallBack_);
    ros::Subscriber sub2 = Node.subscribe("/raw/platform_rx",100,rxMsgCallBack);
    ros::MultiThreadedSpinner spinner(2);

    spinner.spin();
  
}
*/

int main(int argc, char *argv[]){
    if(argc < 2){
        ROS_ERROR("give me [path]");
        return -1;
    }
    ros::init(argc, argv, "platform_tx");
    ros::NodeHandle nh;
    
    initTx(nh);

    ros::Subscriber sub = nh.subscribe(cmd_platform_topic_name, 100, &Cmd_CallBack_);

    //open serial
    ser = new serial::Serial();
    ser->setPort(argv[1]);
    ser->setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser->setTimeout(to);
    ser->open();
    if(!ser->isOpen()) throw serial::IOException("ser.isOpen() error!",__LINE__,"ser.isOpen() error!");
    ROS_INFO("serial setting done");

    ros::Rate loop_rate(frequency);

    std::thread tr(serialWrite);
    tr.detach();

#ifndef TX_DEBUG
    while(ros::ok()){
        ros::spinOnce();
        //ser->write(packet,TX_PACKET_LENGTH);
        loop_rate.sleep();
    }
#else
    //5hz->9.75
    //10hz->9.95
    //15hz->9.56
    //20hz->9.68
    //50hz->9.02
    while(true){
        ackermann_msgs::AckermannDriveStamped::ConstPtr msg;
        createSerialPacket(msg);
        ser->write(packet, TX_PACKET_LENGTH);
        for(int i = 0 ; i < TX_PACKET_LENGTH;++i)
            ROS_INFO("[%d] : %#x",i,packet[i]);
        loop_rate.sleep();
    }
#endif
}