#include <ros/ros.h>
#include <serial/serial.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <deque>
#include <thread>
#include <mutex>
#include "platform_controller/cmd_platform.h"

#define TX_PACKET_LENGTH 14
#define TX_SERIAL_FREQUENCY 25
#define TX_STOP_CHECK_PERIOD 10

/*
 실제 차량의 dynamic에 의해 제한되는 최대조향각과 최대가속도, 최대속도값은 teb_local_planner의 parameter 튜닝으로 설정함.
 여기서는 Boundary Check만 진행 (원래 여기에 걸리는 것도 문제가 있음)
 여기서 걸리지 않게 테스트하면서 teb_local_planner parameter 튜닝할 필요가 있지만, 이 작업은 시간이 있으면 진행하는게 좋을듯
*/


//#define TX_DEBUG
#define RX_SUBSCRIBE


static std::string cmd_platform_topic_name;
static int alignmentBias;
static bool tx_stop;

uint8_t packet[TX_PACKET_LENGTH] = {};

//serial
serial::Serial *ser;
std::mutex lock;

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
    nh.param<std::string>("/platform_tx/cmd_platform_topic_name", cmd_platform_topic_name, "/control/cmd_platform");

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


void serialWrite(){
    ros::Rate loop_rate(TX_SERIAL_FREQUENCY);
    while(true){
        lock.lock();
        if(tx_stop){
            *(uint16_t*)(packet + 7) = static_cast<uint16_t>(0);//speed
            *(int8_t*)(packet + 8) = 0;
            *(int8_t*)(packet + 9) = 0; //steer
            packet[10] = static_cast<uint8_t>(0); // brake
        }
        ser->write(packet,TX_PACKET_LENGTH);
        lock.unlock();
        loop_rate.sleep();
    }
}

void processTxStop(ros::NodeHandle& nh){
    ros::Rate loop_rate(TX_STOP_CHECK_PERIOD);
    while(true){ 
        nh.getParam("hl_controller/tx_stop", tx_stop);
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
    packet[5] = static_cast<uint8_t>(msg->gear);
    ROS_INFO("%s",(msg->gear == 0) ? "forward" :"back");
// SPEED (원래는 accel이 맞는데 혼란을 피하기 위해 변수 이름들은 그냥 speed로 했음)
    int speed = fabs(msg->accel); //   checkSpeedBound(speed);
    uint16_t serialSpeed = speed;
    *(uint16_t*)(packet + 7) = static_cast<uint16_t>(serialSpeed);
    ROS_INFO("serial speed : %u", serialSpeed);

// STEER
    int angle = - msg->steer; //   checkSteeringBound(angle);
    int16_t serialSteeringAngle = angle + alignmentBias;
    *(int8_t*)(packet + 8) = *((int8_t*)(&serialSteeringAngle) + 1);
    *(int8_t*)(packet + 9) = *(int8_t*)(&serialSteeringAngle);
    ROS_INFO("serial angle : %d", serialSteeringAngle);

// BRAKE
    int brake = msg->brake; //   checkBrakeBound(brake); 
    packet[10] = static_cast<uint8_t>(brake);
    ROS_INFO("serial brake : %u", brake);

// ALIVE
    packet[11] = static_cast<uint8_t>(alive);    alive = (alive + 1) % 256;
}



void Cmd_CallBack_(const platform_controller::cmd_platform::ConstPtr& msg){
    createSerialPacket(msg);
}



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

    std::thread tr(serialWrite);
    tr.detach();
    std::thread tr2(std::bind(processTxStop, nh));
    tr2.detach();

#ifndef TX_DEBUG
    ros::spin();
///    while(ros::ok()){
///        ros::spinOnce();
///        //ser->write(packet,TX_PACKET_LENGTH);
///        loop_rate.sleep();
///    }
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