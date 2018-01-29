#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <string>
#include <deque>

#define FRE 50
//vector의 0번방 : 최근 value
//vector의 1번방 : 나중 value
//(나중데이터 - 최근데이터) / 99.28 * (1.655m)
//(나중데이터 - 최근데이터) / (바퀴당 회전값) * (한바퀴당 주행 거리) * (주기를 이용한 시간 값)

#define PlatformParamName "RAW_PLATFORM_ENCODER_SPEED"
#define ENCODER_INDEX 11
//


int main(int argc, char *argv[]){
    ros::init(argc, argv, "test_main_node");
    ros::NodeHandle nh;
    serial::Serial ser;

    try{
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        //0 is error num, but i don't know each number point which err. i just put it to
        //complete function call
    }
    catch(serial::IOException& e){
        ROS_INFO("erro : %s",e.what());
        return -1;
    }
    if(!ser.isOpen()) {
        ROS_INFO("Serial is not open!");
        return -1;
    }

    ros::Rate loop_rate(FRE);
     std::string buffer = "";
     unsigned int total = 0;
     char str[20] = "0";
     uint8_t TXarray[14];   
     TXarray[0] = 0x53;
     TXarray[1] = 0x54;
     TXarray[2] = 0x58;
     TXarray[3] = 0x01;
   TXarray[4] = 0x00;
   TXarray[5] = 0x00;
   TXarray[6] = 0x0;
   TXarray[7] = 0x0;
   TXarray[8] = 0x1;
   TXarray[9] = 0x0;
       TXarray[10] = 0x01;
       TXarray[11] = 0x00;
       TXarray[12] = 0x0D;
       TXarray[13] = 0x0A;    
       std::vector<int> encoHis = {0,0};
       int fr = 0;
       while(ros::ok()){
               ser.write((uint8_t*)TXarray, sizeof(TXarray));
          TXarray[11] = ++TXarray[1] % 0xff;
          nh.setParam(PlatformParamName,encoHis);
          nh.setParam("RAW_PLATFORM_FREQUENCY_BETWEEN_ENCODER",fr);
          fr = 0;
        //  for(int i=0; i<14; ++i)
       //       ROS_INFO("%x",TXarray[i]);
       
      loop_rate.sleep();
  }
}