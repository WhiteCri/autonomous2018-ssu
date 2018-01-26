#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_configurator/DriveIssue.h"
#include "sensor_configurator/platformtx_data.h"

#define FRE 50
PlatformTXdata TXdata;

void msgCallback(const sensor_configurator::DriveIssue::ConstPtr& msg)
{
    TXdata.setTXdata(msg->speed,msg->steer,msg->brake);   

    for(int i = 0; i < 14; ++i)
        ROS_INFO("%x",TXdata.TXarray[i]);

    ROS_INFO("---------");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_subscriber");
    ros::NodeHandle nh;  
    /*serial::Serial ser;
     
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
        //return -1;
    }
    if(!ser.isOpen()) {
        ROS_INFO("Serial is not open!");
        //return -1;
    }*/

    ros::Rate loop_rate(FRE);

    while(ros::ok()){
        ros::Subscriber drive_sub = nh.subscribe("drive_msg",100,msgCallback);
        //ser.write((uint8_t*)TXdata.TXarray, sizeof(TXdata.TXarray));
        loop_rate.sleep();
        ros::spin();
     }
   
    //return 0;
}