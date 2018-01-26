//Header for Ros
#include"ros/ros.h"
#include"nmea_navsat_driver/nmea_msg.h"
#include"nmea_navsat_driver/NmeaMsg.h"
#include"GPSParameter.h"
//Use String 함수
#include<string>
#include<sstream>



//Subscribe 후 Publish하기 위한 callback 함수
void msgCallback(const nmea_navsat_driver::nmea_msg::ConstPtr& msg2){
    // GPS Node로부터 Subscribe
    std::string raw = msg2->data;
    // Publish하기 위한 변수선언
    ros::NodeHandle nh;
    ros::Publisher GPS_publisher = nh.advertise<nmea_navsat_driver::NmeaMsg>("raw/GNSS",100);
    nmea_navsat_driver::NmeaMsg msg;
    // Data Pharse 변수선언
    std::stringstream ss(raw);
    std::string data;
    // 속도 변환하기 위한 변수선언
    double speed;
    for(int i=1; i<11;i++){
        std::getline(ss,data,',');
        //ROS_INFO("%s",data.c_str());
        switch(i){
            case GPS_Sentence_ID:
                if(data.compare("$GPRMC")){ // GPRMC Data Format이 아니면 종료
                //  ROS_INFO("WRONG DATA FORMAT");
                    i=11;
                }
                break;
            case GPS_Status:
                msg.Status = data;
                break;
            case GPS_Latitude:
                msg.Latitude = data;
                break;
            case GPS_NS_Indicator:
                msg.NSIndicator = data;
                break;
            case GPS_Longitude:
                msg.Longitude = data;
                break;
            case GPS_EW_Indicator:
                msg.EWIndicator = data;
                break;
            case GPS_Speed_Over_Ground:
                speed = atof(data.c_str());
                speed *= 1.852;
                data = std::to_string(speed);
                msg.Speed = data;
                break;
            case GPS_Course_Over_Ground:
                msg.Course = data;
                break;
            case GPS_Publish:
                GPS_publisher.publish(msg);
                break;
            default:
                break;
            }
       
    }
    return;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "GPS_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber nmea_subscriber = nh.subscribe("Nmea_Publish",100,msgCallback);
    ros::Publisher GPS_publisher = nh.advertise<nmea_navsat_driver::NmeaMsg>("raw/GNSS",100);
  
    ros::spin();
         
    return 0;
}