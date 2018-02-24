#include "IMUConfigurator.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_main");
    ros::NodeHandle nh;
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("raw/imu",100);
    
    IMUConfigurator imu(nh);

    if(argc < 3){
        ROS_ERROR("give me [path] [frequency]");
        return -1;
    }
    if(!imu.serialCommucation(argv[1])){
        ROS_ERROR("Serial port error");
        return -1;
    }

    int fre = atoi(argv[2]);

    ros::Rate loop_rate(fre);
    sensor_msgs::Imu imumsg;

    while(ros::ok()){
        imu.RPY(imu.parse());
        imumsg = imu.transform();
        pub_imu.publish(imumsg);
        loop_rate.sleep();
    }

}