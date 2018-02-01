//Header for Ros
#include"ros/ros.h"
#include"sensor_msgs/NavSatFix.h"
#include"geometry_msgs/Pose.h"
//Use math 함수
#include<math.h>

static const double a = 6377397.155;
static const double e_squre = 0.006674372227347433;
static const double e_prime_squre= 0.006719218794659982;

// TM 계산을 위한 계수들 A-F;
static const double nTMScaleFacter = 1.0;
static const double nKatecScaleFacter = 0.9999;

static const double A = 1.005037306048555;
static const double B = 0.005047849240300;
static const double C = 0.000010563786831;
static const double D = 0.000000020633322;
static const double E = 0.000000000038865;
static const double F = 0.000000000000075;

//한국 경위도
static const double KmLatitude = 37.275529;
static const double KmLongitude = 127.051429;

#ifndef M_PI
#define M_PI		3.14159265358979323846	// pi 
#endif
// 복잡한 계산 수식 코드이므로 프로그램 실행속도를 높이기 위해서 간단한 변환함수는 inline을 이용
inline double Deg2Rad(double Degree)
{
	return Degree*M_PI/180.;
}

inline double Rad2Deg(double Radian)
{
	return Radian*180./M_PI;
}
// TM <-> 경위도 변환공식 http://m.cafe.daum.net/GPSGIS/Lrtt/754/
void Bessel2TM(double lon, double lat, double lonOrg, double latOrg, double *tm_x, double *tm_y)
{
	// Bessel -> TM

	double phi = Deg2Rad(lat);
	double lambda= Deg2Rad(lon);

	double phi_0 = Deg2Rad(latOrg); // 원점 

	double for_B = a*(1-e_squre)*(A*(phi-phi_0) 
		- 1./2. *B*(sin(2. *phi) - sin(2. *phi_0)) 
		+ 1./4. *C*(sin(4. *phi) - sin(4. *phi_0))
		- 1./6. *D*(sin(6. *phi) - sin(6. *phi_0)) 
		+ 1./8. *E*(sin(8. *phi) - sin(8. *phi_0))
		- 1./10.*F*(sin(10.*phi) - sin(10.*phi_0)));

	//계산을 위한 값들....
	double t = tan(phi);
	double eta = sqrt(e_prime_squre)*cos(phi); // eta_squre(eta^2 = e_prime_squre* cos^2 phi
	double N = a / sqrt(1 - e_squre * pow(sin(phi),2.));

	//Gauss-Kruger Projection 식.....
	double Delta_lambda = lambda - Deg2Rad(lonOrg);

	double x = for_B + pow(Delta_lambda,2.) / 2. * N * sin(phi) * pow(cos(phi),1.)
		+ pow(Delta_lambda,4.) / 24. * N * sin(phi) * pow(cos(phi),3.)*(5.- pow(t,2.) + 9.*pow(eta,2.) + 4.*pow(eta,4.))
		+ pow(Delta_lambda,6.) / 720. * N * sin(phi) * pow(cos(phi),5.)*(61. - 58.*pow(t,2) + pow(t,2.) + 270. * pow(eta,2) - 330 * pow(t,2) * pow(eta,2))
		+ pow(Delta_lambda,8.) / 40320.* N * sin(phi) * pow(cos(phi),7.)*(1385. - 3111.*pow(t,2.) + 543. * pow(t,4.) - pow(t,6.));


	double y = Delta_lambda * N * cos(phi)
		+ pow(Delta_lambda,3.) / 6. * N * pow(cos(phi),3.)*(1-pow(t,2.) + pow(eta,2.))
		+ pow(Delta_lambda,5.) / 120. * N * pow(cos(phi),5.)*(5.- 18.*pow(t,2.) + pow(t,4.) + 14.* pow(eta,2.) - 58.*pow(t,2.)*pow(eta,2))
		+ pow(Delta_lambda,7.) / 5040. * N * pow(cos(phi),7.)*(61. - 479.*pow(t,2) + 179.*pow(t,4.) - pow(t,6.));

	*tm_x = nTMScaleFacter*x; 
	*tm_y = nTMScaleFacter*y;
}
//  Matlab Code
void Tmtransform (double alt, double lon, double lat, double lonOrg, double latOrg, double *tm_x, double *tm_y)
{
	double R0 = 6378137.0;	// 지구 장축 길이
	double e = 0.081819191;	// 이심률
	
	double phi = Deg2Rad(lat);
	double den = sqrt(1 - e*e*sin(phi)*sin(phi));
	double RN = R0*(1 - e*e)/(den*den*den);
	double RE = R0/den;

	// 우리나라 TM(Transverse Mercator) 좌표계 원점들의 경위도 값
	// x-축 : 남북 방향, 북쪽이 +
	// y-축 : 동서 방향, 동쪽이 +
	*tm_x = (RN + alt)*Deg2Rad(lat - latOrg);
	*tm_y = (RE + alt)*Deg2Rad(lon - lonOrg)*cos(phi);
}

void msgCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
 
    ros::NodeHandle nh;
    ros::Publisher caculate_publisher = nh.advertise<geometry_msgs::Pose>("Pose",100);
    geometry_msgs::Pose mypose;

    double mylongitude = msg->longitude;
    double mylatitude = msg->latitude;

	// testing code : Bessel2TM(mylongitude, mylatitude, 127.051429, 37.275529, &mypose.position.x, &mypose.position.y);
	Tmtransform(0,mylongitude,mylatitude,KmLongitude, KmLatitude, &mypose.position.x, &mypose.position.y);
	caculate_publisher.publish(mypose);
   
   // Test 출력문 ROS_INFO("%lf",mylongitude);
    
    return;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "GPS_transformer");
    ros::NodeHandle nh;
    ros::Subscriber nmea_subscriber = nh.subscribe("fix",100,msgCallback);
    ros::Publisher caculate_publisher = nh.advertise<geometry_msgs::Pose>("Pose",100);
	
    ros::spin();
         
    return 0;
}