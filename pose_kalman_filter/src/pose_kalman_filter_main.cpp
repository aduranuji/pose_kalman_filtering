/** This service has generated using service_base 1.5 version */
#define SERVICE_BASE_VERSION 1.5

#include <pose_kalman_filter_service.h>

int main(int argc,char ** argv)
{
	ROS_INFO_STREAM("Service Base Version: "<<SERVICE_BASE_VERSION);
	PoseKalmanFilterService service;
	service.initService(argc,argv,"PoseKalmanFilter");
	return service.runService();
}
