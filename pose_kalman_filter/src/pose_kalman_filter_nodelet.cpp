/*
 * nodelet_test.cpp
 *
 *  Created on: 09/05/2012
 *      Author: angeld
 */


#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pose_kalman_filter_service.h>



namespace pose_kalman_filter
{
	class PoseKalmanFilterNodelet:public nodelet::Nodelet
	{
	public:
	   PoseKalmanFilterNodelet(){};
	private:
	  PoseKalmanFilterService serv_;
		void onInit()
		{
			NODELET_DEBUG("Initializing nodelet....");
			NODELET_INFO_STREAM("Init nodelet: "<<getName());
			
			serv_.initService(0,NULL,"PoseKalmanFilter");
			serv_.runService();
		  
		}
	};

	PLUGINLIB_DECLARE_CLASS(pose_kalman_filter,PoseKalmanFilterNodelet,pose_kalman_filter::PoseKalmanFilterNodelet,nodelet::Nodelet)
}

