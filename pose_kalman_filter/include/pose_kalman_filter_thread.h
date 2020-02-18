#ifndef POSE_KALMAN_FILTER_THREAD_H
#define POSE_KALMAN_FILTER_THREAD_H

#include <service_thread.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/video/tracking.hpp>

typedef bool boolean;

using namespace services;
using namespace std;

/**
 * @class PoseKalmanFilterThread
 * @file pose_kalman_filter_thread.h
 * @brief The main loop of the service is defined in this class.
 */

namespace pose_kalman_filter{

namespace commands {
const string ACTIVATE_FILTERING="activate_filtering";
const string DEACTIVATE_FILTERING="deactivate_filtering";
const string SETUP_TF_PUBLISH="setup_tf_publish";
const string SET_MODE="set_mode";
}


namespace params {
const string LOOP_RATE="loop_rate";
const string BASE_FRAME="base_frame";
const string TARGET_FRAME="target_frame";
const string MODE="mode";
const string COVARIANCE_MEASURAMENT="covariance_measurament";
const string COVARIANCE_PROCESS="covariance_process";
const string POSE_FILTERED_ID="pose_filtered_id";
const string FRAME_ID_CHECK="frame_id_check";
}

namespace mode{
const string TOPIC="topic";
const string TF="tf";
}
}

using namespace pose_kalman_filter::params;
using namespace pose_kalman_filter::mode;

class PoseKalmanFilterThread:public ServiceThread
{
private:

	geometry_msgs::PoseStamped		pose_in_msg_;		/*!< Message to store pose_in IO message */
	geometry_msgs::PoseStamped		pose_out_msg_;		/*!< Message to store pose_out IO message */
	visualization_msgs::Marker		marker_pose_out_msg_;		/*!< Message to store marker_pose_out IO message */

	Subscriber			 pose_in_;

	Publisher			 pose_out_;
	Publisher			 marker_pose_out_;

	bool filterToInit_;
	bool activate_filtering_;

	cv::KalmanFilter kf_;
	cv::Mat rotationMat_; // 3x3
	cv::Mat positionVec_; // 3x1
	cv::Mat measurement_; // 6x1

	tf::TransformListener* tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;
	tf::StampedTransform current_pose_;
	tf::StampedTransform filtered_pose_;


public:

	/**
	 * Class constructor. You can create objects here  for using in thread
	 * You can change the behavior of thread calling a method to activate respond just new input or hold last input
	 */
	PoseKalmanFilterThread(string nodeName);
	/**
	 * Class destructor. You must destroy all objects that you created in constructor
	 */
	~PoseKalmanFilterThread();
	/**
	 *  This method override the compute method of service base thread class
	 * This method is called if:
	 * 1 A message has just arrived in a callback, and the main loop has processed the queue
	 * * The respondJustToNewInput() function is called in this thread constructor
	 * This methods is always called if the  respondHoldingTheLastInput() is configured in
	 * thread constructor
	 */
	virtual bool compute();

	/**
	 * 	This method is used to configure parameters, before the thread is going to start.
	 */
	virtual bool configureThread();

	/** 
	 * This methods is called when command activate is arrived
	 * \param iq is a internal request structure
	 * \return normally, the return values is true
	 */
	bool activateCommand(ModuleCommand::InternalRequest iq);
	/** 
	 * This methods is called when command deactivate is arrived
	 * \param iq is a internal request structure
	 * \return normally, the return values is true
	 */
	bool deactivateCommand(ModuleCommand::InternalRequest iq);




	/** This method is called when command: activate_filtering arrives at the server
	 * \param iq  it is a ModuleCommand Internal request structure, into this structure
	 * you could find two pointer, one to object representing the command received, and
	 * other, to object representing the response.
	 * Description:Activate filtering. The results will be published in the topic
	 */
	bool activateFiltering(ModuleCommand::InternalRequest iq);

	/** This method is called when command: deactivate_filtering arrives at the server
	 * \param iq  it is a ModuleCommand Internal request structure, into this structure
	 * you could find two pointer, one to object representing the command received, and
	 * other, to object representing the response.
	 * Description:Deactivate the filtering, In the topic will be published the same value that the input
	 */
	bool deactivateFiltering(ModuleCommand::InternalRequest iq);

	/** This method is called when command: setup_tf_publish arrives at the server
	 * \param iq  it is a ModuleCommand Internal request structure, into this structure
	 * you could find two pointer, one to object representing the command received, and
	 * other, to object representing the response.
	 * Description:The string array values of the message command will have to define a valid refrence tf frame in pose 0 of the array, and the target frame in pose 1 of the array
	 */
	bool setupTfPublish(ModuleCommand::InternalRequest iq);

	/** This method is called when command: set_mode arrives at the server
	 * \param iq  it is a ModuleCommand Internal request structure, into this structure
	 * you could find two pointer, one to object representing the command received, and
	 * other, to object representing the response.
	 * Description:In the message string array must be defined one value with these: tf|topic.
	 */
	bool setMode(ModuleCommand::InternalRequest iq);


	/** This callback is executed when a message arrives to pose_in subscriber
	 *  \param in it is the message of type geometry_msgs/PoseStamped
	 *  rief Pose stamped will be filtered
	 */
	void callBackPoseIn(const geometry_msgs::PoseStamped::ConstPtr & in);


	 void createKalmanFilter();
	 void setIntervalOfTheFilter(double dt);
	 void initKalmanFilter();
	 void predictKalmanFilter(cv::KalmanFilter &KF,cv::Mat &translation_predicted, cv::Mat &rotation_predicted );

	 cv::Mat euler2rot(const cv::Mat & euler);
	 cv::Mat rot2euler(const cv::Mat & rotationMatrix);

	 void copyStampedTfToCv(const tf::StampedTransform &pose);
	 void copyStampedPoseToCv(const geometry_msgs::PoseStamped & pose);

	 void fillMeasurements( cv::Mat &measurements,
	                    const cv::Mat &translation_measured, const cv::Mat &rotation_measured);
	 void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
	                      cv::Mat &translation_estimated, cv::Mat &rotation_estimated );

	 bool getCurrentTfTransform();
	 bool checkCurrentTfTransform();

	 void getPoseFromCv(  tf::Vector3 &pos,tf::Quaternion &q);

	 bool changeModeBehaviour();
};



#endif /* POSE_KALMAN_FILTER_THREAD_H */
