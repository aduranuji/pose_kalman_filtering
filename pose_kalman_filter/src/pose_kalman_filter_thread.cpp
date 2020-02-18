#include "pose_kalman_filter_thread.h"

PoseKalmanFilterThread :: PoseKalmanFilterThread(string nodeName): ServiceThread(nodeName)
{

	ROS_INFO_STREAM("Thread " <<   n_->getNamespace()  <<  " constructor...");
	// Set the thread behavior
	// The line below to have the service responding only when a new input arrives, 
	// remember to use receivedNewInput(); 
	// in the callback to tell the thread that new data is ready to be processed

	respondJustToNewInput();

	// Uncomment the line below to have the service running continously,
	// does not take into account if new data has arrived to run the main loop
	//respondHoldingTheLastInput();

	//Loop rate is set by default at 30Hz, it is a required parameter and must be sopecified in every XML file used to generate services
	frequency_ = 30;
	setLoopRate(frequency_);
	tf_listener_ = NULL;
}

bool PoseKalmanFilterThread::changeModeBehaviour()
{
	if (T_PARAM(MODE,string)==TOPIC)
	{
		respondJustToNewInput();

	}else if (T_PARAM(MODE,string)==TF)
	{
		if (!checkCurrentTfTransform()) return false;
		respondHoldingTheLastInput();

	}else
	{
		ROS_ERROR_STREAM(n_->getNamespace()<<": Mode error. The module mode only can be tf|topic value");
		return false;
	}
	return true;
}
bool PoseKalmanFilterThread :: configureThread(){


	ROS_INFO_STREAM("Thread " <<   n_->getNamespace()  <<  " configure thread...");



	pose_in_=n_->subscribe<geometry_msgs::PoseStamped>("pose_in",1,&PoseKalmanFilterThread::callBackPoseIn,this);
	input_flags_["pose_in"]=0;

	pose_out_=n_->advertise<geometry_msgs::PoseStamped>("pose_out",1);

	marker_pose_out_=n_->advertise<visualization_msgs::Marker>("marker_pose_out",1);
	setLoopRate(frequency_);

	tf_listener_ = new tf::TransformListener();
	//******************************************************************
	// * This is the initialzation of the thread
	// * You must write here the initialization code that will be
	// * executed before the main loop starts.
	// * if this method returns false the execution is halt
	// * ****************************************************************


	positionVec_ =cv::Mat::zeros(3,1,CV_64F);
	rotationMat_ =cv::Mat::eye(3,3,CV_64F);
	measurement_ =cv::Mat::zeros(6,1,CV_64F);

	createKalmanFilter();

	if (!changeModeBehaviour())
		return false;

	activate_filtering_=true;
	filterToInit_=true;


	return true;
}

void PoseKalmanFilterThread::createKalmanFilter()
{
	ROS_INFO_STREAM("CREATE FILTER ... ");
	int nStates = 18;            // the number of states
	int nMeasurements = 6;       // the number of measured states
	int nInputs = 0;              // the number of action control
	double cov_measurement =PARAM(COVARIANCE_MEASURAMENT);
	double cov_process=PARAM(COVARIANCE_PROCESS);


	kf_.init(nStates, nMeasurements, nInputs, CV_64F);   // init linear Kalman Filter
	cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(cov_process));       // 1e-4 set process noise
	cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(cov_measurement));   // set measurement noise
	cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(100));  // er

	double default_dt = 1.0/frequency_;
	ROS_INFO_STREAM(" cov process "<<cov_process<<"cov meas "<<cov_measurement<<" dt= " <<default_dt);
	setIntervalOfTheFilter(default_dt);


	/* DYNAMIC MODEL */

	//  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]


	/* MEASUREMENT MODEL */

	//  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	//  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	//  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	//  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
	//  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
	//  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

	kf_.measurementMatrix.at<double>(0,0) = 1;  // x
	kf_.measurementMatrix.at<double>(1,1) = 1;  // y
	kf_.measurementMatrix.at<double>(2,2) = 1;  // z
	kf_.measurementMatrix.at<double>(3,9) = 1;  // roll
	kf_.measurementMatrix.at<double>(4,10) = 1; // pitch
	kf_.measurementMatrix.at<double>(5,11) = 1; // yaw

}

void PoseKalmanFilterThread::setIntervalOfTheFilter(double dt)
{
	// position
	kf_.transitionMatrix.at<double>(0,3) = dt;
	kf_.transitionMatrix.at<double>(1,4) = dt;
	kf_.transitionMatrix.at<double>(2,5) = dt;
	kf_.transitionMatrix.at<double>(3,6) = dt;
	kf_.transitionMatrix.at<double>(4,7) = dt;
	kf_.transitionMatrix.at<double>(5,8) = dt;
	kf_.transitionMatrix.at<double>(0,6) = 0.5*dt*dt;
	kf_.transitionMatrix.at<double>(1,7) = 0.5*dt*dt;
	kf_.transitionMatrix.at<double>(2,8) = 0.5*dt*dt;

	// orientation
	kf_.transitionMatrix.at<double>(9,12) = dt;
	kf_.transitionMatrix.at<double>(10,13) = dt;
	kf_.transitionMatrix.at<double>(11,14) = dt;
	kf_.transitionMatrix.at<double>(12,15) = dt;
	kf_.transitionMatrix.at<double>(13,16) = dt;
	kf_.transitionMatrix.at<double>(14,17) = dt;
	kf_.transitionMatrix.at<double>(9,15) = 0.5*dt*dt;
	kf_.transitionMatrix.at<double>(10,16) = 0.5*dt*dt;
	kf_.transitionMatrix.at<double>(11,17) = 0.5*dt*dt;

}
PoseKalmanFilterThread :: ~PoseKalmanFilterThread()
{
	if (tf_listener_ !=NULL) delete tf_listener_;
}


bool PoseKalmanFilterThread :: compute()
{

	//******************************************************************
	// * This is the main loop of the thread
	// * You must write here the code of the main function of the service
	// * ****************************************************************


	// get current pose
	if (T_PARAM(MODE,string)==TOPIC)
	{
		copyStampedPoseToCv(pose_in_msg_);
		if (!T_PARAM_PTR(FRAME_ID_CHECK,string)->empty())
		{
			pose_out_msg_.header.frame_id=T_PARAM(FRAME_ID_CHECK,string);
			ROS_INFO("label");
		}

	}else if (T_PARAM(MODE,string)==TF)
	{
		if (getCurrentTfTransform()){
			copyStampedTfToCv(current_pose_);
			pose_out_msg_.header.frame_id=T_PARAM(BASE_FRAME,string);
		}
	}else
	{
		ROS_ERROR_STREAM(n_->getNamespace()<<": Mode error. The module is not computing");
		return true;
	}
	if (activate_filtering_){
		if (filterToInit_)
		{
			initKalmanFilter();
		}else
		{

			cv::Mat posVec=positionVec_.clone();
			cv::Mat rotMat=rotationMat_.clone();
			predictKalmanFilter( kf_, posVec , rotMat);

			fillMeasurements(measurement_, positionVec_ , rotationMat_);
			updateKalmanFilter( kf_, measurement_, positionVec_ , rotationMat_);

		}

	}
	tf::Vector3 out_pos;
	tf::Quaternion out_or;
	getPoseFromCv(out_pos,out_or);
	pose_out_msg_.pose.position.x=out_pos.getX();
	pose_out_msg_.pose.position.y=out_pos.getY();
	pose_out_msg_.pose.position.z=out_pos.getZ();
	pose_out_msg_.pose.orientation.x=out_or.getX();
	pose_out_msg_.pose.orientation.y=out_or.getY();
	pose_out_msg_.pose.orientation.z=out_or.getZ();
	pose_out_msg_.pose.orientation.w=out_or.getW();

	pose_out_msg_.header.stamp=ros::Time::now();
	pose_out_.publish(pose_out_msg_);

	ROS_INFO_STREAM("diff input-output"<<"x="<<pose_out_msg_.pose.orientation.x<<","<<pose_in_msg_.pose.orientation.x);
	ROS_INFO_STREAM("diff input-output"<<"y="<<pose_out_msg_.pose.orientation.y<<","<<pose_in_msg_.pose.orientation.y);
	ROS_INFO_STREAM("diff input-output"<<"z="<<pose_out_msg_.pose.orientation.z<<","<<pose_in_msg_.pose.orientation.z);
	ROS_INFO_STREAM("diff input-output"<<"w="<<pose_out_msg_.pose.orientation.w<<","<<pose_in_msg_.pose.orientation.w);

	tf::Quaternion inq(pose_in_msg_.pose.orientation.x,pose_in_msg_.pose.orientation.y,pose_in_msg_.pose.orientation.z,pose_in_msg_.pose.orientation.w);

	ROS_INFO_STREAM("angle"<<out_or.angle(inq));
	return true;

}


void PoseKalmanFilterThread::callBackPoseIn(const geometry_msgs::PoseStamped::ConstPtr & in)
{
	/*implement callback here.*/

	pose_in_msg_=*in;
	input_flags_["pose_in"]++;
	receivedNewInput(); // When you do not use, you must comment this line
}


bool PoseKalmanFilterThread::activateCommand(ModuleCommand::InternalRequest iq)
{
	activate();
	iq.RESPONSE(OK);
	return true;
}

bool PoseKalmanFilterThread::deactivateCommand(ModuleCommand::InternalRequest iq)
{
	deactivate();
	iq.RESPONSE(OK);
	return true;
}

bool PoseKalmanFilterThread::activateFiltering(ModuleCommand::InternalRequest iq)
{
	/* You sure that the iq.response_->identifier is not empty because this may be useful for caller
	 *  the options for this identifier are defined in std_msg response message
	 */
	lockMutex();
	activate_filtering_=true;
	filterToInit_=true;
	unlockMutex();
	iq.RESPONSE(OK);   // This macro simplify the response
	return true;
};

bool PoseKalmanFilterThread::deactivateFiltering(ModuleCommand::InternalRequest iq)
{
	/* You sure that the iq.response_->identifier is not empty because this may be useful for caller
	 *  the options for this identifier are defined in std_msg response message
	 */
	lockMutex();
	activate_filtering_=false;
	unlockMutex();
	iq.RESPONSE(OK);   // This macro simplify the response
	return true;
};

bool PoseKalmanFilterThread::setupTfPublish(ModuleCommand::InternalRequest iq)
{
	/* You sure that the iq.response_->identifier is not empty because this may be useful for caller
	 *  the options for this identifier are defined in std_msg response message
	 */
	if (iq.command_->getStringValues().size()==2){

		lockMutex();
		string old_base_frame=PARAM(BASE_FRAME);
		string old_target_frame=PARAM(TARGET_FRAME);
		param_server_->set(BASE_FRAME,iq.command_->getStringValues()[0]);
		param_server_->set(TARGET_FRAME,iq.command_->getStringValues()[1]);
		unlockMutex();
		if (T_PARAM(MODE,string)==TF && checkCurrentTfTransform()){

			filterToInit_=true;
			iq.RESPONSE(OK);   // This macro simplify the response
		}
		else
		{
			lockMutex();
			param_server_->set(BASE_FRAME,old_base_frame);
			param_server_->set(TARGET_FRAME,old_target_frame);
			unlockMutex();
			iq.RESPONSE(ERROR);
		}

	}else{
		iq.RESPONSE(ERROR);   // This macro simplify the response
	}
	return true;
};

bool PoseKalmanFilterThread::setMode(ModuleCommand::InternalRequest iq)
{
	/* You sure that the iq.response_->identifier is not empty because this may be useful for caller
	 *  the options for this identifier are defined in std_msg response message
	 */
	if (iq.command_->getStringValues().size()==1 && (iq.command_->getStringValues()[0]==TOPIC || iq.command_->getStringValues()[0]==TF))
	{
		lockMutex();
		param_server_->set(MODE,iq.command_->getStringValues()[0]);

		unlockMutex();
		if (!changeModeBehaviour())
			iq.RESPONSE(ERROR);
		else{
			lockMutex();
			filterToInit_=true;
			iq.RESPONSE(OK);
			unlockMutex();
		}
	}else
		iq.RESPONSE(ERROR);

	return true;
};

void PoseKalmanFilterThread::initKalmanFilter(){
	ROS_INFO_STREAM("INIT FILTER ");

	cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(T_PARAM(COVARIANCE_PROCESS,double)));       // 1e-4 set process noise
	cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(T_PARAM(COVARIANCE_MEASURAMENT,double)));   // set measurement noise
	cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(100));             // error covariance


	predictKalmanFilter( kf_, positionVec_ , rotationMat_);
	//cv::Mat rMat=rotationMat_.clone(); // 3x3
	//cv::Mat pVec=positionVec_.clone(); // 3x1

	//predictKalmanFilter( kf_, pVec , rMat);

	fillMeasurements(measurement_, positionVec_ , rotationMat_);

	updateKalmanFilter( kf_, measurement_, positionVec_ , rotationMat_);


	filterToInit_ = false;
}

void PoseKalmanFilterThread::predictKalmanFilter( cv::KalmanFilter &KF,
		cv::Mat &translation_predicted, cv::Mat &rotation_predicted )
{
	// First predict, to update the internal statePre variable
	cv::Mat predicted = KF.predict();


	// Predicted translation
	translation_predicted.at<double>(0) = predicted.at<double>(0);
	translation_predicted.at<double>(1) = predicted.at<double>(1);
	translation_predicted.at<double>(2) = predicted.at<double>(2);

	// Predicted euler angles
	cv::Mat eulers_predicted(3, 1, CV_64F);
	eulers_predicted.at<double>(0) = predicted.at<double>(9);
	eulers_predicted.at<double>(1) = predicted.at<double>(10);
	eulers_predicted.at<double>(2) = predicted.at<double>(11);

	// Convert predicted quaternion to rotation matrix
	rotation_predicted = euler2rot(eulers_predicted);


}

cv::Mat PoseKalmanFilterThread::euler2rot(const cv::Mat & euler)
{
	cv::Mat rotationMatrix(3,3,CV_64F);
	double x = euler.at<double>(0);
	double y = euler.at<double>(1);
	double z = euler.at<double>(2);
	// Assuming the angles are in radians.
	double ch = cos(z);
	double sh = sin(z);
	double ca = cos(y);
	double sa = sin(y);
	double cb = cos(x);
	double sb = sin(x);
	double m00, m01, m02, m10, m11, m12, m20, m21, m22;
	m00 = ch * ca;
	m01 = sh*sb - ch*sa*cb;
	m02 = ch*sa*sb + sh*cb;
	m10 = sa;
	m11 = ca*cb;
	m12 = -ca*sb;
	m20 = -sh*ca;
	m21 = sh*sa*cb + ch*sb;
	m22 = -sh*sa*sb + ch*cb;
	rotationMatrix.at<double>(0,0) = m00;
	rotationMatrix.at<double>(0,1) = m01;
	rotationMatrix.at<double>(0,2) = m02;
	rotationMatrix.at<double>(1,0) = m10;
	rotationMatrix.at<double>(1,1) = m11;
	rotationMatrix.at<double>(1,2) = m12;
	rotationMatrix.at<double>(2,0) = m20;
	rotationMatrix.at<double>(2,1) = m21;
	rotationMatrix.at<double>(2,2) = m22;
	return rotationMatrix;
}

void PoseKalmanFilterThread:: copyStampedTfToCv(const tf::StampedTransform &pose)
{
	for (int r=0; r<3;r++){
		for (int c=0; c<3;c++){
			rotationMat_.at<double>(r,c) = pose.getBasis()[r][c];
		}
		positionVec_.at<double>(r,0) = pose.getOrigin()[r];
	}

}

void PoseKalmanFilterThread:: copyStampedPoseToCv(const geometry_msgs::PoseStamped & pos)
{
	tf::Transform tr;
	tf::Vector3 position(pos.pose.position.x,pos.pose.position.y,pos.pose.position.z);
	tf::Quaternion orien(pos.pose.orientation.x,pos.pose.orientation.y,pos.pose.orientation.z,pos.pose.orientation.w);


	tr.setOrigin(position);
	tr.setRotation(orien);

	for (int r=0; r<3;r++){
		for (int c=0; c<3;c++){
			rotationMat_.at<double>(r,c) = tr.getBasis()[r][c];
		}
		positionVec_.at<double>(r,0) = tr.getOrigin()[r];
	}


}
void PoseKalmanFilterThread::fillMeasurements( cv::Mat &measurements,
		const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
	// Convert rotation matrix to euler angles
	cv::Mat measured_eulers(3, 1, CV_64F);
	measured_eulers = rot2euler(rotation_measured);
	// Set measurement to predict
	measurements.at<double>(0) = translation_measured.at<double>(0); // x
	measurements.at<double>(1) = translation_measured.at<double>(1); // y
	measurements.at<double>(2) = translation_measured.at<double>(2); // z
	measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
	measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
	measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw


}

cv::Mat PoseKalmanFilterThread::rot2euler(const cv::Mat & rotationMatrix)
{
	cv::Mat euler(3,1,CV_64F);
	double m00 = rotationMatrix.at<double>(0,0);
	double m02 = rotationMatrix.at<double>(0,2);
	double m10 = rotationMatrix.at<double>(1,0);
	double m11 = rotationMatrix.at<double>(1,1);
	double m12 = rotationMatrix.at<double>(1,2);
	double m20 = rotationMatrix.at<double>(2,0);
	double m22 = rotationMatrix.at<double>(2,2);
	double x, y, z;
	// Assuming the angles are in radians.
	if (m10 > 0.998) { // singularity at north pole
		x = 0;
		y = CV_PI/2;
		z = atan2(m02,m22);
	}
	else if (m10 < -0.998) { // singularity at south pole
		x = 0;
		y = -CV_PI/2;
		z = atan2(m02,m22);
	}
	else
	{
		x = atan2(-m12,m11);
		y = asin(m10);
		z = atan2(-m20,m00);
	}
	euler.at<double>(0) = x;
	euler.at<double>(1) = y;
	euler.at<double>(2) = z;
	return euler;
}

void PoseKalmanFilterThread::updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
		cv::Mat &translation_estimated, cv::Mat &rotation_estimated )
{


	// The "correct" phase that is going to use the predicted value and our measurement
	cv::Mat estimated = KF.correct(measurement);

	// Estimated translation
	translation_estimated.at<double>(0) = estimated.at<double>(0);
	translation_estimated.at<double>(1) = estimated.at<double>(1);
	translation_estimated.at<double>(2) = estimated.at<double>(2);

	// Estimated euler angles
	cv::Mat eulers_estimated(3, 1, CV_64F);
	eulers_estimated.at<double>(0) = estimated.at<double>(9);
	eulers_estimated.at<double>(1) = estimated.at<double>(10);
	eulers_estimated.at<double>(2) = estimated.at<double>(11);

	// Convert estimated quaternion to rotation matrix
	rotation_estimated = euler2rot(eulers_estimated);

}

bool PoseKalmanFilterThread::getCurrentTfTransform()
{

	try{
		tf_listener_->lookupTransform(PARAM(BASE_FRAME) , PARAM(TARGET_FRAME),ros::Time(0), current_pose_);
		return true;
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("ERROR: reading TF %s : %s",T_PARAM(TARGET_FRAME,string).c_str(), ex.what());
		return false;
	}

	return true;
}

bool PoseKalmanFilterThread::checkCurrentTfTransform()
{

	try{
		tf_listener_->waitForTransform(PARAM(BASE_FRAME) ,PARAM(TARGET_FRAME),ros::Time(0),ros::Duration(1) );
		tf_listener_->lookupTransform(PARAM(BASE_FRAME) , PARAM(TARGET_FRAME),ros::Time(0), current_pose_);
		return true;
	}
	catch (tf::TransformException &ex){
		ROS_ERROR("ERROR: reading TF %s : %s",T_PARAM(TARGET_FRAME,string).c_str(), ex.what());
		return false;
	}

	return true;
}

void PoseKalmanFilterThread::getPoseFromCv(  tf::Vector3 &pos,tf::Quaternion &q)
{
	tf::Matrix3x3 m;
	for (int r=0; r<3;r++){
		for (int c=0; c<3;c++){
			m[r][c] =  rotationMat_.at<double>(r,c);
		}
		pos[r] = positionVec_.at<double>(r,0);
	}
	m.getRotation(q);
}
