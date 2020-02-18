#ifndef POSE_KALMAN_FILTER_SERVICE_H
#define POSE_KALMAN_FILTER_SERVICE_H

#include <service_base.h>
#include <pose_kalman_filter_thread.h>


using namespace std;
using namespace services;

/**
* @class PoseKalmanFilterService
* @file pose_kalman_filter_service.h
* @brief The objects of this class are the base  to launch the threads. This class has several methods to change running configuration dinamically 
*/


class PoseKalmanFilterService:public ServiceBase
{
 private:
  /// Thread which manage the input and output flows
	PoseKalmanFilterThread * th_;


 protected:

	/* Override
	 * update service in each loop
	 */
	 
	 /**
	 * Override this method to do something when one loop is finished
	 * \return If it returns True the service goes on running. If it returns False the service will be stopped.
	 */
	bool updateService(){return true;};

	/*
	 * @Override
	 * Override respond function to define respond actions from entering messages
	 */
	/**
	*  \deprecated
	* This method is used by compatibility with older versions. You can parse commands from here
	* but it is not recommended. It's better use module_commands_ map, and macros con 
	* The actions when a request is arrived to the services, are defined in this method.
	* \param request it is a message that it arrives to service.
	* \param response it is a message that the service answers
	* \return If all is correct response method must be return a True. If it is False the service loop will be interrupted
	*/
	 bool command(service_base::std_msg::Request & request,service_base::std_msg::Response & response);

	/*
	 * @Override
	 * Override configure methods to define enviroment variables
	 */
	/**
	* This method parse the initial parameters and configures the service when it is started
	* \param argc number of the command line parameters
	* \param argv command line parameters
	*/
	bool configureService(int argc, char ** argv);
public:
    /**
    * Class constructor to support ros node which handles the commands requests.
    * You can modify this method if you need create some objects when the node are going to start
    */
	PoseKalmanFilterService();
	/**
	* class destructor. You must destroy all objects that you have created in constructor
	*/
	~PoseKalmanFilterService();
	

};



#endif /* POSE_KALMAN_FILTER_SERVICE_H */
