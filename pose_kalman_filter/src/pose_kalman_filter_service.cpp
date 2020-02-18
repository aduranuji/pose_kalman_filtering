#include <pose_kalman_filter_service.h>

PoseKalmanFilterService :: PoseKalmanFilterService():ServiceBase()
{
  th_=NULL;
  
}

PoseKalmanFilterService :: ~PoseKalmanFilterService()
{
  if (th_!=NULL){
    th_->stop();
    delete th_;
  }
  
}

bool PoseKalmanFilterService :: command(service_base::std_msg::Request & request,service_base::std_msg::Response & response)
{

  /* OVERRIDE THIS METHOD  */
  /* This method must be override to process service request.  By other
   * hand in this method is generated response service message */
   
   

  return true;
}

bool PoseKalmanFilterService :: configureService(int argc,char **argv)
{
  // Here you must configure service
  
  th_=new  PoseKalmanFilterThread("~");
  // start the thread
  th_->setMutex(mutex_);
  th_->setParameterServer(module_parameters_);
  th_->setEventList();
  
  
    if (!module_parameters_->parse("loop_rate"))
    {
        ROS_ERROR_STREAM("Error to parse parameter:loop_rate");
        return false;
    }
            
	
    if (!module_parameters_->parse("base_frame"))
    {
        ROS_ERROR_STREAM("Error to parse parameter:base_frame");
        return false;
    }
            
	
    if (!module_parameters_->parse("target_frame"))
    {
        ROS_ERROR_STREAM("Error to parse parameter:target_frame");
        return false;
    }
            
	
    if (!module_parameters_->parse("mode"))
    {
        ROS_ERROR_STREAM("Error to parse parameter:mode");
        return false;
    }
            
	
    if (!module_parameters_->parse("covariance_measurament"))
    {
        ROS_ERROR_STREAM("Error to parse parameter:covariance_measurament");
        return false;
    }
            
	
    if (!module_parameters_->parse("covariance_process"))
    {
        ROS_ERROR_STREAM("Error to parse parameter:covariance_process");
        return false;
    }
            
	
    if (!module_parameters_->parse("pose_filtered_id"))
    {
        ROS_ERROR_STREAM("Error to parse parameter:pose_filtered_id");
        return false;
    }
            
    if (!module_parameters_->parse("frame_id_check"))
        {
            ROS_ERROR_STREAM("Error to parse parameter: frame_id_check");
            return false;
        }
  /* SERVICE DEFAULT COMMANDS (ADD AUTOMATICALLY WHEN SERVICE IS CREATED */
  
    CREATE_MODULE_COMMAND_COMPUTE_SCOPE("activate",PoseKalmanFilterThread,activateCommand);
	
    CREATE_MODULE_COMMAND_COMPUTE_SCOPE("deactivate",PoseKalmanFilterThread,deactivateCommand);
  
  /* USER COMMAND DEFINITIONS*/
  
    CREATE_MODULE_COMMAND_COMPUTE_SCOPE("activate_filtering",PoseKalmanFilterThread,activateFiltering);
	
    CREATE_MODULE_COMMAND_COMPUTE_SCOPE("deactivate_filtering",PoseKalmanFilterThread,deactivateFiltering);
	
    CREATE_MODULE_COMMAND_COMPUTE_SCOPE("setup_tf_publish",PoseKalmanFilterThread,setupTfPublish);
	
    CREATE_MODULE_COMMAND_COMPUTE_SCOPE("set_mode",PoseKalmanFilterThread,setMode);
  
  
    th_->activate();
  th_->start();
  
  
  return true; 
}
