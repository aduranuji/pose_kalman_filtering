#include <pose_handler.h>

PoseHandler::PoseHandler()//std::string base, std::string frame)
{
  base_  = "";//base;
  frame_ = "";//frame;
  
  // prev_pose_ =cv::Mat::eye(4,4,CV_32F);
  // curr_pose_ =cv::Mat::eye(4,4,CV_32F);
  // displ_ =cv::Mat::eye(4,4,CV_32F);

  vel_.resize(6);

  pose_listener_= new tf::TransformListener();

  
}

void  PoseHandler ::setBaseAndFrame(std::string base, std::string frame){
    base_  = base;
    frame_ = frame; 
    displ_.frame_id_ = base_; 
    std::stringstream ss;
    ss << frame <<"_displacement";    
    displ_.child_frame_id_= ss.str();
}


void PoseHandler :: stampedTransform2cvMat( tf::StampedTransform &ros_pose,  cv::Mat &cv_pose)
{
  for (int r=0; r<3;r++){
    for (int c=0; c<3;c++){
      cv_pose.at<float>(r,c) = ros_pose.getBasis()[r][c];
    }
    cv_pose.at<float>(r,3) = ros_pose.getOrigin()[r];
  }
}

bool PoseHandler :: getCurrentPose(ros::Time stamp){
  tf::StampedTransform pose;
  try{
    // read the current pose
    pose_listener_->waitForTransform( base_,frame_, stamp, ros::Duration(0.03) );
    pose_listener_->lookupTransform( base_,frame_, stamp, pose);    

  }    
  catch (tf::TransformException ex){    
    ROS_ERROR("%s",ex.what());
    //       ros::Duration(1.0).sleep();
    return false;
  }
  // save the timestamp
  //  stamp_ = stamp;

  // save the previous pose
  //prev_pose_ = curr_pose_.clone();  
  prev_pose_ = curr_pose_;//.clone();  
  curr_pose_ = pose;
  // copy the pose to the current pose (opencv mat)
  // stampedTransform2cvMat( pose, curr_pose_);
  displ_.setData(prev_pose_.inverse()*curr_pose_);// stamp_, base_, "/displacement");
  displ_.stamp_ = curr_pose_.stamp_;  
  return true;
}


bool PoseHandler :: getPose(ros::Time stamp, std::string base, std::string frame,   tf::StampedTransform &pose){
  //  tf::StampedTransform pose;
  try{
    // read the current pose
    pose_listener_->waitForTransform( base,frame, stamp, ros::Duration(0.03) );
    pose_listener_->lookupTransform( base,frame, stamp, pose);    

  }    
  catch (tf::TransformException ex){    
    ROS_ERROR("%s",ex.what());
    //       ros::Duration(1.0).sleep();
    return false;
  }
  return true;
}



std::vector<float> PoseHandler :: getVelocity()
{
  vel_[0] = -displ_.getBasis()[1][2]; //wx
  vel_[1] =  displ_.getBasis()[0][2]; //wy
  vel_[2] = -displ_.getBasis()[0][1]; //wz
  vel_[3] =  displ_.getOrigin()[0];   //vx 
  vel_[4] =  displ_.getOrigin()[1];   //vy
  vel_[5] =  displ_.getOrigin()[2];   //vz  
  return vel_;
}

float PoseHandler ::  getLengthDispl(){
  return  displ_.getOrigin().length();
}

void PoseHandler::publishDisplacement()
{
//   tf::StampedTransform stamp_tf;
//   stamp_tf.frame_id_ = base_;
//   stamp_tf.stamp_ = stamp_;
//   stamp_tf.child_frame_id_ = frame_;
//   // filtered_tf_[i].setOrigin(pos_p);   
//   // filtered_tf_[i].setRotation(q_p);        
  tf_broadcaster_.sendTransform(displ_);  
 }


bool PoseHandler::readDisplacement(ros::Time stamp){
  tf::StampedTransform pose;
  try{
    // read the current pose
    pose_listener_->waitForTransform( base_,displ_.child_frame_id_, stamp, ros::Duration(0.03) );
    pose_listener_->lookupTransform( base_,displ_.child_frame_id_, stamp, pose);    
    
  }    
  catch (tf::TransformException ex){    
    ROS_ERROR("%s",ex.what());
    //       ros::Duration(1.0).sleep();
    return false;
  }
  displ_ =  pose;
  return true;
}


void PoseHandler::transf2Pose(tf::StampedTransform &transf,geometry_msgs::PoseStamped &pose_stamp ){
  pose_stamp.pose.position.x = transf.getOrigin().x();
  pose_stamp.pose.position.y = transf.getOrigin().y() ;
  pose_stamp.pose.position.z = transf.getOrigin().z() ;
  pose_stamp.pose.orientation.x = transf.getRotation().x();
  pose_stamp.pose.orientation.y = transf.getRotation().y();
  pose_stamp.pose.orientation.z = transf.getRotation().z();
  pose_stamp.pose.orientation.w = transf.getRotation().w();
  pose_stamp.header.stamp = ros::Time::now();
  pose_stamp.header.frame_id = "/base";//target_frame ;

}

