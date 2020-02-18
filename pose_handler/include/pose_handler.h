#ifndef POSE_HANDLER_H
#define POSE_HANDLER_H

#include <string.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>

class PoseHandler
{
 protected:
  std::string base_;
  std::string frame_;

  /** Listen to the pose of the frame wrt the base*/
  tf::TransformListener *pose_listener_; 
  /** send the displacement*/
  tf::TransformBroadcaster tf_broadcaster_;

  /** Previous pose of the frame (4x4 matrix)*/
  //  cv::Mat prev_pose_;
  tf::StampedTransform prev_pose_;

  /** Current pose of the frame (4x4 matrix)*/
  // cv::Mat curr_pose_;
  tf::StampedTransform curr_pose_;

  /** Displacement between the previous and the cuerrent frame (4x4 matrix). */ 
  //cv::Mat displ_;
  tf::StampedTransform displ_;

  /** It is a 6d vector composed by three rotations and three translations. */  
  std::vector<float> vel_;

  /** stamp of the current frame */
  //  ros::Time stamp_;
 public:
  PoseHandler();//(std::string base, std::string frame);

  /** set the name of the base and of the child frames adn init the displacemente transform */ 
  void setBaseAndFrame(std::string base, std::string frame);
  
  void stampedTransform2cvMat( tf::StampedTransform &ros_pose,  cv::Mat &cv_pose);

  /** get the pose of the "frame" wrt the "base"  at the time "stamp"
      and compute the displacement between the previous and the current pose
   */
  bool getCurrentPose(ros::Time stamp);
  
  /** get a generic pose without change the internal state of the class */
  bool getPose(ros::Time stamp, std::string base, std::string frame,   tf::StampedTransform &pose);

  /** return the displacement between the previous and the current frame*/
  //  inline cv::Mat& getDisplacement (){return displ_;};

  /** fill the 6d velocity vector with data from displ_*/ 
  std::vector<float> getVelocity();

  /** Publish the stamped transform with the displacement of the frame*/
  void publishDisplacement();

  /** Read the displacement of the pose from the tf */
  bool readDisplacement(ros::Time stamp);

  /** get the distance of the displacement*/
  float getLengthDispl();

  /** convert a stamped transform to a pose stamped */
  void transf2Pose(tf::StampedTransform &transf,geometry_msgs::PoseStamped &pose_stamp );

};
#endif// POSE_HANDLER_H
