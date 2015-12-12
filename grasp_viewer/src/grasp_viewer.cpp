/* Author: Guillaume Walck */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include "grasp_viewer/grasp_viewer.h"
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

GraspViewer::GraspViewer(const std::string ns)
{
  ns_ = ns;
}

bool GraspViewer::init()
{
  std::string robot_description = ns_ + "/robot_description";
  
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model_loader.reset(new robot_model_loader::RobotModelLoader(robot_description, false));
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();   
  if (!robot_model) {
    ROS_ERROR("Error Loading robot model");
    return false;
  }
  robot_state_.reset(new moveit::core::RobotState(robot_model));
  return true;
}


bool GraspViewer::createGraspMarker(const std::string ee_name, const moveit_msgs::Grasp &grasp, visualization_msgs::MarkerArray &marker_array)
{
  // check existence of end-effector
  if(!robot_state_->getRobotModel()->hasEndEffector(ee_name))
  { 
    ROS_WARN("This end-effector is not part of the model" );
    return false;
  }
  // retrieve joint_names from grasp posture and check if part of end-effector
  const std::vector<std::string> &joint_names = grasp.grasp_posture.joint_names;
  for(size_t i = 0; i < joint_names.size(); ++i)
  {
    if (!robot_state_->getRobotModel()->getEndEffector(ee_name)->hasJointModel(joint_names[i]))
    {
      ROS_WARN_STREAM("joint " <<  joint_names[i] << " is not part of end-effector " << ee_name );
      return false;
    }
  }
  // get all link names of the end-effector
  std::vector<std::string> link_names = robot_state_->getRobotModel()->getEndEffector(ee_name)->getLinkModelNames();
 
  // update the state to the grasp posture 
  size_t traj_size = grasp.grasp_posture.points.size();
  if (traj_size > 0)
  {
    // take the last point of the trajectory
    const std::vector<double> &positions = grasp.grasp_posture.points[traj_size-1].positions;
    robot_state_->setVariablePositions(joint_names, positions);
  }
  else
  {
    ROS_WARN("Empty grasp posture trajectory point" );
    return false;
  }
  robot_state_->update();

  // get the markers
  //link_names.clear();
  //link_names.push_back("palm");
  std_msgs::ColorRGBA color;
  color.a = 0.3;
  color.b = 1.0;
  
  size_t cur_num = marker_array.markers.size();

  visualization_msgs::Marker marker;
  marker.pose = grasp.grasp_pose.pose;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = grasp.grasp_pose.header.frame_id;
  marker.ns = "arrows";
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.005;
  marker.scale.z = 0.005;
  marker.id = ++cur_num;
  marker_array.markers.push_back(marker);
  
  robot_state_->getRobotMarkers(marker_array,link_names, color, "grasp", ros::Duration(0));
  Eigen::Affine3d tf_link_to_root;
  std::string grasp_link = "grasp_frame";
  // check if grasp_frame exist
  if(!robot_state_->getRobotModel()->getEndEffector(ee_name)->hasLinkModel(grasp_link))
  {
    // get commont root 
    grasp_link = robot_state_->getRobotModel()->getEndEffector(ee_name)->getCommonRoot()->getChildLinkModel()->getName();
    ROS_INFO_STREAM("using '" << grasp_link << "' as grasp link");
    tf_link_to_root = robot_state_->getGlobalLinkTransform(grasp_link).inverse();
  }
  else
  {
    tf_link_to_root = robot_state_->getGlobalLinkTransform(grasp_link).inverse();
  }
  // compute offset transforms from grasp pose to marker
  Eigen::Affine3d tf_grasp;
  tf::poseMsgToEigen(grasp.grasp_pose.pose, tf_grasp);
  for(size_t i = cur_num; i < marker_array.markers.size() ; ++i)
  {
    marker_array.markers[i].header.frame_id = grasp.grasp_pose.header.frame_id;
    Eigen::Affine3d tf_root_to_mesh;
    tf::poseMsgToEigen(marker_array.markers[i].pose, tf_root_to_mesh);
    tf::poseEigenToMsg(tf_grasp * tf_link_to_root * tf_root_to_mesh, marker_array.markers[i].pose);
  }

  if (marker_array.markers.empty()) return false;
    return true;
}



