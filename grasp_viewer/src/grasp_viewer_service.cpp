/* Author: Guillaume Walck */

#include "ros/ros.h"
#include "moveit_msgs/Grasp.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "grasp_viewer/DisplayGrasps.h"
#include "grasp_viewer/grasp_viewer.h"

GraspViewer *grasp_view;
ros::Publisher pub;
std::string left_arm_name;
std::string right_arm_name;
std::string left_hand_ee;
std::string right_hand_ee;
std::string grasp_frame;

void delete_all_grasps()
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.id = 1;
  marker_array.markers.push_back(marker);
  pub.publish(marker_array);
  return;
}


bool display_grasp(grasp_viewer::DisplayGrasps::Request  &req,
                                grasp_viewer::DisplayGrasps::Response &res)
{
  res.success = true;
  
  const std::vector<moveit_msgs::Grasp> grasps = req.grasps;
  visualization_msgs::MarkerArray marker_array;
  
  // clear previously displayed markers
  delete_all_grasps();
  
  // display new markers
  std::string ee_name;
  for (size_t i=0; i < std::min(5, (int)grasps.size()); ++i)
  {
    // In grasp id, find which ee_name depending on which arm name
    if (grasps[i].id.find(left_hand_ee) != std::string::npos) 
    {
      ee_name = left_hand_ee;
    }
    else {
      if (grasps[i].id.find(right_hand_ee) != std::string::npos) 
      {
        ee_name = right_hand_ee;
      }
      else
      {
        ROS_WARN("No indication about end-effector name in grasp.id, no end-effector can be selected for grasp %zd.", i);
        return false;
      }
    }
    if(!grasp_view->createGraspMarker(ee_name, grasps[i], marker_array, req.color, grasp_frame))
    {
      ROS_WARN("Failed to create grasp marker for grasp %zd.", i);
    }
  }
  pub.publish(marker_array);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_viz");
  ros::NodeHandle n, n_tilde("~");

  grasp_frame = "grasp_frame";
  if (n_tilde.hasParam("grasp_frame"))
  {
    n_tilde.getParam("grasp_frame", grasp_frame);
    ROS_INFO_STREAM("Using grasp frame named " << grasp_frame);
  }
  else
  {
    ROS_INFO_STREAM("Did not find ~grasp_frame" << grasp_frame);
  }
  
  // get arm and hand names
  n_tilde.param<std::string>("left_arm_name", left_arm_name, "LeftArm");
  ROS_INFO_STREAM("Using left_arm named " << left_arm_name);
  n_tilde.param<std::string>("left_hand_ee", left_hand_ee, "left_hand");
  ROS_INFO_STREAM("Using left_hand_ee named " << left_hand_ee);
  n_tilde.param<std::string>("right_hand_ee", right_hand_ee, "right_hand");
  ROS_INFO_STREAM("Using right_hand_ee named " << right_hand_ee);

  grasp_view = new GraspViewer(n.getNamespace());
  if(!grasp_view->init())
  {
    delete grasp_view;
    ROS_ERROR("Failed to init");
    return -1;
  }
  
  pub = n.advertise<visualization_msgs::MarkerArray>("grasp_marker_array", 10);
  ros::ServiceServer service = n.advertiseService("display_grasp", display_grasp);
  ROS_INFO("Ready to  display grasps.");
  ros::spin();

  return 0;
}
