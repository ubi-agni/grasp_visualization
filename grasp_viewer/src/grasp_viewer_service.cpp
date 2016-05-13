/* Author: Guillaume Walck */

#include "ros/ros.h"
#include "moveit_msgs/Grasp.h"
#include "visualization_msgs/MarkerArray.h"
#include "grasp_viewer/DisplayGrasps.h"
#include "grasp_viewer/grasp_viewer.h"

GraspViewer *grasp_view;
ros::Publisher pub;


bool display_grasp(grasp_viewer::DisplayGrasps::Request  &req,
                                grasp_viewer::DisplayGrasps::Response &res)
{
  res.success = true;
  
  const std::vector<moveit_msgs::Grasp> grasps = req.grasps;
  visualization_msgs::MarkerArray marker_array;
  
  std::string ee_name;
  for (size_t i=0; i < std::min(5, (int)grasps.size()); ++i)
  {
    if (grasps[i].id.find("LeftArm") != std::string::npos) 
      ee_name = "left_hand";
    else
      ee_name = "right_hand";
    
    if(!grasp_view->createGraspMarker(ee_name, grasps[i], marker_array, req.color))
    {
      ROS_WARN("Failed to create grasp marker for grasp %d.", i);
    }
  }
  pub.publish(marker_array);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_viz");
  ros::NodeHandle n, n_tilde("~");

  std::string grasp_frame = "grasp_frame";
  if (n_tilde.hasParam("grasp_frame"))
  {
    n_tilde.getParam("grasp_frame", grasp_frame);
    ROS_INFO_STREAM("Using grasp frame named " << grasp_frame);
  }
  else
  {
    ROS_INFO_STREAM("Did not find ~grasp_frame" << grasp_frame);
  }

  grasp_view = new GraspViewer(n.getNamespace(), grasp_frame);
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
