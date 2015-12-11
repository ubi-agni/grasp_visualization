#include "ros/ros.h"
#include "moveit_msgs/Grasp.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "agni_grasp_viewer/DisplayGrasps.h"


ros::Publisher pub;

bool display_grasp(agni_grasp_viewer::DisplayGrasps::Request  &req,
         agni_grasp_viewer::DisplayGrasps::Response &res)
{
  res.success = true;
  
  const std::vector<moveit_msgs::Grasp> grasps = req.grasps;
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(grasps.size());  
  
  visualization_msgs::Marker marker; 
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;


  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;
  marker.color.a = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.ns = "grasp";

  for (size_t i=0; i < grasps.size(); ++i)
  {
    marker.header.frame_id = "base_link";
    marker.pose = grasps[i].grasp_pose.pose;
    marker.id = 1000000 + i;    
    marker.ns = "grasp";
    if (grasps[i].id.find("LeftArm") != std::string::npos) 
    {
      marker.color.a = 0.3;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      if (grasps[i].id.find("TwoFingerPrecision") != std::string::npos)
        marker.mesh_resource = "package://agni_grasp_manager/models/shadow_hand/LeftTwoFingerPrecision.stl";
      else
        marker.mesh_resource = "package://agni_grasp_manager/models/shadow_hand/LeftAllFingerPrecision.stl";
    }
    else
    {
      marker.color.a = 0.3;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      if (grasps[i].id.find("TwoFingerPrecision") != std::string::npos)
        marker.mesh_resource = "package://agni_grasp_manager/models/shadow_hand/RightTwoFingerPrecision.stl";
      else
        marker.mesh_resource = "package://agni_grasp_manager/models/shadow_hand/RightAllFingerPrecision.stl";
    }
    
    if (grasps[i].grasp_quality == -1.0)
    {
      marker.ns = "badgrasp";
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    
    marker_array.markers[i] = marker;
  }
  pub.publish(marker_array);
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_viz");
  ros::NodeHandle n;

  pub = n.advertise<visualization_msgs::MarkerArray>("grasp_marker_array", 10);
  ros::ServiceServer service = n.advertiseService("display_grasp", display_grasp);
  ROS_INFO("Ready to  display grasps.");
  ros::spin();

  return 0;
}
