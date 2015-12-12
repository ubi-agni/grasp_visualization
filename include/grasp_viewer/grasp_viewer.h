/* Author: Guillaume Walck */

#ifndef GRASP_VIEWER_H
#define GRASP_VIEWER_H

#include <string>
#include <moveit_msgs/Grasp.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/robot_state/robot_state.h>

class GraspViewer
{
  public:
      GraspViewer(const std::string ns);
      bool init();
      bool createGraspMarker(const std::string ee_name, const moveit_msgs::Grasp &grasp, visualization_msgs::MarkerArray &marker_array);
      bool getGraspMarkerMesh(const moveit_msgs::Grasp &grasp, visualization_msgs::MarkerArray &marker_array);
      
  private:
      std::string ns_;
      std::string root_link_;
      moveit::core::RobotStatePtr robot_state_;

};

#endif
