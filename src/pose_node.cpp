/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include "ros/ros.h"
#include "nodelet/loader.h"
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  std::cout << nodelet_name << std::endl;

  nodelet.load(nodelet_name, "bluerov_visual_pose/VisualPoseNodelet", remap, nargv);
  ros::spin();

  return 0;
}
