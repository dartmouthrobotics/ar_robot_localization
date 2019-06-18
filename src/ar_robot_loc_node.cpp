#include "ros/ros.h"
#include "nodelet/loader.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ar_robot_loc_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "ar_robot_localization/ar_robot_loc_nodelet", remap, nargv);
  ros::spin();

  return 0;
}
