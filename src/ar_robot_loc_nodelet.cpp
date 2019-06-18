#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/PoseStamped.h"
#include "pluginlib/class_list_macros.h"

#include "ar_robot_localization/ar_robot_loc_nodelet.h"

namespace ar_robot_localization
{
  void ArRobotLocNodelet::onInit()
  {
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    timer_ = nh_.createTimer(ros::Duration(1.0), boost::bind(&ArRobotLocNodelet::timerCb, this, _1));
    sub_ = nh_.subscribe("ar_chatter", 10, &ArRobotLocNodelet::poseCb, this);
    pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  };

  void ArRobotLocNodelet::poseCb(const ar_track_alvar_msgs::AlvarMarkers message)
  {
    geometry_msgs::PoseStamped new_message;
    pub_.publish(new_message);
  }

  void ArRobotLocNodelet::timerCb(const ros::TimerEvent& event)
  {
    NODELET_INFO_STREAM("The time is now " << event.current_real);
  }

} // namespace ar_robot_localization

PLUGINLIB_EXPORT_CLASS(ar_robot_localization::ArRobotLocNodelet, nodelet::Nodelet);
