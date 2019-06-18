#ifndef AR_ROBOT_LOCALIZATION_AR_ROBOT_LOCALIZATION_NODELET_H
#define AR_ROBOT_LOCALIZATION_AR_ROBOT_LOCALIZATION_NODELET_H

#include "ros/ros.h"
// #include "boost/unordered_map.hpp"
#include "nodelet/nodelet.h"

namespace ar_robot_localization
{
  class ArRobotLocNodelet : public nodelet::Nodelet
  {
  public:
    ArRobotLocNodelet(){};
    ~ArRobotLocNodelet(){}

  private:
    virtual void onInit();
    void poseCb(const ar_track_alvar_msgs::AlvarMarkers message);
    void timerCb(const ros::TimerEvent& event);

    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Timer timer_;
  };
} // namespace ar_robot_localization

#endif  // AR_ROBOT_LOCALIZATION_AR_ROBOT_LOCALIZATION_NODELET_H
