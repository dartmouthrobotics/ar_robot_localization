/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

#include <bluerov_visual_pose/visual_pose.h>

namespace bluerov_visual_pose
{

class VisualPoseNodelet : public nodelet::Nodelet
{
public:
  VisualPoseNodelet() {}
  ~VisualPoseNodelet() {}

  virtual void onInit()
  {
    ros::NodeHandle nh = this->getPrivateNodeHandle();

    std::string name = nh.getUnresolvedNamespace();
    int pos = name.find_last_of('/');
    name = name.substr(pos+1);

    NODELET_INFO_STREAM("Initializing nodelet... [" << name << "]");
    controller_.reset(new VisualPose(nh, name));

    if (controller_->init())
    {
      NODELET_INFO_STREAM("Nodelet initialized. [" << name << "]");
      controller_->spin();
    }
    else
    {
      NODELET_ERROR_STREAM("Could not initialize nodelet! Please restart. [" << name << "]");
    }
  }

private:
  boost::shared_ptr<VisualPose> controller_;
};

}  // namespace bluerov_visual_pose

PLUGINLIB_EXPORT_CLASS(bluerov_visual_pose::VisualPoseNodelet, nodelet::Nodelet);
