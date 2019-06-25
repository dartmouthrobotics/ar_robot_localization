/**
*
* \author     Monika Roznere <mroznere@gmail.com>
* \copyright  Copyright (c) 2019, Dartmouth Robotics Lab.
*
*/

#ifndef BLUEROV_VISUAL_POSE_VISUAL_POSE_H
#define BLUEROV_VISUAL_POSE_VISUAL_POSE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/VFR_HUD.h>


namespace bluerov_visual_pose
{

/*!
  Class that handles sending and receiving messages from the ping echo sounder.
  Publishes distance and confidence data.
*/
class VisualPose
{
  public:
    /*!
      Receives the nodehandler and nodelet name from the nodelet initializer
    */
    VisualPose(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name) {}
    ~VisualPose() {}

    bool init()
    {
      /*! Create rostopic to publish distance and confidence */
      pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/visual_pose/pose", 1000);
      sub_ = nh_.subscribe("/mavros/vfr_hud", 10, &VisualPose::depthCb, this);

      // std::cout << "hello in init" << std::endl;

      return true;
    }


    void spin()
    {
      ros::Rate rate(30);

      while (nh_.ok())
      {
        if (depth_detected)
        {
          broadcast_new_frame(); // publish a new frame; currently just the depth
          pub_robot_pose(); // publish the robot pose to a rostopic
        }

        rate.sleep();
        ros::spinOnce();
      }
    }


    void depthCb(const mavros_msgs::VFR_HUD message)
    {
      depth = message.altitude;
      depth_detected = true;
    }


    void pub_robot_pose()
    {
      std::string ar_marker_frame("/ar_marker_14");
      std::string depth_frame("/depth");
      tf::StampedTransform ar_in_base_link_frame;
      tf::StampedTransform depth_in_base_link_frame;

      try
      {
          tl_.lookupTransform("/base_link", ar_marker_frame, ros::Time(0), ar_in_base_link_frame);
          auto translation_ar = ar_in_base_link_frame.getOrigin();
          auto rotation_ar = ar_in_base_link_frame.getRotation();

          tl_.lookupTransform("/base_link", depth_frame, ros::Time(0), depth_in_base_link_frame);
          auto translation_depth = depth_in_base_link_frame.getOrigin();
          auto rotation_depth = depth_in_base_link_frame.getRotation();

          geometry_msgs::PoseStamped pose_message;
          pose_message.header.stamp = ros::Time::now();
          pose_message.header.frame_id = "base_link";

          pose_message.pose.position.x = translation_ar.x();
          pose_message.pose.position.y = translation_ar.y();
          pose_message.pose.position.z = translation_depth.z();

          pose_message.pose.orientation.x = rotation_ar[0];
          pose_message.pose.orientation.z = rotation_ar[2];
          pose_message.pose.orientation.y = rotation_ar[1];
          pose_message.pose.orientation.w = rotation_ar[3];

          pub_.publish(pose_message);
      }
      catch (tf::TransformException error)
      {
          // ROS_ERROR("%s", error.what());
      }
    }


    void broadcast_new_frame()
    {
      if (nh_.ok())
      {
        // std::string ar_world_frame("/ar_marker_14");
        tf::StampedTransform depth_in_base_link_frame;

        try
        {
            // tl_.lookupTransform("/base_link", ar_world_frame, ros::Time(0), ar_in_base_link_frame);
            // auto translation = ar_in_base_link_frame.getOrigin();
            // auto rotation = ar_in_base_link_frame.getRotation();
            //
            // // std::cout << "create world frame" << std::endl;
            // trans.setOrigin( tf::Vector3(0.0, 0.0, depth + translation.z()) );
            // trans.setRotation( rotation );
            // std::cout << depth << std::endl;
            trans.setOrigin( tf::Vector3(0.0, 0.0, depth) );
            trans.setRotation( tf::Quaternion(0, 0, 0, 1) );
            br_.sendTransform( tf::StampedTransform(trans, ros::Time::now(), "/base_link", "/depth") );

            tag_detected = true;
        }
        catch (tf::TransformException error)
        {
            // ROS_ERROR("%s", error.what());
        }
      }
    }

  private:
    ros::NodeHandle nh_;  /**< node handler used for publishing ping data */
    ros::Publisher pub_;
    ros::Subscriber sub_;
    tf::TransformListener tl_;
    tf::TransformBroadcaster br_;
    tf::Transform trans;

    bool tag_detected = false; /** NOT USED **/
    bool depth_detected = false;
    float depth;

    std::string name_;    /**< name of the nodelet (not used) */
};

}   // namespace bluerov_visual_pose

#endif    // BLUEROV_VISUAL_POSE_VISUAL_POSE_H
