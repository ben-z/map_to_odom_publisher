/****
 * MapToOdomPublisher
 * 
 * Receives pose of robot in map frame and
 * publishes the map -> odom tf, as per tf
 * conventions (https://www.ros.org/reps/rep-0105.html) 
 * 
 * Note this file is a modified / cut down version of 
 * https://github.com/ros-planning/navigation/blob/noetic-devel/fake_localization/fake_localization.cpp
 * Basically I took the original and removed everything apart from
 * the map -> odom related stuff.
 * 
 * **/
 
#include <ros/ros.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>

#include <angles/angles.h>

#include "ros/console.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class MapToOdomPublisher
{
  public:
    MapToOdomPublisher(void)
    {
      m_tfServer = new tf2_ros::TransformBroadcaster();
      m_tfBuffer = new tf2_ros::Buffer();
      m_tfListener = new tf2_ros::TransformListener(*m_tfBuffer);

      ros::NodeHandle private_nh("~");
      private_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
      private_nh.param("base_frame_id", base_frame_id_, std::string("base_link")); 
      private_nh.param("global_frame_id", global_frame_id_, std::string("map"));     
      private_nh.param("transform_tolerance", transform_tolerance_, 0.1);      
      std::string transform_stamped_topic_;
      private_nh.param("transform_stamped_topic", transform_stamped_topic_, std::string(""));
      std::string odom_topic_;
      private_nh.param("odom_topic", odom_topic_, std::string(""));
      private_nh.param("publish_frequency", publish_freq_, 50.0);
      ros::NodeHandle nh;

      ROS_INFO("odom_frame_id: %s", odom_frame_id_.c_str());
      ROS_INFO("base_frame_id: %s", base_frame_id_.c_str());
      ROS_INFO("global_frame_id: %s", global_frame_id_.c_str());
      ROS_INFO("transform_tolerance: %f", transform_tolerance_);

      if (!transform_stamped_topic_.empty()) {
        ROS_INFO("Using pose from transform_stamped_topic: %s", transform_stamped_topic_.c_str());
        pose_sub_ = nh.subscribe(transform_stamped_topic_, 100, &MapToOdomPublisher::updateFromTransformStamped, this);
      } else if (!odom_topic_.empty()) {
        ROS_INFO("Using pose from odom_topic: %s", odom_topic_.c_str());
        pose_sub_ = nh.subscribe(odom_topic_, 100, &MapToOdomPublisher::updateFromOdometry, this);
      } else {
        ROS_ERROR("No pose topic set");
        throw std::runtime_error("No pose topic set");
      }

      publish_timer_ = nh.createTimer(ros::Duration(1.0 / publish_freq_), &MapToOdomPublisher::publish, this);

      last_update_time_ = ros::Time(0);
    }

    ~MapToOdomPublisher(void)
    {
      if (m_tfServer)
        delete m_tfServer; 
      if (m_tfListener)
        delete m_tfListener;
      if (m_tfBuffer)
        delete m_tfBuffer;
    }


  private:
    ros::NodeHandle m_nh;
    tf2_ros::TransformBroadcaster       *m_tfServer;
    tf2_ros::TransformListener          *m_tfListener;
    tf2_ros::Buffer                     *m_tfBuffer;
    ros::Subscriber pose_sub_; 

    double transform_tolerance_;
    double publish_freq_;
    ros::Timer publish_timer_;

    tf2::Transform odom_to_map_inv_;
    ros::Time last_update_time_;

    //parameter for what odom to use
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string global_frame_id_;

    geometry_msgs::TransformStamped last_published_transform_;

  public:

    void updateFromOdometry(const nav_msgs::OdometryConstPtr& message) {
      tf2::Transform txi;
      tf2::convert(message->pose.pose, txi);

      update(txi, message->header);
    }

    void updateFromTransformStamped(const geometry_msgs::TransformStampedConstPtr& message){
      tf2::Transform txi;
      tf2::convert(message->transform, txi);

      update(txi, message->header);
    }

    void update(const tf2::Transform& txi, const std_msgs::Header& msg_header) {
      geometry_msgs::TransformStamped odom_to_map;
      try
      {
        geometry_msgs::TransformStamped txi_inv;
        txi_inv.header.frame_id = base_frame_id_;
        txi_inv.header.stamp = msg_header.stamp;
        tf2::convert(txi.inverse(), txi_inv.transform);

        m_tfBuffer->transform(txi_inv, odom_to_map, odom_frame_id_, ros::Duration(transform_tolerance_));
      }
      catch(tf2::TransformException &e)
      {
        if (last_update_time_ == ros::Time(0)) {
          ROS_INFO_THROTTLE(1.0, "Input message received before the %s->%s transform is available. Waiting for the transform. TransformException: %s", odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
        } else {
          ROS_ERROR("Failed to transform to %s from %s: %s\n", odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
        }
        return;
      }
      tf2::Transform odom_to_map_tf2;
      tf2::convert(odom_to_map.transform, odom_to_map_tf2);
      odom_to_map_inv_ = odom_to_map_tf2.inverse();

      last_update_time_ = msg_header.stamp;
    }

    void publish(const ros::TimerEvent& event) {
      if (last_update_time_ == ros::Time(0)) {
        ROS_INFO_THROTTLE(1.0, "Waiting for initial pose");
        return;
      }

      geometry_msgs::TransformStamped trans;
      trans.header.stamp = ros::Time::now() + ros::Duration(transform_tolerance_);
      trans.header.frame_id = global_frame_id_;
      trans.child_frame_id = odom_frame_id_;
      tf2::convert(odom_to_map_inv_, trans.transform);
      m_tfServer->sendTransform(trans);

      {
        tf2::Matrix3x3 m(odom_to_map_inv_.getRotation());
        double diff_roll, diff_pitch, diff_yaw;
        m.getRPY(diff_roll, diff_pitch, diff_yaw);
        ROS_INFO_THROTTLE(1.0, "Published map -> odom correction: x: %.2f m, y: %.2f m, yaw: %.2f rad", odom_to_map_inv_.getOrigin().x(), odom_to_map_inv_.getOrigin().y(), diff_yaw);
      }


      // Print out the difference between the last published transform and the current one
      if (last_published_transform_.header.frame_id != "")
      {
        tf2::Transform last_published_tf2;
        tf2::convert(last_published_transform_.transform, last_published_tf2);
        tf2::Transform diff = last_published_tf2.inverseTimes(odom_to_map_inv_);
        double diff_x = diff.getOrigin().x();
        double diff_y = diff.getOrigin().y();
        tf2::Matrix3x3 m(diff.getRotation());
        double diff_roll, diff_pitch, diff_yaw;
        m.getRPY(diff_roll, diff_pitch, diff_yaw);
        ROS_DEBUG("map -> odom correction diff: x: %.2f m, y: %.2f m, yaw: %.2f rad compared to %.2f s ago", diff_x, diff_y, diff_yaw, trans.header.stamp.toSec() - last_published_transform_.header.stamp.toSec());
      }

      last_published_transform_ = trans;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_to_odom_publisher");

  ROS_INFO("Starting map_to_odom_publisher node...");

  MapToOdomPublisher node;

  ros::spin();

  return 0;
}