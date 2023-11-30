// Copyright (c) 2023 Sebastian Peralta
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>
#include <dgl_ros/util/tf.hpp>
namespace dgl
{
namespace util
{
TransformLookup::TransformLookup(rclcpp::Node* node) : node_(node)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  timer_interface_ =
      std::make_shared<tf2_ros::CreateTimerROS>(node_->get_node_base_interface(), node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface_);
}

bool TransformLookup::get_tf_msg(const std::string& target_frame, const std::string& src_frame, int timeout,
                             geometry_msgs::msg::TransformStamped& tf_out)
{
  // Look up for the transformation between src_frame and turtle2 frames
  // and send velocity commands for turtle2 to reach src_frame
  try
  {
    tf_out = tf_buffer_->lookupTransform(target_frame, src_frame, rclcpp::Time(0), rclcpp::Duration(timeout,0));
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_INFO(node_->get_logger(), "Could not transform %s to %s: %s", src_frame.c_str(), target_frame.c_str(),
                ex.what());
    return false;
  }
}  

bool TransformLookup::get_tf_isometry(const std::string& target_frame, const std::string& src_frame,
                                  int timeout,    Eigen::Isometry3d& tf_out)

{
  geometry_msgs::msg::TransformStamped tf_stamped;
  if (get_tf_msg(target_frame, src_frame,timeout,  tf_stamped))
  {
    tf_out = tf2::transformToEigen(tf_stamped);
    return true;
  }
  return false;
}
bool TransformLookup::get_tf_affine(const std::string& target_frame, const std::string& src_frame,
                               int timeout,     Eigen::Affine3d& tf_out)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  if (get_tf_msg(target_frame, src_frame, timeout, tf_stamped))
  {
    geometry_msgs::msg::Pose tf_pose = Eigen::toMsg(tf2::transformToEigen(tf_stamped));
    Eigen::fromMsg(tf_pose, tf_out);
    return true;
  }
  return false;
}
}  // namespace util
}  // namespace dgl