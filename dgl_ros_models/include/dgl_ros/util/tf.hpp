// Copyright (c) 2023 Sebastian Peralta
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
namespace dgl
{
namespace util
{
class TransformLookup
{
public:
  TransformLookup(rclcpp::Node* node);
  bool get_tf_msg(const std::string& target_frame, const std::string& src_frame, int timeout,
                  geometry_msgs::msg::TransformStamped& tf_out);
  bool get_tf_isometry(const std::string& target_frame, const std::string& src_frame, int timeout,
                       Eigen::Isometry3d& tf_out);
  bool get_tf_affine(const std::string& target_frame, const std::string& src_frame, int timeout,
                     Eigen::Affine3d& tf_out);

private:
  rclcpp::Node* node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface_;
};
}  // namespace util
}  // namespace dgl