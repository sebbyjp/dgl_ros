// Copyright (c) 2023 Sebastian Peralta
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/observer.hpp>
#include <dgl_ros/agent.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <dgl_ros/util/tf.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
namespace dgl_models
{

typedef dgl::Observer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> CgnObserver;
typedef dgl::Agent<sensor_msgs::msg::PointCloud2, dgl_ros_interfaces::action::SampleGraspPoses,
                   sensor_msgs::msg::PointCloud2>
    CgnAgent;
class ContactGraspnet : public CgnAgent
{
public:
  ContactGraspnet(rclcpp::NodeOptions& options);

  dgl_ros_interfaces::action::SampleGraspPoses::Feedback::SharedPtr
  actionFromObs(std::shared_ptr<CgnObserver> observer) override;

  sensor_msgs::msg::PointCloud2::UniquePtr obsFromSrcs(std::shared_ptr<sensor_msgs::msg::PointCloud2> msg) override;

private:
  // TODO(speralta): Make new Observation msg with centroid and point cloud.
  Eigen::Vector4f centroid_;
  Eigen::Affine3d tf_world_src_;
  std::unique_ptr<dgl::util::TransformLookup> tf_lookup_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
};
}  // namespace dgl_models