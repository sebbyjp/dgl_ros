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
#include <gpd/grasp_detector.h>

namespace dgl_models
{

typedef dgl::Observer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> GpdObserver;
typedef dgl::Agent<sensor_msgs::msg::PointCloud2, dgl_ros_interfaces::action::SampleGraspPoses,
                   sensor_msgs::msg::PointCloud2>
    GpdAgent;
class Gpd : public GpdAgent
{
public:
  Gpd(rclcpp::NodeOptions& options);

  dgl_ros_interfaces::action::SampleGraspPoses::Feedback::SharedPtr
  actionFromObs(std::shared_ptr<GpdObserver> observer) override;

  sensor_msgs::msg::PointCloud2::UniquePtr obsFromSrcs(std::shared_ptr<sensor_msgs::msg::PointCloud2> msg) override;

private:
  std::unique_ptr<gpd::GraspDetector> gpd_grasp_detector_;
  Eigen::Isometry3d transform_base_opt_;
};
}  // namespace dgl_models