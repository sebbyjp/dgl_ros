#pragma once
#include <rclcpp/rclcpp.hpp>
#include <ros_dgl/actor.hpp>
#include <ros_dgl/observer.hpp>
#include <ros_dgl/agent.hpp>
#include <ros_dgl_interfaces/action/sample_grasp_poses.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>

typedef ros_dgl_interfaces::action::SampleGraspPoses SampleGraspPoses;
typedef sensor_msgs::msg::PointCloud2 PointCloud2;
namespace ros_dgl
{

class GPDGraspDetectionServer : public Agent<PointCloud2, SampleGraspPoses, PointCloud2>
{
public:
  GPDGraspDetectionServer(rclcpp::NodeOptions& options);

  SampleGraspPoses::Feedback::SharedPtr
  actionFromObs(std::shared_ptr<Observer<PointCloud2, PointCloud2>> observer) override;

  std::unique_ptr<PointCloud2> obsFromSrcs(const PointCloud2& msg) override;

private:
  std::unique_ptr<gpd::GraspDetector> gpd_grasp_detector_;
  Eigen::Isometry3d transform_base_opt_;
};
// template class Observer<PointCloud2, PointCloud2>::Observer;
// template class Actor<SampleGraspPoses>::Actor;
// template class Agent<PointCloud2, SampleGraspPoses, PointCloud2>::Agent;

}  // namespace ros_dgl