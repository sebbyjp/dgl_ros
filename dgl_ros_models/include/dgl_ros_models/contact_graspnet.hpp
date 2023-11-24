#pragma once
#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/actor.hpp>
#include <dgl_ros/observer.hpp>
#include <dgl_ros/agent.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>

namespace dgl_ros
{
typedef dgl_ros_interfaces::action::SampleGraspPoses SampleGraspPoses;
typedef sensor_msgs::msg::PointCloud2 PointCloud2;

typedef dgl_ros::Observer<PointCloud2, PointCloud2> CgnObserver;
typedef dgl_ros::Agent<PointCloud2, SampleGraspPoses, PointCloud2> CgnAgent;

class ContactGraspnet : public CgnAgent
{
public:
  ContactGraspnet(rclcpp::NodeOptions& options);

  SampleGraspPoses::Feedback::SharedPtr
  actionFromObs(std::shared_ptr<CgnObserver> observer) override;

  std::unique_ptr<PointCloud2> obsFromSrcs(const PointCloud2& msg) override;

private:
  Eigen::Isometry3d transform_base_opt_;
};
// template class Observer<PointCloud2, PointCloud2>::Observer;
// template class Actor<SampleGraspPoses>::Actor;
// template class Agent<PointCloud2, SampleGraspPoses, PointCloud2>::Agent;

}  // namespace dgl_ros