#pragma once
#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/actor.hpp>
#include <dgl_ros/observer.hpp>
#include <dgl_ros/agent.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <gpd/grasp_detector.h>

namespace sm = sensor_msgs::msg;
namespace da = dgl_ros_interfaces::action;
namespace dgl_models
{
  // template class Observer<sm::PointCloud2, sm::PointCloud2>::Observer;
  // template class Actor<da::SampleGraspPoses>::Actor;
  // template class Agent<sm::PointCloud2, da::SampleGraspPoses, sm::PointCloud2>::Agent;

// typedef Observer<sm::PointCloud2, sm::PointCloud2> GpdObserver;
// typedef Agent<sm::PointCloud2, da::SampleGraspPoses, sm::PointCloud2> GpdAgent;
// typedef Actor<da::SampleGraspPoses> GpdActor;




class Gpd : public dgl::Agent<sm::PointCloud2, da::SampleGraspPoses, sm::PointCloud2>
{
public:
  Gpd(rclcpp::NodeOptions& options);

  da::SampleGraspPoses::Feedback::SharedPtr actionFromObs(std::shared_ptr<dgl::Observer<sm::PointCloud2, sm::PointCloud2>> observer) override;

  sm::PointCloud2::UniquePtr obsFromSrcs(std::shared_ptr<sm::PointCloud2> msg) override;

private:
  std::unique_ptr<gpd::GraspDetector> gpd_grasp_detector_;
  Eigen::Isometry3d transform_base_opt_;
};
}  // namespace dgl_models