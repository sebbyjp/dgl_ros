#pragma once
#include <dgl_ros/observer.hpp>
#include <dgl_ros/agent.hpp>
#include <dgl_ros/actor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>

namespace dgl_ros {

using sensor_msgs::msg::PointCloud2;
using dgl_ros_interfaces::action::SampleGraspPoses;

template class Observer<PointCloud2, PointCloud2>::Observer;
template class Actor<SampleGraspPoses>::Actor;
template class Agent<PointCloud2, SampleGraspPoses, PointCloud2>::Agent;
}