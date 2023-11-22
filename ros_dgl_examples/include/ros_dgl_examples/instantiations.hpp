#pragma once
#include <ros_dgl/observer.hpp>
#include <ros_dgl/agent.hpp>
#include <ros_dgl/actor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ros_dgl_interfaces/action/sample_grasp_poses.hpp>

namespace ros_dgl {

using sensor_msgs::msg::PointCloud2;
using ros_dgl_interfaces::action::SampleGraspPoses;

template class Observer<PointCloud2, PointCloud2>::Observer;
template class Actor<SampleGraspPoses>::Actor;
template class Agent<PointCloud2, SampleGraspPoses, PointCloud2>::Agent;
}