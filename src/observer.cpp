/**
 * @file observer.cpp
 * @brief
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <queue>
#include "dgl_ros/observer.hpp"

namespace dgl_ros
{
// TODO(speralta): Make variadic with arbitrary number of source types.
// TODO(speralta): ObsT must be a ros message with a UUID field. Use C++20 concepts to enforce this?
template <class SrcT, class ObsT>

 // TODO(speralta):  Set up efficient intra-process communication.
  // - See if it's better to pass around a shared ptr, unique_ptr, or UUID for
  // clients and subscribers to access observation.
  // - See if ros2 type adapters should be used.
Observer<SrcT, ObsT>::Observer(const std::string& obs_topic, const std::string& src_topic,
                               std::function<ObsT(SrcT)> callback):  rclcpp::Node("observer")
{
    obs_pub_ = this->create_publisher<ObsT>(obs_topic, 10);
    src_sub_ = this->create_subscription<SrcT>(
        src_topic, 10, [this, callback](const std::shared_ptr<SrcT> msg) { obs_buffer_.push(callback(*msg)); });
}
}  // namespace dgl_ros