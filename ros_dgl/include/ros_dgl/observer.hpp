/**
 * @file observer.hpp
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <queue>
#include <memory>

namespace ros_dgl
{
template <typename ObsT, typename... SrcTs>

/**
 * @brief ROS params: src_topic, publish=false
 *
 */
class Observer : public rclcpp::Node
{
public:
  // Set up parent Node.
  Observer(const rclcpp::NodeOptions& options, std::initializer_list<std::string> src_topics)
    : Node("observer", options)
  {
    int i = 0;
    for(const auto& src_topic : src_topics) {
    this->declare_parameter("src_topics" + std::to_string( i), src_topic);
    i++;
    }
    this->declare_parameter("publish", true);
  }
  /**
   * @brief  Observer subscribes to src_topic and stores
   *  an observation.
   *
   * @param options NodeOptions for the Node.
   * @param obs_from_src Function that creates Observation from Source Data
   */
  Observer(const rclcpp::NodeOptions& options, std::function<std::unique_ptr<ObsT>(const SrcTs&...)> obs_from_srcs,
          std::initializer_list<std::string> src_topics);

  ObsT* getObservation()
  {
    observation_ = std::move(std::apply(obs_from_srcs_, src_msgs_));
    std::shared_lock lock(obs_mutex_);
    return observation_.get();
  }

  std::shared_lock<std::shared_mutex> getSharedLock() const
  {
    std::shared_lock lock(obs_mutex_);
    return lock;
  }

private:
  typename rclcpp::Publisher<ObsT>::SharedPtr obs_pub_;
  std::tuple<typename rclcpp::Subscription<SrcTs>::SharedPtr...> src_subs_;
  std::tuple<std::unique_ptr<SrcTs>...> src_msgs_;
  mutable std::shared_mutex obs_mutex_;
  std::unique_ptr<ObsT> observation_;

  std::function<std::unique_ptr<ObsT>(const SrcTs&...)> obs_from_srcs_;
};
}  // namespace ros_dgl