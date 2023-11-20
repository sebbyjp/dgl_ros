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
template <typename ObsT, typename SrcT>

/**
 * @brief ROS params: src_topic, publish=false
 *
 */
class Observer : public rclcpp::Node
{
public:
  // Set up parent Node.
  Observer(const rclcpp::NodeOptions& options) : Node("observer", options)
  {
      this->declare_parameter("src_topic", "");
      this->declare_parameter("publish", true);
  }
  /**
   * @brief  Observer subscribes to src_topic and stores
   *  an observation.
   *
   * @param options NodeOptions for the Node.
   * @param obs_from_src Function that creates Observation from Source Data
   */
  Observer(const rclcpp::NodeOptions& options, std::function<std::unique_ptr<ObsT>(const SrcT&)> obs_from_src);

  ObsT* getObservation()
  {
    std::shared_lock lock(obs_mutex_);
    return last_observation_.get();
  }

  std::shared_lock<std::mutex> getSharedLock() const
  {
    std::shared_lock lock(obs_mutex_);
    return lock;
  }

private:
  typename rclcpp::Publisher<ObsT>::SharedPtr obs_pub_;
  typename rclcpp::Subscription<SrcT>::SharedPtr src_sub_;

  mutable std::mutex obs_mutex_;
  std::unique_ptr<ObsT> last_observation_ RCPPUTILS_TSA_GUARDED_BY(obs_mutex_);
};
}  // namespace ros_dgl