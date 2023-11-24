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

namespace dgl_ros
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
  Observer(const rclcpp::NodeOptions& options, std::function<std::unique_ptr<ObsT>(const SrcT&)> obs_from_src)
  {
    obs_pub_ = this->create_publisher<ObsT>("observation", 10);  // For debugging.

    auto callback = [this, obs_from_src](const std::shared_ptr<SrcT> msg) {
      std::unique_lock lock(obs_mutex_, std::try_to_lock);
      if (!lock.owns_lock())
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Observer is busy. Skipping observation.");
        return;
      }

      last_observation_ = std::move(obs_from_src(*msg));

      if (this->get_parameter("publish").as_bool())
      {
        obs_pub_->publish(*last_observation_);
      }
    };
    src_sub_ = this->create_subscription<SrcT>(this->get_parameter("src_topic").as_string(), 10, callback);
  }

  ObsT* getObservation()
  {
    std::shared_lock lock(obs_mutex_);
    return last_observation_.get();
  }

  std::shared_lock<std::shared_mutex> getSharedLock() const
  {
    std::shared_lock lock(obs_mutex_);
    return lock;
  }

private:
  typename rclcpp::Publisher<ObsT>::SharedPtr obs_pub_;
  typename rclcpp::Subscription<SrcT>::SharedPtr src_sub_;

  mutable std::shared_mutex obs_mutex_;
  std::unique_ptr<ObsT> last_observation_ RCPPUTILS_TSA_GUARDED_BY(obs_mutex_);
};
}  // namespace dgl_ros