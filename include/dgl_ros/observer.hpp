/**
 * @file observer.hpp
 * @brief
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <queue>

namespace dgl_ros {
template <class SrcT, class ObsT>
class Observer : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Observer Node. SharedPtr types are recommended for the
   *        template parameters.
   *
   * @param obs_topic
   * @param src_topics
   * @param callback Function that creates Observation from Source Data
   */
  Observer(const std::string& obs_topic, const std::string& src_topic, std::function<ObsT(SrcT)> callback);

private:
  typename rclcpp::Publisher<ObsT>::SharedPtr obs_pub_;
  typename rclcpp::Subscription<SrcT>::SharedPtr src_sub_;
  std::queue<ObsT> obs_buffer_;
};
}  // namespace dgl_ros