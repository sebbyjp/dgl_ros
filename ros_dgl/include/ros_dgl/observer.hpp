/**
 * @file observer.hpp
 * @brief Subscribes to 1 or 2 topics and creates buffer of observations.
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
template <typename SrvT, typename ObsT, typename Src1T, typename Src2T = nullptr_t>
class Observer : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Observer Node.
   *
   * @param src1_topic Topic of the first source data
   * @param src2_topic Topic of the second source data
   * @param obs_from_src Function that creates Observation from Source Data
   */
  Observer(const rclcpp::NodeOptions& options, std::function<ObsT(Src1T)> obs_from_src);
  Observer(const rclcpp::NodeOptions& options,
           std::function<ObsT(Src1T, std::vector<std::shared_ptr<Src2T>>)> obs_from_src, int src2_size = 1);

private:
  void init() {
    this->declare_parameter("src1_topic", "");
    this->declare_parameter("src2_topic", "");
    this->declare_parameter("buffer_size", 10);
    this->declare_parameter("publish", true);
    obs_buffer_ = std::queue<std::shared_ptr<ObsT>>();
    obs_pub_ = this->create_publisher<ObsT>("observation", 10);  // For debugging.
    buffer_size_ = this->get_parameter("buffer_size").as_int();

    src1_sub_ = this->create_subscription<Src1T>(this->get_parameter("src1_topic").as_string(), 10,
                                                 [this](const std::shared_ptr<Src1T> src_msg) {
                                                   last_src1_ = std::move(src_msg);
                                                   new_src1_recieved_ = true;
                                                 });
  }
  void respond(std::shared_ptr<ObsT> observation, bool publish, const std::shared_ptr<typename SrvT::Response> response)
  {
    if (obs_buffer_.size() + 1 == buffer_size_)
    {
      obs_buffer_.pop();
    }

    obs_buffer_.push(observation);
    response->observation = *obs_buffer_.back();
    if (publish)
    {
      obs_pub_->publish(*obs_buffer_.back());
    }
  }
  typename rclcpp::Publisher<ObsT>::SharedPtr obs_pub_;

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr src_listener_node_;
  typename rclcpp::Subscription<Src1T>::SharedPtr src1_sub_;
  typename rclcpp::Subscription<Src2T>::SharedPtr src2_sub_;

  typename rclcpp::Service<SrvT>::SharedPtr obs_srv_;
  std::queue<std::shared_ptr<ObsT>> obs_buffer_;

  std::shared_ptr<Src1T> last_src1_;
  std::shared_ptr<Src2T> last_src2_;

  bool new_src1_recieved_ = false;
  bool new_src2_recieved_ = false;

  size_t buffer_size_;
};
}  // namespace ros_dgl