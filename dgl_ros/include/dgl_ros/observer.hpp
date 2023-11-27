// Copyright (c) 2023 Sebastian Peralta
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <queue>
#include <memory>
#include <shared_mutex>
#include <dgl_ros/util/generic_subscription.hpp>
namespace dgl
{
/**
 * @brief Observer subscribes to src_topics and stores
 *  a buffer of observations.
 * 
 * @tparam ObsT 
 * @tparam SrcTs 
 */
template <typename ObsT, typename... SrcTs>
class Observer : public rclcpp::Node
{
public:
 /**
  * @brief  Observer subscribes to src_topics and stores
   *  a buffer of observations.
  * 
  * @param options 
  * @param obs_from_srcs Function that creates an Observation from Source messages.
  * @param src_topics List of topics to subscribe to.
  * @param buffer_size 
  */
  Observer(const rclcpp::NodeOptions& options,
           const std::function<std::unique_ptr<ObsT>(std::shared_ptr<SrcTs>...)> obs_from_srcs,
           const std::array<std::string, sizeof...(SrcTs)>& src_topics, size_t buffer_size = 10)
    : Node("observer", options), buffer_size_(buffer_size)
  {
    int i = 0;
    for (const auto& src_topic : src_topics)
    {
      this->declare_parameter("src_topic" + std::to_string(i), src_topic);
      i++;
    }
    for (auto& b : recieved_first_srcs_)
    {
      b = false;
    }
    this->declare_parameter("publish", false);

    obs_from_srcs_ = obs_from_srcs;
    src_msgs_ = std::make_tuple(std::make_shared<SrcTs>()...);
  
    auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    src_subs_ = dgl::util::addSubscriptions(this, src_topics, callback_group, src_msgs_, recieved_first_srcs_,
                                                std::index_sequence_for<SrcTs...>{});
    // For debugging.
    auto pub_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions options_pub;
    options_pub.callback_group = pub_callback_group;
    obs_pub_ = this->create_publisher<ObsT>("observation", 10, options_pub); 
  }

  /**
   * @brief Creates an observation from the last set of source messages and returns
   * the index of the observation in the buffer and a pointer to the observation.
   * 
   * @return std::pair<size_t, ObsT*> 
   */
  std::pair<size_t, ObsT*> observe()
  {
    while (!recieved_first_srcs())
    {
      RCLCPP_DEBUG_ONCE(this->get_logger(), "Observer is waiting for observation.");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::unique_lock lock(obs_mutex_);
    if (observations_.size() == buffer_size_)
    {
      observations_.pop_front();
      num_pops_++;
    }
    observations_.push_back(std::move(std::apply(obs_from_srcs_, src_msgs_)));

    if (this->get_parameter("publish").as_bool())
    {
      obs_pub_->publish(*observations_.back());
    }
    return std::pair(observations_.size() - 1, observations_.back().get());
  }

  /**
   * @brief Returns a pointer to the observation at index id or nullptr if id is out of range.
   * 
   * @param id 
   * @return ObsT* 
   */
  ObsT* get(int id)
  {
    // Subtract by num_pops_ to account for id's changing.
    if (id - num_pops_ < 0 || id - num_pops_ >= static_cast<int>(observations_.size()))
    {
      return nullptr;
    }
    return observations_.at(id).get();
  }

private:
  bool recieved_first_srcs()
  {
    return std::all_of(recieved_first_srcs_.begin(), recieved_first_srcs_.end(), [](bool b) { return b; });
  }
  std::array<bool, sizeof...(SrcTs)> recieved_first_srcs_;
  typename rclcpp::Publisher<ObsT>::SharedPtr obs_pub_;
  std::tuple<std::shared_ptr<rclcpp::Subscription<SrcTs>>...> src_subs_;
  std::tuple<std::shared_ptr<SrcTs>...> src_msgs_; // The last set of src messages revieved.
  mutable std::shared_mutex obs_mutex_;
  std::deque<std::unique_ptr<ObsT>> observations_ RCPPUTILS_TSA_GUARDED_BY(obs_mutex_);
  int num_pops_ = 0;
  size_t buffer_size_;
  std::function<std::unique_ptr<ObsT>(std::shared_ptr<SrcTs>...)> obs_from_srcs_;
};

}  // namespace dgl