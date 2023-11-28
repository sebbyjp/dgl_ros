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
           const std::function<std::unique_ptr<ObsT>(std::shared_ptr<SrcTs>...)> obs_from_srcs_function)
    : Node("observer", options)
    , obs_from_srcs_function_(obs_from_srcs_function)
    , last_src_msgs_recieved_(std::make_tuple(std::make_shared<SrcTs>()...))
  {
    // Initialize and seet parameters, allowing for overrides.
    std::array<std::string, sizeof...(SrcTs)> src_topics;
    for (int i = 0; i < sizeof...(SrcTs); i++)
    {
      this->declare_parameter("src_topic" + std::to_string(i), "rgbd_camera/points");
      src_topics[i] = this->get_parameter("src_topic" + std::to_string(i)).as_string();
      recieved_first_src_msgs_[i] = false;
    }
    this->declare_parameter("publish_observation", true);
    this->declare_parameter("cache_size", 10);

    auto src_sub_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    src_subs_ = dgl::util::addSubscriptions(this, src_topics, src_sub_group, last_src_msgs_recieved_,
                                            recieved_first_src_msgs_, std::index_sequence_for<SrcTs...>{});

    // For debugging.
    auto pub_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = pub_group;
    obs_pub_ = this->create_publisher<ObsT>("observation", 10, pub_options);
    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() -> void {
      if (observation_cache_.size() > 0 && this->get_parameter("publish_observation").as_bool())
      {
        ObsT obs = *observation_cache_.back();
        try
        {
          obs.header.stamp = this->now();
        }
        catch (std::exception& e)
        {
          RCLCPP_WARN(this->get_logger(), "Tried to update observation timestamp. Exception caught: %s", e.what());
        }
        obs_pub_->publish(obs);
      }
    });
  }

  /**
   * @brief Creates an observation from the last set of source messages and returns
   * the index of the observation in the buffer and a pointer to the observation.
   *
   * @return std::pair<size_t, ObsT*>
   */
  std::pair<size_t, ObsT*> observe()
  {
    while (!recieved_first_src_msgs())
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Observer is waiting for observation.");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    auto observation = std::apply(obs_from_srcs_function_, last_src_msgs_recieved_);
    std::unique_lock lock(obs_mutex_);
    if (observation_cache_.size() == this->get_parameter("cache_size").as_int())
    {
      observation_cache_.pop_front();
      num_pops_++;
    }
    observation_cache_.push_back(std::move(observation));
    return std::pair(observation_cache_.size() - 1, observation_cache_.back().get());
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
    if (id - num_pops_ < 0 || id - num_pops_ >= static_cast<int>(observation_cache_.size()))
    {
      return nullptr;
    }
    return observation_cache_.at(id).get();
  }

private:
  bool recieved_first_src_msgs()
  {
    return std::all_of(recieved_first_src_msgs_.begin(), recieved_first_src_msgs_.end(), [](bool b) { return b; });
  }

  std::tuple<std::shared_ptr<rclcpp::Subscription<SrcTs>>...> src_subs_;
  std::tuple<std::shared_ptr<SrcTs>...> last_src_msgs_recieved_;  // The last set of src messages revieved.
  std::array<bool, sizeof...(SrcTs)> recieved_first_src_msgs_;    // True if the first message has been recieved.

  std::function<std::unique_ptr<ObsT>(std::shared_ptr<SrcTs>...)> obs_from_srcs_function_;
  mutable std::shared_mutex obs_mutex_;
  // List of last `cache_size` observations made in reverse chronological order.
  std::deque<std::unique_ptr<ObsT>> observation_cache_ RCPPUTILS_TSA_GUARDED_BY(obs_mutex_);
  int num_pops_ = 0;

  typename rclcpp::Publisher<ObsT>::SharedPtr obs_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

}  // namespace dgl