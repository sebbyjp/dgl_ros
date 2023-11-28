// Copyright (c) 2023 Sebastian Peralta
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/actor.hpp>
#include <dgl_ros/observer.hpp>

namespace dgl
{

/**
 * @brief This class is responsible for recieving observations and generatoring
 * actions.
 *
 * @tparam ObsT
 * @tparam ActionT
 * @tparam SrcTs
 */
template <typename ObsT, typename ActionT, typename... SrcTs>
class Agent : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Agent object
   *
   * @param options
   * @param src_topics Source topics to subscribe to and generate observations from.
   */
  Agent(rclcpp::NodeOptions& options)
    : Node("agent", options)

  {
    this->declare_parameter("world_frame", "world");
    for (int i = 0; i < sizeof...(SrcTs); i++)
    {
      this->declare_parameter("src_frame" + std::to_string(i), "camera_locobot_link");
    }
    
    const auto obs_from_srcs_callback = [this](std::shared_ptr<SrcTs>... src_msgs) { return obsFromSrcs(src_msgs...); };
    observer_ = std::make_shared<Observer<ObsT, SrcTs...>>(options, obs_from_srcs_callback);
    actor_ = std::make_shared<Actor<ActionT>>(options, std::bind(&Agent::actionFromObs, this, observer_));
  }

  virtual void run()
  {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(actor_);
    exec.add_node(observer_);
    exec.spin();
  }

  /**
   * @brief Function that generates an action from an observer.
   *
   * @param observer  Observer that can be called multiple times to generate observations.
   * @return ActionT::Feedback::SharedPtr
   */
  virtual typename ActionT::Feedback::SharedPtr actionFromObs(std::shared_ptr<Observer<ObsT, SrcTs...>> observer) = 0;

  /**
   * @brief Function that generates an observation from a set of source messages.
   *
   * @param srcs Source messages to generate observation from.
   * @return std::unique_ptr<ObsT>
   */
  virtual std::unique_ptr<ObsT> obsFromSrcs(std::shared_ptr<SrcTs>... srcs) = 0;

protected:
  std::shared_ptr<Observer<ObsT, SrcTs...>> observer_;
  std::shared_ptr<Actor<ActionT>> actor_;
};
}  // namespace dgl
