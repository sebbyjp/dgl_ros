#pragma once
#include <rclcpp/rclcpp.hpp>
#include <ros_dgl/actor.hpp>
#include <ros_dgl/observer.hpp>

namespace ros_dgl
{

template <typename ObsT, typename ActionT, typename... SrcTs>
class Agent : public rclcpp::Node
{
public:
  typedef std::function<typename ActionT::Feedback::SharedPtr(std::shared_ptr<Observer<ObsT, SrcTs...>> observer)>
      ActionGeneratorFunc;
  typedef std::function <std::unique_ptr<ObsT>(const SrcTs&... srcs)> ObsFromSrcsFunc;
  Agent(rclcpp::NodeOptions& options) : Node("agent", options)
  {
  }
  Agent(rclcpp::NodeOptions& options, ActionGeneratorFunc action_generator_func_, ObsFromSrcsFunc obs_from_srcs_func_);
  void run();

protected:
  std::shared_ptr<Observer<ObsT, SrcTs...>> observer_;
  std::shared_ptr<Actor<ActionT>> actor_;
};
}  // namespace ros_dgl
