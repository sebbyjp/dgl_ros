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
  Agent(rclcpp::NodeOptions& options);
  void run();

  virtual typename ActionT::Feedback::SharedPtr actionFromObs(std::shared_ptr<Observer<ObsT, SrcTs...>> observer) = 0;

  virtual std::unique_ptr<ObsT> obsFromSrcs(const SrcTs&... srcs) = 0;



protected:
  std::shared_ptr<Observer<ObsT, SrcTs...>> observer_;
  std::shared_ptr<Actor<ActionT>> actor_;
};
} // namespace ros_dgl
