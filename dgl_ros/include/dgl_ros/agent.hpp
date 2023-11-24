#pragma once
#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/actor.hpp>
#include <dgl_ros/observer.hpp>

namespace dgl_ros
{

template <typename ObsT, typename ActionT, typename... SrcTs>
class Agent : public rclcpp::Node
{
public:
  Agent(rclcpp::NodeOptions& options) : Node("agent", options)
{
  observer_ =
      std::make_shared<Observer<ObsT, SrcTs...>>(options, std::bind(&Agent::obsFromSrcs, this, std::placeholders::_1));
  actor_ = std::make_shared<Actor<ActionT>>(options, std::bind(&Agent::actionFromObs, this, observer_));
}
  void run()
{
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(observer_);
  exec.add_node(actor_);
  exec.spin();
}

  virtual typename ActionT::Feedback::SharedPtr actionFromObs(std::shared_ptr<Observer<ObsT, SrcTs...>> observer) = 0;

  virtual std::unique_ptr<ObsT> obsFromSrcs(const SrcTs&... srcs) = 0;



protected:
  std::shared_ptr<Observer<ObsT, SrcTs...>> observer_;
  std::shared_ptr<Actor<ActionT>> actor_;
};
} // namespace dgl_ros
