#pragma once
#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/actor.hpp>
#include <dgl_ros/observer.hpp>

namespace dgl
{

template <typename ObsT, typename ActionT, typename... SrcTs>
class Agent : public rclcpp::Node
{
public:
  Agent(rclcpp::NodeOptions& options, const std::array<std::string, sizeof...(SrcTs)>& src_topics)
    : Node("agent", options)

  {
    const auto obs_from_srcs_callback = [this](std::shared_ptr<SrcTs>... src_msgs) { return obsFromSrcs(src_msgs...); };
    observer_ = std::make_shared<Observer<ObsT, SrcTs...>>(options, obs_from_srcs_callback, src_topics);
    actor_ = std::make_shared<Actor<ActionT>>(options, std::bind(&Agent::actionFromObs, this, observer_));
  }

 void run()
  {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(actor_);
    exec.add_node(observer_);
    exec.spin();
  }

  virtual typename ActionT::Feedback::SharedPtr actionFromObs(std::shared_ptr<Observer<ObsT, SrcTs...>> observer) = 0;

  virtual std::unique_ptr<ObsT> obsFromSrcs(std::shared_ptr<SrcTs>... srcs) = 0;

protected:
  std::shared_ptr<Observer<ObsT, SrcTs...>> observer_;
  std::shared_ptr<Actor<ActionT>> actor_;

};
}  // namespace dgl
