#include <rclcpp/rclcpp.hpp>
#include <ros_dgl/actor.hpp>
#include <ros_dgl/observer.hpp>
#include <ros_dgl/agent.hpp>
#include <ros_dgl_interfaces/action/sample_grasp_poses.hpp>

namespace ros_dgl
{

template <typename ObsT, typename ActionT, typename... SrcTs>
Agent<ObsT, ActionT, SrcTs...>::Agent(rclcpp::NodeOptions& options, ActionGeneratorFunc action_generator_func_,
                                      ObsFromSrcsFunc obs_from_srcs_func_)
  : Agent(options)
{
  observer_ = std::make_shared<Observer<ObsT, SrcTs...>>(options, obs_from_srcs_func_);
  actor_ = std::make_shared<Actor<ActionT>>(
      options, [this, action_generator_func_] { return action_generator_func_(observer_); });
}

template <typename ObsT, typename ActionT, typename... SrcTs>
void Agent<ObsT, ActionT, SrcTs...>::run()
{
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(actor_);
  exec.add_node(observer_);
  exec.spin();
}
// template class Agent<sensor_msgs::msg::PointCloud2, ros_dgl_interfaces::action::SampleGraspPoses,
// sensor_msgs::msg::PointCloud2>::Agent;
}  // namespace ros_dgl