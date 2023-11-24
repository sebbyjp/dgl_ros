#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/actor.hpp>
#include <dgl_ros/observer.hpp>
#include <dgl_ros/agent.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>

namespace dgl_ros
{

template <typename ObsT, typename ActionT, typename... SrcTs>
Agent<ObsT, ActionT, SrcTs...>::Agent(rclcpp::NodeOptions& options) : Node("agent", options)
{
  observer_ =
      std::make_shared<Observer<ObsT, SrcTs...>>(options, std::bind(&Agent::obsFromSrcs, this, std::placeholders::_1));
  actor_ = std::make_shared<Actor<ActionT>>(options, std::bind(&Agent::actionFromObs, this, observer_));
}

template <typename ObsT, typename ActionT, typename... SrcTs>
void Agent<ObsT, ActionT, SrcTs...>::run()
{
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(actor_);
  exec.add_node(observer_);
  exec.spin();
}
// template class Agent<sensor_msgs::msg::PointCloud2, dgl_ros_interfaces::action::SampleGraspPoses, sensor_msgs::msg::PointCloud2>::Agent;
}  // namespace dgl_ros