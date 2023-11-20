
// ROS
#include <ros_dgl/grasp_generator.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
namespace ros_dgl {
using std::placeholders::_1;
using std::placeholders::_2;
template <typename ActionT>
ActionProducer<ActionT>::ActionProducer(std::function<typename ActionT::Feedback::SharedPtr()> action_generator_func)
  : rclcpp::Node("grasp_generator_server")
{
  action_generator_ = action_generator_func;
  RCLCPP_INFO(this->get_logger(), "Grasp detection action server ready");

  server_ = rclcpp_action::create_server<ActionT>(
      this, "sample_grasp_poses", std::bind(&ActionProducer::handle_goal, this, _1, _2),
      std::bind(&ActionProducer::handle_cancel, this, _1), std::bind(&ActionProducer::handle_accepted, this, _1));
}
template <typename ActionT>
rclcpp_action::CancelResponse ActionProducer<ActionT>::handle_cancel(const GoalHandleSharedPtr goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionT>
rclcpp_action::GoalResponse ActionProducer<ActionT>::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                        std::shared_ptr<const typename ActionT::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
template <typename ActionT>
void ActionProducer<ActionT>::handle_accepted(const GoalHandleSharedPtr& goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "New goal accepted");

  // This needs to return quickly to avoid blocking the executor, so spin up a new thread.
  auto publish_grasps = [this](const GoalHandleSharedPtr& goal_handle) {
    goal_handle->publish_feedback(action_generator_());
  };
  std::thread{ publish_grasps, goal_handle }.detach();
}
template class ActionProducer<ros_dgl_interfaces::action::SampleGraspPoses>;
} // namespace ros_dgl