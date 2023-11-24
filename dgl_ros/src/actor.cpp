
// ROS
#include <dgl_ros/actor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
namespace dgl_ros {
using std::placeholders::_1;
using std::placeholders::_2;
template <typename ActionT>
Actor<ActionT>::Actor(const rclcpp::NodeOptions& options, std::function<typename ActionT::Feedback::SharedPtr()> action_generator_func)
  : Actor(options)
{
  action_generator_ = action_generator_func;
  RCLCPP_INFO(this->get_logger(), "Grasp detection action server ready");
  auto action_topic = this->get_parameter("action_topic").as_string();
  server_ = rclcpp_action::create_server<ActionT>(
      this, action_topic, std::bind(&Actor::handle_goal, this, _1, _2),
      std::bind(&Actor::handle_cancel, this, _1), std::bind(&Actor::handle_accepted, this, _1));
}
template <typename ActionT>
rclcpp_action::CancelResponse Actor<ActionT>::handle_cancel(const GoalHandleSharedPtr goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionT>
rclcpp_action::GoalResponse Actor<ActionT>::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                        std::shared_ptr<const typename ActionT::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
template <typename ActionT>
void Actor<ActionT>::handle_accepted(const GoalHandleSharedPtr& goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "New goal accepted");

  // This needs to return quickly to avoid blocking the executor, so spin up a new thread.
  auto publish_grasps = [this](const GoalHandleSharedPtr& goal_handle) {
    goal_handle->publish_feedback(action_generator_());
  };
  std::thread{ publish_grasps, goal_handle }.detach();
}

template class Actor< dgl_ros_interfaces::action::SampleGraspPoses>::Actor;
} // namespace dgl_ros