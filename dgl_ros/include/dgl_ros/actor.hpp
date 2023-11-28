// Copyright (c) 2023 Sebastian Peralta
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>

namespace dgl
{
/**
 * @brief This class is responsible for generating actions over a ROS2 action server.
 */
template <typename ActionT>
class Actor : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Actor object
   *
   * @param options
   * @param action_generator_func
   */
  Actor(const rclcpp::NodeOptions& options,
        std::function<typename ActionT::Feedback::SharedPtr()> action_generator_func)
    : Node("actor", options), action_generator_func_(action_generator_func)
  {
    this->declare_parameter("action_topic", "sample_grasp_poses");
    RCLCPP_INFO(this->get_logger(), "Grasp detection action server ready");

    namespace sp = std::placeholders;
    action_server_ = rclcpp_action::create_server<ActionT>(this,  this->get_parameter("action_topic").as_string(), 
    std::bind(&Actor::handle_goal, this, sp::_1, sp::_2),
                                                    std::bind(&Actor::handle_cancel, this, sp::_1),
                                                    std::bind(&Actor::handle_accepted, this, sp::_1));
  }

private:
  typedef std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> GoalHandleSharedPtr;

  rclcpp_action::CancelResponse handle_cancel(const GoalHandleSharedPtr goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const GoalHandleSharedPtr& goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "New goal accepted");
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread.
    auto publish_grasps_callback = [this](const GoalHandleSharedPtr& goal_handle) {
      goal_handle->publish_feedback(action_generator_func_());
    };
    std::thread{ publish_grasps_callback, goal_handle }.detach();
  }

  /**
   * @brief Called every time feedback is received for the goal
   * @param feedback - pointer to the feedback message
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const typename ActionT::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  std::function<typename ActionT::Feedback::SharedPtr()> action_generator_func_;
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
};
}  // namespace dgl