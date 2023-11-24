/**
 * @file grasp_generator.hpp
 * @brief
 *
 */
#pragma once
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>

namespace dgl_ros
{
/**
 * @brief Generates grasp poses.
 */
template <typename ActionT>
class Actor : public rclcpp::Node
{
public:
  Actor(const rclcpp::NodeOptions& options) : Node("actor", options)
  {
    this->declare_parameter("action_topic", "action_producer_topic");
  }

  /**
  * @brief Constructor
  * @details loads parameters, registers callbacks for the action server,
             and initializes GPD
  */
  Actor(const rclcpp::NodeOptions& options,
        std::function<typename ActionT::Feedback::SharedPtr()> action_generator_func)
    : Actor(options)
  {
    (void)goal_handle;
    action_generator_ = action_generator_func;
    RCLCPP_INFO(this->get_logger(), "Grasp detection action server ready");
    auto action_topic = this->get_parameter("action_topic").as_string();
    server_ = rclcpp_action::create_server<ActionT>(this, action_topic, std::bind(&Actor::handle_goal, this, _1, _2),
                                                    std::bind(&Actor::handle_cancel, this, _1),
                                                    std::bind(&Actor::handle_accepted, this, _1));
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
    (void)uuid;
    (void)goal;
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Called every time feedback is received for the goal
   * @param feedback - pointer to the feedback message
   */

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const typename ActionT::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "New goal accepted");

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread.
    auto publish_grasps = [this](const GoalHandleSharedPtr& goal_handle) {
      goal_handle->publish_feedback(action_generator_());
    };
    std::thread{ publish_grasps, goal_handle }.detach();
  }
  std::function<typename ActionT::Feedback::SharedPtr()> action_generator_;

  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
};
}  // namespace dgl_ros