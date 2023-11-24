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

namespace dgl_ros {
/**
 * @brief Generates grasp poses.
 */
template <typename ActionT>
class Actor : public rclcpp::Node
{
public:
Actor(const rclcpp::NodeOptions& options) : Node("actor", options) {
  this->declare_parameter("action_topic", "action_producer_topic");
}

  /**
  * @brief Constructor
  * @details loads parameters, registers callbacks for the action server,
             and initializes GPD
  */
  Actor(const rclcpp::NodeOptions& options, std::function<typename ActionT::Feedback::SharedPtr()> action_generator_func);


private:
  typedef std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>  GoalHandleSharedPtr ;

  rclcpp_action::CancelResponse handle_cancel(const GoalHandleSharedPtr goal_handle);
  void handle_accepted(const GoalHandleSharedPtr& goal_handle);

  /**
   * @brief Called every time feedback is received for the goal
   * @param feedback - pointer to the feedback message
   */

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const typename ActionT::Goal> goal);
  std::function<typename ActionT::Feedback::SharedPtr()> action_generator_;

  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
};
}  // namespace dgl_ros