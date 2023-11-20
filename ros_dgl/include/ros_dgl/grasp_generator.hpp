/**
 * @file grasp_generator.hpp
 * @brief 
 * 
 */
#pragma once
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros_dgl_interfaces/action/sample_grasp_poses.hpp"
#include <memory>

namespace ros_dgl {
/**
 * @brief Generates grasp poses.
 */
template <typename ActionT>
class ActionProducer : public rclcpp::Node
{
public:
  using GoalHandleSharedPtr = typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> ;
  /**
  * @brief Constructor
  * @details loads parameters, registers callbacks for the action server,
             and initializes GPD
  */
  ActionProducer(std::function<typename ActionT::Feedback::SharedPtr()> action_generator_func);


private:
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
}  // namespace ros_dgl