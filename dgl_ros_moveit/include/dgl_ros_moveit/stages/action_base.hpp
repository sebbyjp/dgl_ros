// Copyright (c) 2023 Sebastian Peralta
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT



#pragma once

#include <memory>
#include <string>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace moveit
{
namespace task_constructor
{
namespace stages
{

/**
 * @brief Interface allowing stages to use a simple action client
 * @param ActionT - action message (action message name + "ACTION")
 * @details Some stages may require an action client. This class wraps the
 *          simple client action interface and exposes event based execution callbacks.
 */
template <typename ActionT>
class ActionBase
{
protected:
  typedef typename ActionT::Feedback Feedback;
  typedef typename ActionT::Result Result;
  typedef typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr GoalHandleSharedPtr;
  typedef typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult WrappedResult;

public:
  /**
   * @brief Constructor
   * @param action_name - action namespace
   * @param spin_thread - spins a thread to service this action's subscriptions
   * @param goal_timeout - goal to completed time out (0 is considered infinite timeout)
   * @param server_timeout - connection to server time out (0 is considered infinite timeout)
   * @details Initialize the action client and time out parameters
   */
  ActionBase(const std::string& action_name, bool spin_thread, int goal_timeout = 0, int server_timeout = 0);

  /**
   * @brief Constructor
   * @param action_name - action namespace
   * @param spin_thread - spins a thread to service this action's subscriptions
   * @details Initialize the action client and time out parameters to infinity
   */
  ActionBase(const std::string& action_name, bool spin_thread);

  /* @brief Destructor */
  virtual ~ActionBase() = default;

  /* @brief Called when goal becomes active */
  virtual void goal_response_callback(const GoalHandleSharedPtr& goal_handle) = 0;

  /**
   * @brief Called every time feedback is received for the goal
   * @param feedback - pointer to the feedback message
   */
  virtual void feedback_callback(const GoalHandleSharedPtr goal_handle,
                                 const std::shared_ptr<const Feedback> feedback) = 0;

  /**
   * @brief Called once when the goal completes
   * @param state - state info for goal
   * @param result - pointer to result message
   */

  virtual void result_callback(const WrappedResult& result) = 0;

protected:
  rclcpp::Node::SharedPtr nh_;                                     // node handle
  std::string action_name_;                                        // action name space
  typename rclcpp_action::Client<ActionT>::SharedPtr client_ptr_;  // action client
  int server_timeout_, goal_timeout_;                              // connection and goal completed time out
};

template <typename ActionT>
ActionBase<ActionT>::ActionBase(const std::string& action_name, bool spin_thread, int goal_timeout,
                                int server_timeout)
  : action_name_(action_name), server_timeout_(server_timeout), goal_timeout_(goal_timeout)
{
  (void)spin_thread;
  nh_ = rclcpp::Node::make_shared("action_base");
  client_ptr_ = rclcpp_action::create_client<ActionT>(nh_, action_name);

  // Negative timeouts are set to zero
  server_timeout_ = server_timeout_ < std::numeric_limits<double>::epsilon() ? 0 : server_timeout_;
  goal_timeout_ = goal_timeout_ < std::numeric_limits<double>::epsilon() ? 0 : goal_timeout_;
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
