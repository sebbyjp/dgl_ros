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
class GraspGenerator : public rclcpp::Node
{
public:
  using SampleGraspPoses =  typename ros_dgl_interfaces::action::SampleGraspPoses;
  using GoalHandleSharedPtr = typename std::shared_ptr<rclcpp_action::ServerGoalHandle<SampleGraspPoses>> ;
  /**
  * @brief Constructor
  * @details loads parameters, registers callbacks for the action server,
             and initializes GPD
  */
  GraspGenerator(std::function<SampleGraspPoses::Feedback::SharedPtr()> generate_grasps_func);


private:
  rclcpp_action::CancelResponse handle_cancel(const GoalHandleSharedPtr goal_handle);
  void handle_accepted(const GoalHandleSharedPtr& goal_handle);

  /**
   * @brief Called every time feedback is received for the goal
   * @param feedback - pointer to the feedback message
   */

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const SampleGraspPoses::Goal> goal);
  std::function<SampleGraspPoses::Feedback::SharedPtr()> generate_grasps_func_;

  rclcpp_action::Server<SampleGraspPoses>::SharedPtr server_;
};
}  // namespace ros_dgl