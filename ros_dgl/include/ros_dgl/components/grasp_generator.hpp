/**
 * @file grasp_generator.hpp
 * @brief 
 * 
 */
#pragma once
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// C++
#include <memory>

// Action Server
#include <deep_grasp_msgs/action/sample_grasp_poses.hpp>

/**
 * @brief Generates grasp poses.
 */
class GraspGenerator : public rclcpp::Node
{
public:
  using SampleGraspPoses =  typename deep_grasp_msgs::action::SampleGraspPoses;
  using GoalHandleSharedPtr = typename std::shared_ptr<rclcpp_action::ServerGoalHandle<SampleGraspPoses>> ;
  /**
  * @brief Constructor
  * @details loads parameters, registers callbacks for the action server,
             and initializes GPD
  */
  GraspGenerator(std::function<SampleGraspPoses::Feedback::SharedPtr()> grasp_generator, bool& goal_active);


private:
  rclcpp_action::CancelResponse handle_cancel(const GoalHandleSharedPtr goal_handle);
  void handle_accepted(const GoalHandleSharedPtr& goal_handle);

  /**
   * @brief Called every time feedback is received for the goal
   * @param feedback - pointer to the feedback message
   */

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const SampleGraspPoses::Goal> goal);
  void execute(const GoalHandleSharedPtr& goal_handle);
  // std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  // std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface_;

  // std::string path_to_model_config_;  // path to GPD config file
  // std::string input_sensor_topic_;    // point cloud topic name

  bool& goal_active_;
  std::function<SampleGraspPoses::Feedback::SharedPtr(void)> grasp_generator_;

  rclcpp_action::Server<SampleGraspPoses>::SharedPtr server_;
  SampleGraspPoses::Result result_;
};
