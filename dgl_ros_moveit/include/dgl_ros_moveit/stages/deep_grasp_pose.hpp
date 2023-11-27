// Copyright (c) 2023 Sebastian Peralta
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>
#include <dgl_ros_moveit/stages/action_base.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit
{
namespace task_constructor
{
namespace stages
{

/**
 * @brief Generate grasp candidates using deep learning approaches
 * @param ActionSpec - action message (action message name + "ACTION")
 * @details Interfaces with a deep learning based grasp library using a action client
 */
template <typename ActionT>
class DeepGraspPose : public GeneratePose, ActionBase<ActionT>
{
private:
  typedef ActionBase<ActionT> ActionBaseT;
  typedef typename ActionBaseT::Feedback Feedback;
  typedef typename ActionBaseT::Result Result;
  typedef typename ActionBaseT::GoalHandleSharedPtr GoalHandleSharedPtr;
  typedef typename ActionBaseT::WrappedResult WrappedResult;

public:
  /**
   * @brief Constructor
   * @param action_name - action namespace
   * @param stage_name - name of stage
   * @param goal_timeout - goal to completed time out (0 is considered infinite timeout)
   * @param server_timeout - connection to server time out (0 is considered infinite timeout)
   * @details Initialize the client and connect to server
   */
  DeepGraspPose(const std::string& action_name, const std::string& stage_name = "generate grasp pose",
                int goal_timeout = 0, int server_timeout = 0);

  /**
   * @brief Composes the action goal and sends to server
   */
  void composeGoal();

  /**
   * @brief Monitors status of action goal
   * @return true if grasp candidates are received within (optional) timeout
   * @details This is a blocking call. It will wait until either grasp candidates
   *          are received or the timeout has been reached.
   */
  bool monitorGoal();
  void goal_response_callback(const GoalHandleSharedPtr& goal_handle) override;
  void feedback_callback(const GoalHandleSharedPtr goal_handle,
                         const std::shared_ptr<const Feedback> feedback) override;
  void result_callback(const WrappedResult& result) override;

  void init(const core::RobotModelConstPtr& robot_model) override;
  void compute() override;

  void setEndEffector(const std::string& eef)
  {
    setProperty("eef", eef);
  }
  void setObject(const std::string& object)
  {
    setProperty("object", object);
  }

  void setPreGraspPose(const std::string& pregrasp)
  {
    properties().set("pregrasp", pregrasp);
  }
  void setPreGraspPose(const moveit_msgs::msg::RobotState& pregrasp)
  {
    properties().set("pregrasp", pregrasp);
  }
  void setGraspPose(const std::string& grasp)
  {
    properties().set("grasp", grasp);
  }
  void setGraspPose(const moveit_msgs::msg::RobotState& grasp)
  {
    properties().set("grasp", grasp);
  }

protected:
  void onNewSolution(const SolutionBase& s) override;

private:
  bool found_candidates_;
  std::vector<geometry_msgs::msg::PoseStamped> grasp_candidates_;
  std::vector<double> costs_;
  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit