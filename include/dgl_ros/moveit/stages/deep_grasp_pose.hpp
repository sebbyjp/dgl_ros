/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Boston Cleek
   Desc:   Grasp generator stage using deep learning based grasp synthesizers
*/

#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>
#include <deep_grasp_task/stages/action_base.h>
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
template <typename ActionSpec>
class DeepGraspPose : public GeneratePose
{
private:
  using Feedback = typename ActionSpec::Feedback;
  using Result = typename ActionSpec::Result;
  using GoalHandleSharedPtr = typename rclcpp_action::ClientGoalHandle<ActionSpec>::SharedPtr;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionSpec>::WrappedResult;

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