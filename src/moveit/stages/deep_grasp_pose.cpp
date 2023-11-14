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

#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <deep_grasp_task/stages/deep_grasp_pose.h>
#include <deep_grasp_msgs/action/sample_grasp_poses.hpp>
#include <functional>
#include <rclcpp_action/rclcpp_action.hpp>

namespace moveit
{
namespace task_constructor
{
namespace stages
{

template <class ActionSpec>
DeepGraspPose<ActionSpec>::DeepGraspPose(const std::string& action_name, const std::string& stage_name,
                                         int goal_timeout, int server_timeout)
  : GeneratePose(stage_name), ActionBaseT(action_name, false, goal_timeout, server_timeout), found_candidates_(false)
{
  auto& p = properties();
  p.declare<std::string>("eef", "name of end-effector");
  p.declare<std::string>("object");
  p.declare<boost::any>("pregrasp", "pregrasp posture");
  p.declare<boost::any>("grasp", "grasp posture");

  RCLCPP_INFO(ActionBaseT::nh_->get_logger(), "Waiting for connection to grasp generation action server...");
  ActionBaseT::client_ptr_->wait_for_action_server();
  RCLCPP_INFO(ActionBaseT::nh_->get_logger(), "Connected to server");
  // visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(ActionBaseT::nh_);
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::composeGoal()
{
  typename ActionSpec::Goal goal;
  goal.action_name = ActionBaseT::action_name_;
  typename rclcpp_action::Client<ActionSpec>::SendGoalOptions send_goal_options;
  send_goal_options = typename rclcpp_action::Client<ActionSpec>::SendGoalOptions();
  using namespace std::placeholders;
  send_goal_options.goal_response_callback = std::bind(&DeepGraspPose<ActionSpec>::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&DeepGraspPose<ActionSpec>::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&DeepGraspPose<ActionSpec>::result_callback, this, _1);

  auto result = ActionBaseT::client_ptr_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(ActionBaseT::nh_->get_logger(), "Goal sent to server: %s", ActionBaseT::action_name_.c_str());
}

template <class ActionSpec>
bool DeepGraspPose<ActionSpec>::monitorGoal()
{
  // monitor timeout
  const bool monitor_timeout = ActionBaseT::goal_timeout_ > std::numeric_limits<double>::epsilon() ? true : false;
  const double timeout_time = ActionBaseT::nh_->get_clock()->now().seconds() + ActionBaseT::goal_timeout_;
  while (rclcpp::ok())
  {
    rclcpp::spin_some(ActionBaseT::nh_);

    // timeout reached
    if (ActionBaseT::nh_->get_clock()->now().seconds() > timeout_time && monitor_timeout)
    {
      ActionBaseT::client_ptr_->async_cancel_all_goals();
      RCLCPP_ERROR(ActionBaseT::nh_->get_logger(), "Grasp pose generator time out reached");
      return false;
    }
    else if (found_candidates_)
    {
      break;
    }
  }
  return true;
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::goal_response_callback(const GoalHandleSharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(ActionBaseT::nh_->get_logger(), "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(ActionBaseT::nh_->get_logger(), "Goal accepted by server, waiting for result");
  }
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::feedback_callback(const GoalHandleSharedPtr goal_handle,
                                                  const std::shared_ptr<const Feedback> feedback)
{
  // each candidate should have a cost
  if (feedback->grasp_candidates.size() != feedback->costs.size())
  {
    RCLCPP_ERROR(ActionBaseT::nh_->get_logger(), "Invalid input: each grasp candidate needs an associated cost");
  }
  else
  {
    RCLCPP_INFO(ActionBaseT::nh_->get_logger(),
                "Grasp generated feedback received %lu candidates: ", feedback->grasp_candidates.size());

    grasp_candidates_.resize(feedback->grasp_candidates.size());
    costs_.resize(feedback->costs.size());

    grasp_candidates_ = feedback->grasp_candidates;
    costs_ = feedback->costs;

    found_candidates_ = true;
  }
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::result_callback(const WrappedResult& result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO(ActionBaseT::nh_->get_logger(), "Found grasp candidates (result): %s",
                result.result->grasp_state.c_str());
  }
  else
  {
    RCLCPP_ERROR(ActionBaseT::nh_->get_logger(), "No grasp candidates found (state): %s",
                 result.result->grasp_state.c_str());
  }
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::init(const core::RobotModelConstPtr& robot_model)
{
  InitStageException errors;
  try
  {
    GeneratePose::init(robot_model);
  }
  catch (InitStageException& e)
  {
    errors.append(e);
  }

  const auto& props = properties();

  // check availability of object
  props.get<std::string>("object");
  // check availability of eef
  const std::string& eef = props.get<std::string>("eef");
  if (!robot_model->hasEndEffector(eef))
  {
    errors.push_back(*this, "unknown end effector: " + eef);
  }
  else
  {
    // check availability of eef pose
    const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
    const std::string& name = props.get<std::string>("pregrasp");
    std::map<std::string, double> m;
    if (!jmg->getVariableDefaultPositions(name, m))
    {
      errors.push_back(*this, "unknown end effector pose: " + name);
    }
  }

  if (errors)
  {
    throw errors;
  }
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::compute()
{
  if (upstream_solutions_.empty())
  {
    return;
  }
  planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

  // set end effector pose
  const auto& props = properties();
  const std::string& eef = props.get<std::string>("eef");
  const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

  core::RobotState& robot_state = scene->getCurrentStateNonConst();
  robot_state.setToDefaultValues(jmg, props.get<std::string>("pregrasp"));

  // compose/send goal
  composeGoal();

  // monitor feedback/results
  // blocking function untill timeout reached or results received
  if (monitorGoal())
  {
    // ROS_WARN_NAMED(ActionBaseT::nh_->get_logger(), "number %lu: ",grasp_candidates_.size());
    for (unsigned int i = 0; i < grasp_candidates_.size(); i++)
    {
      InterfaceState state(scene);
      state.properties().set("target_pose", grasp_candidates_.at(i));
      props.exposeTo(state.properties(), { "pregrasp", "grasp" });

      SubTrajectory trajectory;
      trajectory.setCost(costs_.at(i));
      trajectory.setComment(std::to_string(i));

      // add frame at target pose
      rviz_marker_tools::appendFrame(trajectory.markers(), grasp_candidates_.at(i), 0.1, "grasp frame");
      // visual_tools_->publishAxis(grasp_candidates_.at(i).pose);
      // visual_tools.trigger();

      spawn(std::move(state), std::move(trajectory));
    }
    // visual_tools_->trigger();
  }
}

template <class ActionSpec>
void DeepGraspPose<ActionSpec>::onNewSolution(const SolutionBase& s)
{
  planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

  const auto& props = properties();
  const std::string& object = props.get<std::string>("object");
  if (!scene->knowsFrameTransform(object))
  {
    const std::string msg = "object '" + object + "' not in scene";
    if (storeFailures())
    {
      InterfaceState state(scene);
      SubTrajectory solution;
      solution.markAsFailure();
      solution.setComment(msg);
      spawn(std::move(state), std::move(solution));
    }
    else
    {
      RCLCPP_WARN_STREAM(ActionBaseT::nh_->get_logger(), msg);
    }
    return;
  }

  upstream_solutions_.push(&s);
}

// Explicit template instantiation
template class DeepGraspPose<deep_grasp_msgs::action::SampleGraspPoses>;

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit