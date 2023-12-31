// Copyright (c) 2023 Sebastian Peralta
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <dgl_ros_moveit/stages/deep_grasp_pose.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <functional>
#include <rclcpp_action/rclcpp_action.hpp>

namespace moveit
{
namespace task_constructor
{
namespace stages
{

template <class ActionT>
DeepGraspPose<ActionT>::DeepGraspPose(const std::string& action_name, const std::string& stage_name,
                                         int goal_timeout, int server_timeout)
  : GeneratePose(stage_name), ActionBaseT(action_name, false, goal_timeout, server_timeout), found_candidates_(false)
{
  auto& p = properties();
  p.declare<std::string>("eef", "name of end-effector");
  p.declare<std::string>("object");
  p.declare<boost::any>("pregrasp", "pregrasp posture");
  p.declare<boost::any>("grasp", "grasp posture");

  RCLCPP_INFO(this->nh_->get_logger(), "Waiting for connection to grasp generation action server...");
  this->client_ptr_->wait_for_action_server();
  RCLCPP_INFO(this->nh_->get_logger(), "Connected to server");
  // visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(this->nh_);
}

template <class ActionT>
void DeepGraspPose<ActionT>::composeGoal()
{
  typename ActionT::Goal goal;
  goal.action_name = this->action_name_;
  typename rclcpp_action::Client<ActionT>::SendGoalOptions send_goal_options;
  send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
  using namespace std::placeholders;
  send_goal_options.goal_response_callback = std::bind(&DeepGraspPose<ActionT>::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&DeepGraspPose<ActionT>::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&DeepGraspPose<ActionT>::result_callback, this, _1);

  auto result = this->client_ptr_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(this->nh_->get_logger(), "Goal sent to server: %s", this->action_name_.c_str());
}

template <class ActionT>
bool DeepGraspPose<ActionT>::monitorGoal()
{
  // monitor timeout
  const bool monitor_timeout = this->goal_timeout_ > std::numeric_limits<double>::epsilon() ? true : false;
  const double timeout_time = this->nh_->get_clock()->now().seconds() + this->goal_timeout_;
  while (rclcpp::ok())
  {
    rclcpp::spin_some(this->nh_);

    // timeout reached
    if (this->nh_->get_clock()->now().seconds() > timeout_time && monitor_timeout)
    {
      this->client_ptr_->async_cancel_all_goals();
      RCLCPP_ERROR(this->nh_->get_logger(), "Grasp pose generator time out reached");
      return false;
    }
    else if (found_candidates_)
    {
      break;
    }
  }
  return true;
}

template <class ActionT>
void DeepGraspPose<ActionT>::goal_response_callback(const GoalHandleSharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->nh_->get_logger(), "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->nh_->get_logger(), "Goal accepted by server, waiting for result");
  }
}

template <class ActionT>
void DeepGraspPose<ActionT>::feedback_callback(const GoalHandleSharedPtr goal_handle,
                                                  const std::shared_ptr<const Feedback> feedback)
{
  (void)goal_handle;
  // each candidate should have a cost
  if (feedback->grasp_candidates.size() != feedback->costs.size())
  {
    RCLCPP_ERROR(this->nh_->get_logger(), "Invalid input: each grasp candidate needs an associated cost");
  }
  else
  {
    RCLCPP_INFO(this->nh_->get_logger(),
                "Grasp generated feedback received %lu candidates: ", feedback->grasp_candidates.size());

    grasp_candidates_.resize(feedback->grasp_candidates.size());
    costs_.resize(feedback->costs.size());

    grasp_candidates_ = feedback->grasp_candidates;
    costs_ = feedback->costs;

    found_candidates_ = true;
  }
}

template <class ActionT>
void DeepGraspPose<ActionT>::result_callback(const WrappedResult& result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO(this->nh_->get_logger(), "Found grasp candidates (result): %s",
                result.result->grasp_state.c_str());
  }
  else
  {
    RCLCPP_ERROR(this->nh_->get_logger(), "No grasp candidates found (state): %s",
                 result.result->grasp_state.c_str());
  }
}

template <class ActionT>
void DeepGraspPose<ActionT>::init(const core::RobotModelConstPtr& robot_model)
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

template <class ActionT>
void DeepGraspPose<ActionT>::compute()
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
    // ROS_WARN_NAMED(this->nh_->get_logger(), "number %lu: ",grasp_candidates_.size());
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

template <class ActionT>
void DeepGraspPose<ActionT>::onNewSolution(const SolutionBase& s)
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
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), msg);
    }
    return;
  }

  upstream_solutions_.push(&s);
}

// Explicit template instantiation
template class DeepGraspPose<dgl_ros_interfaces::action::SampleGraspPoses>;

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit