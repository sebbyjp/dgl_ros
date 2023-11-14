
// ROS
#include <dgl_ros/components/grasp_generator.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
GraspGenerator::GraspGenerator(
    std::function<void(const GoalHandleSharedPtr& goal_handle)> grasp_generator)
  : rclcpp::Node("grasp_generator_server")
{
  RCLCPP_INFO(this->get_logger(), "Grasp detection action server ready");

  server_ = rclcpp_action::create_server<SampleGraspPoses>(
      this, "sample_grasp_poses", std::bind(&GraspGenerator::handle_goal, this, _1, _2),
      std::bind(&GraspGenerator::handle_cancel, this, _1),
      grasp_generator);

}

rclcpp_action::CancelResponse GraspGenerator::handle_cancel(const GoalHandleSharedPtr goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse GraspGenerator::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                              std::shared_ptr<const SampleGraspPoses::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// void GraspGenerator::handle_accepted(const GoalHandleSharedPtr& goal_handle)
// {
//   RCLCPP_INFO(this->get_logger(), "New goal accepted");
//   goal_active_ = true;
//   // this needs to return quickly to avoid blocking the executor, so spin up a new thread
//   std::thread{ std::bind(&GraspGenerator::execute, this, _1), goal_handle }.detach();
// }

// void GraspGenerator::execute(const GoalHandleSharedPtr& goal_handle)
// {
//   goal_handle->publish_feedback(grasp_generator_());
// }


