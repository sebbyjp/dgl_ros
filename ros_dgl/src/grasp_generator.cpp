
// ROS
#include <ros_dgl/grasp_generator.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
namespace ros_dgl {
using std::placeholders::_1;
using std::placeholders::_2;
GraspGenerator::GraspGenerator(std::function<SampleGraspPoses::Feedback::SharedPtr()> generate_grasps_func)
  : rclcpp::Node("grasp_generator_server")
{
  generate_grasps_func_ = generate_grasps_func;
  RCLCPP_INFO(this->get_logger(), "Grasp detection action server ready");

  server_ = rclcpp_action::create_server<SampleGraspPoses>(
      this, "sample_grasp_poses", std::bind(&GraspGenerator::handle_goal, this, _1, _2),
      std::bind(&GraspGenerator::handle_cancel, this, _1), std::bind(&GraspGenerator::handle_accepted, this, _1));
}

rclcpp_action::CancelResponse GraspGenerator::handle_cancel(const GoalHandleSharedPtr goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
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

void GraspGenerator::handle_accepted(const GoalHandleSharedPtr& goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "New goal accepted");

  // This needs to return quickly to avoid blocking the executor, so spin up a new thread.
  auto publish_grasps = [this](const GoalHandleSharedPtr& goal_handle) {
    goal_handle->publish_feedback(generate_grasps_func_());
  };
  std::thread{ publish_grasps, goal_handle }.detach();
}
} // namespace ros_dgl