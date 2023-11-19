
// ROS
#include <ros_dgl/components/grasp_generator.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
using std::placeholders::_1;
using std::placeholders::_2;
GraspGenerator::GraspGenerator(
    std::function<SampleGraspPoses::Feedback::SharedPtr(void)> grasp_generator, bool& goal_active)
  : rclcpp::Node("grasp_generator_server"), goal_active_(goal_active)
{
  grasp_generator_ = grasp_generator;
  RCLCPP_INFO(this->get_logger(), "Grasp detection action server ready");
  // tfBuffer_ = std::make_unique<tf2_ros::Buffer>(action_node_->get_clock());
  // tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  // timer_interface_ = std::make_shared<tf2_ros::CreateTimerROS>(action_node_->get_node_base_interface(),
  //                                                              action_node_->get_node_timers_interface());
  // tfBuffer_->setCreateTimerInterface(timer_interface_);
  // result_ = std::make_shared<deep_grasp_msgs::action::SampleGraspPoses::Result>();

  server_ = rclcpp_action::create_server<SampleGraspPoses>(
      this, "sample_grasp_poses", std::bind(&GraspGenerator::handle_goal, this, _1, _2),
      std::bind(&GraspGenerator::handle_cancel, this, _1),
      std::bind(&GraspGenerator::handle_accepted, this, _1));

  // grasp_detector_ = std::make_unique<GraspDetector>(path_to_model_config_))
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

void GraspGenerator::handle_accepted(const GoalHandleSharedPtr& goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "New goal accepted");
  goal_active_ = true;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&GraspGenerator::execute, this, _1), goal_handle }.detach();
}

void GraspGenerator::execute(const GoalHandleSharedPtr& goal_handle)
{
  goal_handle->publish_feedback(grasp_generator_());
}



// Uncomment below to be able to load this component as a plugin in a running process.
// See https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html?highlight=composable
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// RCLCPP_COMPONENTS_REGISTER_NODE(GraspGenerator)
