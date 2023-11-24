#include <rclcpp/rclcpp.hpp>
#include <ros_dgl/grasp_generator.hpp>
#include <ros_dgl/observer.hpp>
#include <ros_dgl_interfaces/action/sample_grasp_poses.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
#include <ros_dgl/util/pose.hpp>
#include <ros_dgl/util/sensors/cloud.hpp>

typedef ros_dgl_interfaces::action::SampleGraspPoses SampleGraspPoses;
typedef sensor_msgs::msg::PointCloud2 PointCloud2;
// TODO(speralta): Make generic with action, observation, and sensor the template parameters.

namespace ros_dgl
{

template <typename ObsT, typename ActionT, typename... SrcTs>
class AgentNode : public rclcpp::Node
{
public:
  AgentNode(rclcpp::NodeOptions& options) : Node("agent", options)
  {
    this->declare_parameter("src_topic", "rgbd_camera/points");
    this->declare_parameter("publish", true);
    this->declare_parameter("action_topic", "sample_grasp_poses");
    observer_ =
        std::make_shared<Observer<ObsT, SrcTs...>>(options, std::bind(&AgentNode::obsFromSrcs, this, std::placeholders::_1));
    action_producer_ = std::make_shared<ActionProducer<ActionT>>(std::bind(&AgentNode::actionFromObs, this, observer_));
  }

  virtual typename ActionT::Feedback::SharedPtr actionFromObs(std::shared_ptr<Observer<ObsT, SrcTs...>> observer) = 0;

  virtual std::unique_ptr<ObsT> obsFromSrcs(const SrcTs&... srcs) = 0;

  void run()
  {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(action_producer_);
    exec.add_node(observer_);
    exec.spin();
  }

protected:
  std::shared_ptr<Observer<ObsT, SrcTs...>> observer_;
  std::shared_ptr<ActionProducer<ActionT>> action_producer_;
};

class GPDGraspDetectionServer : public AgentNode<PointCloud2, SampleGraspPoses, PointCloud2>
{
public:
  GPDGraspDetectionServer(rclcpp::NodeOptions& options);

  SampleGraspPoses::Feedback::SharedPtr
  actionFromObs(std::shared_ptr<Observer<PointCloud2, PointCloud2>> observer) override;

  std::unique_ptr<PointCloud2> obsFromSrcs(const PointCloud2& msg) override;

private:
  std::unique_ptr<gpd::GraspDetector> gpd_grasp_detector_;
  Eigen::Isometry3d transform_base_opt_;
};

}  // namespace ros_dgl