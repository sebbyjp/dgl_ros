#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/actor.hpp>
#include <dgl_ros/observer.hpp>
#include <dgl_ros/agent.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <Eigen/Dense>
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dgl_ros/util/geometry.hpp>
#include <dgl_ros/util/cloud.hpp>
#include <dgl_ros_models/contact_graspnet.hpp>

using std::placeholders::_1;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;

namespace dgl_ros
{
  
ContactGraspnet::ContactGraspnet(rclcpp::NodeOptions& options)
  : CgnAgent(
        options.parameter_overrides({ { "src_topic", "rgbd_camera/points" }, { "publish", true }, { "action_topic", "sample_grasp_poses"}}))
{
  const Eigen::Isometry3d trans_base_cam = util::IsometryFromXYZRPY({ 0.084, 0.017, 0.522, 0, 0.8, 0 });
  const Eigen::Isometry3d transform_cam_opt = util::IsometryFromXYZRPY({ 0, 0, 0, 0, 0, 0 });
  transform_base_opt_ = trans_base_cam * transform_cam_opt;
  gpd_grasp_detector_ = std::make_unique<gpd::GraspDetector>("/simply_ws/src/dgl_ros/dgl_ros/config/gpd_config.yaml");
}

SampleGraspPoses::Feedback::SharedPtr
ContactGraspnet::actionFromObs(std::shared_ptr<CgnObserver> observer)
{
  while (observer->getObservation() == nullptr)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  auto l = observer->getSharedLock();

  // Convert to PCL.
  PointCloudRGB cloud;
  pcl::moveFromROSMsg(*observer->getObservation(), cloud);
  l.unlock();
  
  auto feedback = std::make_shared<SampleGraspPoses::Feedback>();
  

  return feedback;
}

std::unique_ptr<PointCloud2> ContactGraspnet::obsFromSrcs(const PointCloud2& msg)
{
  return std::make_unique<PointCloud2>(msg);
}

}  // namespace dgl_ros_models
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  dgl_ros_models::ContactGraspnet server(options);
  server.run();
  rclcpp::shutdown();
  return 0;
}