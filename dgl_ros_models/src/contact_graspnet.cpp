#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <dgl_ros/util/geometry.hpp>
#include <dgl_ros_models/contact_graspnet.hpp>
using sensor_msgs::msg::PointCloud2;
using dgl_ros_interfaces::action::SampleGraspPoses;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
namespace dgl_models
{
  
ContactGraspnet::ContactGraspnet(rclcpp::NodeOptions& options)
  : CgnAgent(
        options.parameter_overrides({{ "publish", true }, { "action_topic", "sample_grasp_poses"}}), { "rgbd_camera/points" })
{
  const Eigen::Isometry3d trans_base_cam = dgl::util::isometryFromXYZRPY({ 0.084, 0.017, 0.522, 0, 0.8, 0 });
  const Eigen::Isometry3d transform_cam_opt =  dgl::util::isometryFromXYZRPY({ 0, 0, 0, 0, 0, 0 });
  transform_base_opt_ =  transform_cam_opt * trans_base_cam ;
}

SampleGraspPoses::Feedback::SharedPtr
ContactGraspnet::actionFromObs(std::shared_ptr<CgnObserver> observer)
{
  auto [id, msg] = observer->observe();
 
  std::vector<geometry_msgs::msg::PoseStamped> grasps;  // detect grasp poses     
 // TODO(speralta): read in onnx module and run inference
  // Create a feedback message
 std::vector<unsigned int> grasp_ids;
  for (unsigned int i = 0; i < grasps.size(); i++)
  {
    grasp_ids.push_back(i);
  }
  auto feedback = std::make_shared<SampleGraspPoses::Feedback>();
  for (auto id : grasp_ids)
  {
  

    feedback->grasp_candidates.emplace_back(grasps[id]);
    feedback->costs.emplace_back(1.0);
  }

  return feedback;
}

std::unique_ptr<PointCloud2> ContactGraspnet::obsFromSrcs(std::shared_ptr<PointCloud2> msg)
{
  // Convert to PCL
  PointCloudRGB cloud;
  pcl::fromROSMsg(*msg, cloud);
  // Transform to world frame
   PointCloudRGB world_cloud;
 
  Eigen::Affine3d transform_affine = transform_base_opt_;
  transform_affine.makeAffine();
    RCLCPP_WARN_STREAM(this->get_logger(), "Transform aff " << transform_affine.matrix());
  pcl::transformPointCloud(cloud, world_cloud, transform_affine);
  world_cloud.header.frame_id = "world";
  // Compute centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(world_cloud, centroid);
  
  // Remove centroid.
  PointCloudRGB demeaned_cloud;
  pcl::demeanPointCloud(world_cloud, centroid, demeaned_cloud);


  // Return the final observation
  std::unique_ptr<PointCloud2> cloud_msg = std::make_unique<PointCloud2>();
  pcl::toROSMsg(demeaned_cloud, *cloud_msg);
  return cloud_msg;
}
}  // namespace dgl_models

int main(int argc, char** argv)
{
  
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  dgl_models::ContactGraspnet server(options);
  server.run();
  rclcpp::shutdown();
  return 0;
}