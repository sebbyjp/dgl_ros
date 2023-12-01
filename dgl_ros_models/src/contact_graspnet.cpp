#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <dgl_ros/util/geometry.hpp>
#include <dgl_ros/util/cloud.hpp>
#include <dgl_ros_models/contact_graspnet.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
using dgl_ros_interfaces::action::SampleGraspPoses;
using sensor_msgs::msg::PointCloud2;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

namespace dgl_models
{
ContactGraspnet::ContactGraspnet(rclcpp::NodeOptions& options) : CgnAgent(options)
{
  this->declare_parameter("tf_timeout_seconds", 5);
  this->declare_parameter("visualize", false);
  this->declare_parameter("success_threshold", 0.5);
  this->declare_parameter("remove_centroid", true);
  this->declare_parameter("max_grasps", 10);
  this->declare_parameter<std::vector<double>>("grasp_model_tf", { 0.0, 0.0, 0.0, M_PI_2, -M_PI_2, 0.0 });

  auto tf_timeout_seconds = this->get_parameter("tf_timeout_seconds").as_int();
  auto src_frame0 = this->get_parameter("src_frame0").as_string();
  auto world_frame = this->get_parameter("world_frame").as_string();

  visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(world_frame, "/rviz_visual_markers", this);
  tf_lookup_ = std::make_unique<dgl::util::TransformLookup>(this);
  tf_lookup_->get_tf_affine(world_frame, src_frame0, tf_timeout_seconds, tf_world_src_);
}

SampleGraspPoses::Feedback::SharedPtr ContactGraspnet::actionFromObs(std::shared_ptr<CgnObserver> observer)
{

  auto visualize = this->get_parameter("visualize").as_bool();
  auto success_threshold = this->get_parameter("success_threshold").as_double();
  auto max_grasps = this->get_parameter("max_grasps").as_int();
  auto remove_centroid = this->get_parameter("remove_centroid").as_bool();
  auto world_frame = this->get_parameter("world_frame").as_string();
  auto grasp_model_tf = dgl::util::isometryFromXYZRPY(
    this->get_parameter("grasp_model_tf").as_double_array());
  auto [id, msg] = observer->observe();

  // Convert to python-accessible representation.
  PointCloudRGB cloud;
  pcl::fromROSMsg(*msg, cloud);
  std::vector<std::array<float, 3>> cloud_points;
  for (const auto& point : cloud.points)
  {
    cloud_points.push_back({ point.x, point.y, point.z });
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "cloud size: " << cloud.points.size());

  // TODO(speralta): Read in onnx module and run inference
  // For now, access cgn-pytorch in an embedded python interpreter.
  std::vector<Eigen::Matrix4d> grasp_list;
  std::vector<double> confidence_list;
  {
    namespace py = pybind11;
    py::scoped_interpreter guard{};
    py::module sys = py::module::import("sys");
    sys.attr("path").attr("append")("/sim_ws/src/cgn_pytorch");

    py::function from_pretrained = py::module::import("cgn_pytorch").attr("from_pretrained");
    py::tuple model_tup = from_pretrained();

    py::function inference = py::module::import("cgn_pytorch").attr("inference");
    py::tuple result = inference(model_tup[0], py::cast(cloud_points), success_threshold, visualize, max_grasps);

    grasp_list = result[0].cast<std::vector<Eigen::Matrix4d>>();
    confidence_list = result[1].cast<std::vector<double>>();
  }

  // Detect grasp poses.
  std::vector<geometry_msgs::msg::PoseStamped> grasps;
  std::vector<unsigned int> grasp_ids;
  RCLCPP_INFO_STREAM(this->get_logger(), "grasp_list size: " << grasp_list.size());
  for (unsigned int i = 0; i < grasp_list.size(); i++)
  {
    grasp_ids.push_back(i);
    geometry_msgs::msg::Pose pose;
    const auto cgn_grasp = Eigen::Affine3d(grasp_list[i]);
    if (visualize)
    {
      visual_tools_->publishAxisLabeled(tf2::toMsg(cgn_grasp), "cgn_frame");
    }

    geometry_msgs::msg::PoseStamped grasp;
    grasp.header.frame_id = world_frame;
    grasp.pose = tf2::toMsg(grasp_model_tf * cgn_grasp);

    if (remove_centroid)
    {
      grasp.pose.position.x += centroid_[0];
      grasp.pose.position.y += centroid_[1];
      grasp.pose.position.z += centroid_[2];
    }
    grasps.push_back(grasp);
    if (visualize)
    {
      visual_tools_->publishAxisLabeled(grasp.pose, std::to_string(confidence_list[i]));
      visual_tools_->trigger();
    }
  }
  

  auto feedback = std::make_shared<SampleGraspPoses::Feedback>();
  for (auto id : grasp_ids)
  {
    feedback->grasp_candidates.emplace_back(grasps[id]);
    feedback->costs.emplace_back(1.0 / confidence_list[id]);
  }

  return feedback;
}

std::unique_ptr<PointCloud2> ContactGraspnet::obsFromSrcs(std::shared_ptr<PointCloud2> msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Creating observation.");
  auto remove_centroid = this->get_parameter("remove_centroid").as_bool();
  // Convert to PCL
  PointCloudRGB cloud;
  pcl::fromROSMsg(*msg, cloud);

  // Transform to world frame
  auto world_cloud = std::make_shared<PointCloudRGB>();
  pcl::transformPointCloud(cloud, *world_cloud, tf_world_src_);
  world_cloud->header.frame_id = "world";

  // Filter workspace.
  dgl::util::cloud::passThroughFilter({ 0.3, -0.5, 0.005 }, { 0.8, 0.5, 0.5 }, world_cloud);
  dgl::util::cloud::removeTable(world_cloud);
  if (!remove_centroid)
  {
    // Return the final observation
    std::unique_ptr<PointCloud2> cloud_msg = std::make_unique<PointCloud2>();
    pcl::toROSMsg(*world_cloud, *cloud_msg);
    return cloud_msg;
  }
  // Compute centroid
  pcl::compute3DCentroid(*world_cloud, centroid_);
  // Remove centroid.
  PointCloudRGB demeaned_cloud;
  pcl::demeanPointCloud(*world_cloud, centroid_, demeaned_cloud);

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
  dgl_models::ContactGraspnet server(options);
  server.run();
  rclcpp::shutdown();
  return 0;
}