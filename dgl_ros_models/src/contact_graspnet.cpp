#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <dgl_ros_interfaces/action/sample_grasp_poses.hpp>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <dgl_ros/util/geometry.hpp>
#include <dgl_ros_models/contact_graspnet.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>
#include <tf2_eigen/tf2_eigen.hpp>
using dgl_ros_interfaces::action::SampleGraspPoses;
using sensor_msgs::msg::PointCloud2;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

namespace dgl_models
{

ContactGraspnet::ContactGraspnet(rclcpp::NodeOptions& options) : CgnAgent(options)
{
  tf_lookup_ = std::make_unique<dgl::util::TransformLookup>(this);
  this->declare_parameter("tf_timeout_seconds", 5);
  tf_lookup_->get_tf_affine(this->get_parameter("world_frame").as_string(),
                            this->get_parameter("src_frame0").as_string(),
                            this->get_parameter("tf_timeout_seconds").as_int(), tf_world_src_);
}

SampleGraspPoses::Feedback::SharedPtr ContactGraspnet::actionFromObs(std::shared_ptr<CgnObserver> observer)
{
  auto [id, msg] = observer->observe();
  // Convert to PCL.
  PointCloudRGB cloud;
  pcl::fromROSMsg(*msg, cloud);

  std::vector<std::array<float, 3>> cloud_points;
  for (const auto& point : cloud.points)
  {
    cloud_points.push_back({ point.x, point.y, point.z });
  }
  RCLCPP_WARN_STREAM(this->get_logger(), "cloud size" << cloud.points.size());
  std::vector<geometry_msgs::msg::PoseStamped> grasps;  // detect grasp poses
                                                        // TODO(speralta): read in onnx module and run inference
  // // Create a feedback message
  // namespace py = pybind11;
  // py::scoped_interpreter guard{};
  // py::module sys = py::module::import("sys");
  // sys.attr("path").attr("append")("/simply_ws/src/cgn_pytorch");
  // py::object cgn_from_pretrained = py::module::import("cgn_pytorch").attr("from_pretrained");
  // py::tuple cgn_tup = cgn_from_pretrained();
  // py::object cgn_infer = py::module::import("cgn_pytorch").attr("inference");
  // py::tuple result = cgn_infer(cgn_tup[0], py::cast(cloud_points));
  // py::list grasp_list = result[0];
  // py::list confidence_list = result[1];
  // std::vector<unsigned int> grasp_ids;
  // for (unsigned int i = 0; i < grasp_list.size(); i++)
  // {
  //   grasp_ids.push_back(i);
  //   geometry_msgs::msg::PoseStamped grasp;
  //   grasp.header.frame_id = "world";
  //   RCLCPP_WARN_STREAM(this->get_logger(), "Grasp " << i << " " << grasp_list[i].cast<Eigen::Matrix4d>());
  //   grasp.pose = tf2::toMsg(Eigen::Affine3d(grasp_list[i].cast<Eigen::Matrix4d>()));
  //   grasps.push_back(grasp);
  // }
  auto feedback = std::make_shared<SampleGraspPoses::Feedback>();
  // for (auto id : grasp_ids)
  // {
  //   feedback->grasp_candidates.emplace_back(grasps[id]);
  //   feedback->costs.emplace_back(1.0 / confidence_list[id].cast<double>());
  // }

  return feedback;
}

std::unique_ptr<PointCloud2> ContactGraspnet::obsFromSrcs(std::shared_ptr<PointCloud2> msg)
{
  // Convert to PCL
  PointCloudRGB cloud;
  pcl::fromROSMsg(*msg, cloud);
  // Transform to world frame
  PointCloudRGB world_cloud;

  pcl::transformPointCloud(cloud, world_cloud, tf_world_src_);
  world_cloud.header.frame_id = "world";
  // Compute centroid
  // Eigen::Vector4f centroid;
  // pcl::compute3DCentroid(world_cloud, centroid);

  // // Remove centroid.
  // PointCloudRGB demeaned_cloud;
  // pcl::demeanPointCloud(world_cloud, centroid, demeaned_cloud);

  // Return the final observation
  std::unique_ptr<PointCloud2> cloud_msg = std::make_unique<PointCloud2>();
  pcl::toROSMsg(world_cloud, *cloud_msg);
  cloud_msg->header.stamp = rclcpp::Clock().now();
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