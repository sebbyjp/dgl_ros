#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/components/sensor_listener.hpp>
#include <dgl_ros/components/grasp_generator.hpp>
#include <deep_grasp_msgs/action/sample_grasp_poses.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <deep_grasp_msgs/action/sample_grasp_poses.hpp>
#include <Eigen/Dense>
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dgl_ros/util/pose.hpp>
#include <dgl_ros/util/sensors/cloud.hpp>
#include <dgl_ros/grasp_detection_server.hpp>
using deep_grasp_msgs::action::SampleGraspPoses;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;
using std::placeholders::_2;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;

template <class SensorMsgT, class ObsMsgT, class ObsT, class ActionT, class GraspDetectorT>
GraspDetectionServer<SensorMsgT, ObsMsgT, ObsT, ActionT, GraspDetectorT>::GraspDetectionServer(rclcpp::NodeOptions& options)
  : rclcpp::Node("grasp_detection_server", options)
{
  const Eigen::Isometry3d trans_base_cam = dgl_ros::util::IsometryFromXYZRPY({ 0.084, 0.017, 0.522, 0, 0.8, 0 });
  const Eigen::Isometry3d transform_cam_opt = dgl_ros::util::IsometryFromXYZRPY({ 0, 0, 0, 0, 0, 0 });
  transform_base_opt_ = trans_base_cam * transform_cam_opt;
  grasp_detector_ = std::make_unique<GraspDetectorT("/simply_ws/src/dgl_ros/config/gpd_config.yaml");
  grasp_generator_ = std::make_shared<GraspGenerator>(std::bind(&GraspDetectionServer::SampleGrasps, this), goal_active_);
  sensor_listener_ = std::make_shared<SensorListener<PointCloud2, PointCloud2>>(
      "rgbd_camera/points", "processed_sensor_data", std::bind(&GraspDetectionServer::SensorCallback, this, _1, _2));
}

void GraspDetectionServer<SensorMsgT, ObsMsgT, ObsT, ActionT, GraspDetectorT>::HandleAccepted(const GoalHandleSharedPtr& goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "New goal accepted");
  goal_active_ = true;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{ std::bind(&GraspGenerator::execute, this, _1), goal_handle }.detach();
}

// void GraspGenerator::execute(const GoalHandleSharedPtr& goal_handle)
// {
//   goal_handle->publish_feedback(grasp_generator_());
// }

SampleGraspPoses::Feedback::SharedPtr GraspDetectionServer::SampleGrasps()
{
  while (CloudCam() == nullptr)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  gpd::util::Cloud cloud_cam = *CloudCam();

  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;              // detect grasp poses
  grasp_detector_->preprocessPointCloud(cloud_cam, transform_base_opt_);  // preprocess the point cloud
  grasps = grasp_detector_->detectGrasps(cloud_cam);                      // detect grasps in the point cloud

  std::vector<unsigned int> grasp_ids;
  for (unsigned int i = 0; i < grasps.size(); i++)
  {
    grasp_ids.push_back(i);
  }
  auto feedback = std::make_shared<SampleGraspPoses::Feedback>();
  if (grasp_ids.empty())
  {
    return feedback;
  }

  for (auto id : grasp_ids)
  {
    // transform grasp from camera optical link into frame_id (panda_link0)
    const Eigen::Isometry3d transform_opt_grasp =
        Eigen::Translation3d(grasps.at(id)->getPosition()) * Eigen::Quaterniond(grasps.at(id)->getOrientation());

    const Eigen::Isometry3d transform_base_grasp = transform_base_opt_ * transform_opt_grasp;
    const Eigen::Vector3d trans = transform_base_grasp.translation();
    const Eigen::Quaterniond rot(transform_base_grasp.rotation());

    // convert back to PoseStamped
    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = "world";
    grasp_pose.pose.position.x = trans.x();
    grasp_pose.pose.position.y = trans.y();
    grasp_pose.pose.position.z = trans.z();

    grasp_pose.pose.orientation.w = rot.w();
    grasp_pose.pose.orientation.x = rot.x();
    grasp_pose.pose.orientation.y = rot.y();
    grasp_pose.pose.orientation.z = rot.z();

    feedback->grasp_candidates.emplace_back(grasp_pose);

    // Grasp is selected based on cost not score
    // Invert score to represent grasp with lowest cost
    feedback->costs.emplace_back(static_cast<double>(1.0 / grasps.at(id)->getScore()));
  }

  return feedback;
}

void GraspDetectionServer::CloudCallback(rclcpp::Publisher<PointCloud2>::SharedPtr pub,
                                         const PointCloud2::SharedPtr& msg)
{
  if (goal_active_)
  {
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    pcl::fromROSMsg(*msg.get(), *cloud.get());

    // Segementation works best with XYXRGB
    dgl_ros::cloud_util::removeTable(cloud);

    // publish the cloud for visualization and debugging purposes
    PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud.get(), cloud_msg);
    pub->publish(cloud_msg);

    // TODO: set alpha channel to 1
    // GPD required XYZRGBA
    PointCloudRGBA::Ptr grasp_cloud = std::make_shared<PointCloudRGBA>();
    pcl::copyPointCloud(*cloud.get(), *grasp_cloud.get());

    // Construct the cloud camera
    Eigen::Matrix3Xd camera_view_point(3, 1);
    SetCloudCam(std::make_unique<gpd::util::Cloud>(grasp_cloud, 0, camera_view_point));
  }

  goal_active_ = false;
}

void GraspDetectionServer::Run()
{
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(grasp_generator_);
  exec.add_node(sensor_listener_);
  exec.spin();

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  GraspDetectionServer server(options);
  server.Run();
  rclcpp::shutdown();
  return 0;
}