#include <rclcpp/rclcpp.hpp>
#include <ros_dgl/components/grasp_generator.hpp>
#include <ros_dgl/components/sensor_listener.hpp>
#include <deep_grasp_msgs/action/sample_grasp_poses.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <deep_grasp_msgs/action/sample_grasp_poses.hpp>
#include <Eigen/Dense>
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros_dgl/util/pose.hpp>
#include <ros_dgl/util/sensors/cloud.hpp>

using deep_grasp_msgs::action::SampleGraspPoses;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;
using std::placeholders::_2;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;

class GraspDetectionServer : public rclcpp::Node
{
    public:

    GraspDetectionServer(rclcpp::NodeOptions& options);


SampleGraspPoses::Feedback::SharedPtr SampleGrasps();

void CloudCallback(rclcpp::Publisher<PointCloud2>::SharedPtr pub,const PointCloud2::SharedPtr& msg);

bool GoalActive() const {
      std::lock_guard<std::mutex> lock(goal_active_mutex_);
  return goal_active_;
}

gpd::util::Cloud* CloudCam() const {
  std::lock_guard<std::mutex> lock(cloud_cam_mutex_);
  return cloud_cam_.get();
}

void SetCloudCam(std::unique_ptr<gpd::util::Cloud> cloud_cam) {
  std::lock_guard<std::mutex> lock(cloud_cam_mutex_);
  cloud_cam_ = std::move(cloud_cam);
}

void Run();

 private:
    mutable std::mutex goal_active_mutex_;
    bool goal_active_ RCPPUTILS_TSA_GUARDED_BY(goal_active_mutex_) = false;
    mutable std::mutex cloud_cam_mutex_;
    std::unique_ptr<gpd::util::Cloud> cloud_cam_  RCPPUTILS_TSA_GUARDED_BY(cloud_cam_mutex_);
    std::unique_ptr<gpd::GraspDetector> grasp_detector_;
    std::shared_ptr<GraspGenerator> grasp_generator_;
    std::shared_ptr<SensorListener<PointCloud2, PointCloud2>> sensor_listener_;
    Eigen::Isometry3d transform_base_opt_;
};

