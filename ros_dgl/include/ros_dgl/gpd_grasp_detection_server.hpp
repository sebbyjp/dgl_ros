#include <rclcpp/rclcpp.hpp>
#include <ros_dgl/grasp_generator.hpp>
#include <ros_dgl/observer.hpp>
#include <ros_dgl_interfaces/action/sample_grasp_poses.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros_dgl/util/pose.hpp>
#include <ros_dgl/util/sensors/cloud.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
// TODO(speralta): Make generic with action, observation, and sensor the template parameters.

namespace ros_dgl {
class GPDGraspDetectionServer : public rclcpp::Node
{
  public:

    GPDGraspDetectionServer(rclcpp::NodeOptions& options);


    ros_dgl_interfaces::action::SampleGraspPoses::Feedback::SharedPtr sampleGrasps();

std::unique_ptr<sensor_msgs::msg::PointCloud2> preprocessCloud(const sensor_msgs::msg::PointCloud2& msg);


void Run();

 private:
    std::unique_ptr<gpd::GraspDetector> gpd_grasp_detector_;
    std::shared_ptr<GraspGenerator> grasp_generator_;
    std::shared_ptr<Observer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>> observer_;
    Eigen::Isometry3d transform_base_opt_;
};

} // namespace ros_dgl