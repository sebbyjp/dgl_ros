#include <rclcpp/rclcpp.hpp>
#include <dgl_ros/components/grasp_generator.hpp>
#include <dgl_ros/components/sensor_listener.hpp>
#include <deep_grasp_msgs/action/sample_grasp_poses.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <deep_grasp_msgs/action/sample_grasp_poses.hpp>
#include <Eigen/Dense>
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dgl_ros/util/pose.hpp>
#include <dgl_ros/util/sensors/cloud.hpp>

using deep_grasp_msgs::action::SampleGraspPoses;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;
using std::placeholders::_2;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;

template <typename SensorMsgT, typename ObsMsgT, typename ObsT, typename ActionT, typename GraspDetectorT,
          typename GraspT>
class GraspDetectionServer : public rclcpp::Node
{
public:
  GraspDetectionServer(rclcpp::NodeOptions& options) : rclcpp::Node("grasp_detection_server", options)
  {
    const Eigen::Isometry3d trans_base_cam = dgl_ros::util::IsometryFromXYZRPY({ 0.084, 0.017, 0.522, 0, 0.8, 0 });
    const Eigen::Isometry3d transform_cam_opt = dgl_ros::util::IsometryFromXYZRPY({ 0, 0, 0, 0, 0, 0 });
    transform_base_opt_ = trans_base_cam * transform_cam_opt;
    grasp_detector_ = std::make_unique < GraspDetectorT>("/simply_ws/src/dgl_ros/config/gpd_config.yaml");
    grasp_generator_ =
        std::make_shared<GraspGenerator>(std::bind(&GraspDetectionServer::handleAccepted, this)));
    sensor_listener_ = std::make_shared<SensorListener<PointCloud2, PointCloud2>>(
        "rgbd_camera/points", "processed_sensor_data", std::bind(&GraspDetectionServer::SensorCallback, this, _1, _2));
  }

  typedef std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> GoalHandleSharedPtr;
  void handleAccepted(const GoalHandleSharedPtr& goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "New goal accepted");
    goal_active_ = true;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&GraspGenerator::execute, this, _1), goal_handle }.detach();
  }

  void detectGrasps(const GoalHandleSharedPtr& goal_handle)
  {
    while (Observation() == nullptr)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ObsT observation = *Observation();

    std::vector<std::unique_ptr<GraspT>> grasps;                              // detect grasp poses
    grasp_detector_->preprocessPointCloud(observation, transform_base_opt_);  // preprocess the point cloud
    grasps = grasp_detector_->detectGrasps(observation);                      // detect grasps in the point cloud

    std::vector<unsigned int> grasp_ids;
    for (unsigned int i = 0; i < grasps.size(); i++)
    {
      grasp_ids.push_back(i);
    }
    auto feedback = std::make_shared<ActionT::Feedback>();
    if (grasp_ids.empty())
    {
      goal_handle->abort(std::make_shared<ActionT::Result>());
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
      goal_handle->publish_feedback(feedback);
    }

    void SensorCallback(rclcpp::Publisher<ObsMsgT>::SharedPtr pub, const SensorMsgT& msg)
    {
       if (GoalActive())
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

    bool GoalActive() const
    {
        std::shared_lock lock(goal_active_mutex_);
      return goal_active_;
    }

    void SetGoalActive(bool goal_active)
    {
        std::unique_lock lock(goal_active_mutex_);
      goal_active_ = goal_active;
    }

    std::shared_ptr<ObsT> Observation() const
    {
       std::shared_lock lock(obs_mutex_);
      return obs_.get();
    }

    void SetObservation(std::unique_ptr<ObsT> observation)
    {
      std::unique_lock lock(obs_mutex_);
      obs_ = observation;
    }

    void Run();

  private:
    mutable std::shared_mutex goal_active_mutex_;
    bool goal_active_ RCPPUTILS_TSA_GUARDED_BY(goal_active_mutex_) = false;
    mutable std::shared_mutex obs_mutex_;
    std::shared_ptr<ObsT> obs_ RCPPUTILS_TSA_GUARDED_BY(obs_mutex_);
    std::unique_ptr<GraspDetectorT> grasp_detector_;
    std::shared_ptr<GraspGenerator> grasp_generator_;
    std::shared_ptr<SensorListener<SensorMsgT, ObsMsgT>> sensor_listener_;
    Eigen::Isometry3d transform_base_opt_;
  };
