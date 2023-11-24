#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <dgl_ros/observer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// Questions
// Should action_producer request observations to begin being made?
// OR should obs be made continuously
namespace dgl_ros
{
template <class ObsT, class SrcT>
Observer<ObsT, SrcT>::Observer(const rclcpp::NodeOptions& options,
                               std::function<std::unique_ptr<ObsT>(const SrcT&)> obs_from_src) : Observer(options)
 
{
  obs_pub_ = this->create_publisher<ObsT>("observation", 10);  // For debugging.

  auto callback = [this, obs_from_src](const std::shared_ptr<SrcT> msg) {
    std::unique_lock lock(obs_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Observer is busy. Skipping observation.");
      return;
    }
    
    last_observation_ = std::move(obs_from_src(*msg));

    if (this->get_parameter("publish").as_bool())
    {
      obs_pub_->publish(*last_observation_);
    }
  };
  src_sub_ = this->create_subscription<SrcT>(this->get_parameter("src_topic").as_string(), 10, callback);
}
template class Observer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>::Observer;
}  // namespace dgl_ros