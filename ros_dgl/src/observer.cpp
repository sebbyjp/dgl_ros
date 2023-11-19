#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <queue>
#include "ros_dgl/observer.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ros_dgl_interfaces/srv/observe_cloud.hpp"

// Questions
// Should action_producer request observations to begin being made?
// OR should obs be made continuously
namespace ros_dgl
{
// TODO(speralta):  Set up efficient intra-process communication.
// - See if it's better to pass around a shared ptr, unique_ptr, or UUID for
// clients and subscribers to access observation.
// - See if ros2 type adapters should be used.
template <class SrvT, class ObsT, class Src1T, class Src2T>
Observer<SrvT, ObsT, Src1T, Src2T>::Observer(const rclcpp::NodeOptions& options,
                                             std::function<ObsT(Src1T)> obs_from_src)
  : rclcpp::Node("observer", options)
{
  init();
  obs_srv_ = this->create_service<SrvT>("observe",
                                        [this, obs_from_src](const std::shared_ptr<rmw_request_id_t> request_header,
                                                             const std::shared_ptr<typename SrvT::Request> request,
                                                             const std::shared_ptr<typename SrvT::Response> response) {
                                          auto observation = std::make_shared<ObsT>();
                                          *observation = obs_from_src(*last_src1_);
                                          respond(observation, this->get_parameter("publish").as_bool(), response);
                                        });
}

// TODO(speralta): Make variadic with arbitrary number of source types.
template <class SrvT, class ObsT, class Src1T, class Src2T>
Observer<SrvT, ObsT, Src1T, Src2T>::Observer(
    const rclcpp::NodeOptions& options, std::function<ObsT(Src1T, std::vector<std::shared_ptr<Src2T>>)> obs_from_src,
    int src2_size)
  : rclcpp::Node("observer", options)
{
  init();
  src_listener_node_ = std::make_shared<rclcpp::Node>("src_listener");
  executor_.add_node(src_listener_node_);
  src2_sub_ = src_listener_node_->create_subscription<Src2T>(this->get_parameter("src2_topic").as_string(), 10,
                                                             [this](const std::shared_ptr<Src2T> src_msg) {
                                                               last_src2_ = std::move(src_msg);
                                                               new_src2_recieved_ = true;
                                                             });

  obs_srv_ = this->create_service<SrvT>(
      "observe", [this, obs_from_src, src2_size](const std::shared_ptr<rmw_request_id_t> request_header,
                                                 const std::shared_ptr<typename SrvT::Request> request,
                                                 const std::shared_ptr<typename SrvT::Response> response) {
        std::vector<std::shared_ptr<Src2T>> src2_history;
        for (int i = 0; i < src2_size; i++)
        {
          while (!new_src2_recieved_ && rclcpp::ok())
          {
            executor_.spin_once();
          }
          src2_history.push_back(last_src2_);
          new_src2_recieved_ = false;
        }
        auto observation = std::make_shared<ObsT>();
        *observation = obs_from_src(*last_src1_, src2_history);
        respond(observation, this->get_parameter("publish").as_bool(), response);
      });
}

template class Observer<ros_dgl_interfaces::srv::ObserveCloud, sensor_msgs::msg::PointCloud2,
                        sensor_msgs::msg::PointCloud2>::Observer;

}  // namespace ros_dgl