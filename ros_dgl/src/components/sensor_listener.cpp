#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ros_dgl/components/sensor_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>

using std::placeholders::_1;
using std::placeholders::_2;

template <class SensorMsgT, class ObsMsgT>
SensorListener<SensorMsgT, ObsMsgT>::SensorListener(const std::string& input_topic, const std::string& output_topic,
                                            SensorListener<SensorMsgT, ObsMsgT>::ListenerCallback callback)
  : rclcpp::Node("sensor_listener")
{
  sensor_pub_ = this->create_publisher<SensorMsgT>(output_topic, 10);
  sensor_sub_ = this->create_subscription<ObsMsgT>(
      input_topic, 10, [this, callback](const std::shared_ptr<SensorMsgT> msg) { callback(sensor_pub_, msg); });
}

// Explicit template instantiation
// TODO(speralta): Consider automating instantiation for all types ?
// https://stackoverflow.com/questions/50338955/how-can-i-concisely-write-a-lot-of-explicit-function-template-instantiations
template class SensorListener<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;