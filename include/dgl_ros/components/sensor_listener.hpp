// TODO(speralta): Consider renaming to sensor_to_observation
/** 
 * @file sensor_listener.hpp
 * @brief SensorListener listens to a sensor topic and publishes the processed data to a new topic
 * with a callback function.
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <queue>

// TODO(speralta): Have buffer queue for sub to write to and pub to read from. 
// Callback should be std::function<ObsMsgT(const std::shared_ptr<SensorMsgT>&)>
template <typename SensorMsgT, typename ObsMsgT>
class SensorListener : public rclcpp::Node
{
public:
  typedef std::function<void(typename rclcpp::Publisher<ObsMsgT>::SharedPtr, const std::shared_ptr<SensorMsgT>&)>
      ListenerCallback;
/**
 * @brief Construct a new Sensor Listener object
 * 
 * @param input_topic 
 * @param output_topic 
 * @param callback Function that takes in a publisher, and sensor data message and publishes an  to the output topic
 */
  SensorListener(const std::string& input_topic, const std::string& output_topic, ListenerCallback callback);

private:
  typename rclcpp::Publisher<SensorMsgT>::SharedPtr sensor_pub_;
  typename rclcpp::Subscription<SensorMsgT>::SharedPtr sensor_sub_;
};
