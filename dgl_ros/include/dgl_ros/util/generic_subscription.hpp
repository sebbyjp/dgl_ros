#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <queue>
#include <memory>
#include <any>
namespace dgl
{
namespace util
{
/**
 * @brief Adds a subscription that stores message in a shared_ptr argument and sets
 *  recieved_observation to true.
 *
 * @tparam SrcT
 * @param node
 * @param topic
 * @param callback_group
 * @param src_msg Pointer to message to store subscription in.
 * @param received_observation
 * @return std::shared_ptr<rclcpp::Subscription<SrcT>>
 */
template <typename SrcT>
std::shared_ptr<rclcpp::Subscription<SrcT>> generic_subscription(rclcpp::Node* node, const std::string& topic,
                                                                 rclcpp::CallbackGroup::SharedPtr callback_group,
                                                                 SrcT* src_msg, bool* received_observation)
{
  auto callback = [node, src_msg, received_observation](const std::shared_ptr<SrcT> msg) {
    *src_msg = *msg;
    *received_observation = true;
    RCLCPP_WARN(node->get_logger(), "Received observation!");
  };
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;
  return node->create_subscription<SrcT>(topic, rclcpp::SensorDataQoS(), callback, options);
}

/**
 * @brief Creates and returns tuple of generic subscriptions.
 *
 * @tparam Tuple
 * @tparam Is compile-time sequence of integers.
 * @param node
 * @param src_topics
 * @param callback_group
 * @param recieved_first_srcs
 * @param src_msgs
 * @return auto
 */
template <typename Tuple, size_t... Is>
auto addSubscriptions(rclcpp::Node* node, const std::array<std::string, sizeof...(Is)>& src_topics,
                      rclcpp::CallbackGroup::SharedPtr callback_group, Tuple& src_msgs,
                      std::array<bool, sizeof...(Is)>& recieved_first_srcs, std::index_sequence<Is...>)
{
  return std::make_tuple(generic_subscription(node, std::get<Is>(src_topics), callback_group,
                                              std::get<Is>(src_msgs).get(), &std::get<Is>(recieved_first_srcs))...);
}
}  // namespace util
}  // namespace dgl
