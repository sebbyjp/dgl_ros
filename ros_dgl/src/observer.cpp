#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <queue>
#include "ros_dgl/observer.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>


// Questions
// Should action_producer request observations to begin being made?
// OR should obs be made continuously
namespace ros_dgl
{
template <class ObsT, class... SrcTs>
Observer<ObsT, SrcTs...>::Observer(const rclcpp::NodeOptions& options,
                               std::function<std::unique_ptr<ObsT>(const SrcTs&...)> obs_from_srcs, std::initializer_list<std::string> src_topics) : Observer(options, src_topics)
 
{
  obs_from_srcs_ = obs_from_srcs;
  src_msgs_ = std::make_tuple(std::make_unique<SrcTs>()...);
  src_subs_ = std::tuple<typename rclcpp::Subscription<SrcTs>::SharedPtr...>();
  int i = 0;
  for (const auto& src_topic: src_topics)
  {
    auto callback = [this, i](const std::shared_ptr<typename std::remove_pointer<decltype(std::get<i>(src_msgs_))>::type> msg) {
        TupleAssigner<decltype(src_msgs_), decltype(msg), sizeof...(SrcTs)>::assign_tuple(src_msgs_, msg, i);
    };
   TupleAssigner<decltype(src_subs_)>, this->create_subscription(<typename std::remove_pointer<decltype(std::get<i>(src_msgs_))>::type>(src_topic, 10, callback));
    i++;
  }
    obs_pub_ = this->create_publisher<ObsT>("observation", 10);  // For debugging.

  // auto callback = [this, obs_from_src](const std::shared_ptr<SrcT> msg) {
  //   std::unique_lock lock(obs_mutex_, std::try_to_lock);
  //   if (!lock.owns_lock())
  //   {
  //     RCLCPP_WARN_ONCE(this->get_logger(), "Observer is busy. Skipping observation.");
  //     return;
  //   }
    
  //   last_observation_ = std::move(obs_from_src(*msg));

  //   if (this->get_parameter("publish").as_bool())
  //   {
  //     obs_pub_->publish(*last_observation_);
  //   }
  // };
  // src_sub_ = this->create_subscription<SrcT>(this->get_parameter("src_topic").as_string(), 10, callback);
}

// helper function to print a tuple of any size

template<class Tuple, class V, std::size_t N>
struct TupleAssigner
{
   static void assign_tuple(Tuple& t, V& v, int i)
    {
        TupleAssigner<Tuple, V, N - 1>::assign_tuple(t, v, i);
        if (i == N-1) {
         std::get<N-1>(t) = std::move(v);
        }
    }
};

template<class Tuple, class V>
struct TupleAssigner<Tuple, V, 1>
{
    static void assign_tuple(Tuple& t, V& v, int i)
    {
        if (i == 0) {
         std::get<0>(t) = std::move(v);
        }
    }
};

// template<class Tuple, std::size_t N>
// struct TupleGetter
// {
//    static std::tuple_element_t get(Tuple& t, int i)
//     {
//         TupleGetter<Tuple, N - 1>::get(t, i);
//         if (i == N-1) {
//           return std::get<N-1>(t);
//         }
//     }
// };

template class Observer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>::Observer;

}  // namespace ros_dgl