// Conversions from vector to pose

#include <vector>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

namespace dgl_ros
{
namespace util
{
geometry_msgs::msg::Pose poseFromXYZRPY(std::vector<double> pose);

Eigen::Isometry3d IsometryFromXYZRPY(std::vector<double> pose);
}  // namespace util
}  // namespace dgl_ros