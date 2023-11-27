// Conversions from vector to pose

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

namespace dgl
{
namespace util
{
geometry_msgs::msg::Pose poseFromXYZRPY(std::vector<double> pose);

Eigen::Isometry3d isometryFromXYZRPY(std::vector<double> pose);
}  // namespace util
}  // namespace dgl