// Conversions from vector to pose and isometry

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <ros_dgl/util/geometry.hpp>

namespace ros_dgl
{
namespace util
{
geometry_msgs::msg::Pose poseFromXYZRPY(std::vector<double> pose)
{
  geometry_msgs::msg::Pose p;
  p.position.x = pose[0];
  p.position.y = pose[1];
  p.position.z = pose[2];
  Eigen::Quaterniond q = Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

Eigen::Isometry3d IsometryFromXYZRPY(std::vector<double> pose)
{
  Eigen::Translation3d translate(Eigen::Vector3d(pose[0], pose[1], pose[2]));
  Eigen::Quaterniond q = Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ());
  return translate * q;
}
}  // namespace util
}  // namespace ros_dgl