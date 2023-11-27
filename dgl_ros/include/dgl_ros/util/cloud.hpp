
#pragma once

// C++
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

/**
 * @brief Segments objects from table plane
 * @param [out] cloud - Segemented point cloud XYZRGB
 */

namespace dgl
{
namespace util
{
namespace cloud
{
void removeTable(PointCloudRGB::Ptr cloud);

/**
 * @brief Removes points outside the specified cartesian limits
 * @param xyz_lower - 3dim vector x,y,z lower limits on cloud
 * @param xyz_upper - 3dim vector x,y,z upper limits on cloud
 * @param [out] cloud - cloud XYZRGB
 */
void passThroughFilter(const std::vector<double>& xyz_lower, const std::vector<double>& xyz_upper,
                       PointCloudRGB::Ptr cloud);
}  // namespace cloud
}  // namespace util

}  // namespace dgl